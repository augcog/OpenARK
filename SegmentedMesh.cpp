#include "SegmentedMesh.h"

namespace ark {

	std::shared_ptr<open3d::geometry::RGBDImage> generateRGBDImageFromCV(cv::Mat color_mat, cv::Mat depth_mat, double max_depth, int width, int height) {

		auto color_im = std::make_shared<open3d::geometry::Image>();
		color_im->Prepare(width, height, 3, sizeof(uint8_t));

		uint8_t *pi = (uint8_t *)(color_im->data_.data());

		for (int i = 0; i < height; i++) {
			for (int k = 0; k < width; k++) {

				cv::Vec3b pixel = color_mat.at<cv::Vec3b>(i, k);

				*pi++ = pixel[0];
				*pi++ = pixel[1];
				*pi++ = pixel[2];
			}
		}


		auto depth_im = std::make_shared<open3d::geometry::Image>();
		depth_im->Prepare(width, height, 1, sizeof(uint16_t));

		uint16_t * p = (uint16_t *)depth_im->data_.data();

		for (int i = 0; i < height; i++) {
			for (int k = 0; k < width; k++) {
				*p++ = depth_mat.at<uint16_t>(i, k);
			}
		}

		//change the second-to-last parameter here to set maximum distance
		auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*color_im, *depth_im, 1000.0, max_depth, false);
		return rgbd_image;
	}

	void SegmentedMesh::readConfig(std::string& recon_config) {

		cv::FileStorage file;

		struct stat buffer;
		if (stat (recon_config.c_str(), &buffer) == 0) {
			file =  cv::FileStorage(recon_config, cv::FileStorage::READ);
		} else {
			file =  cv::FileStorage();
		}

		std::cout << "Add entries to the intr.yaml to configure 3drecon parameters." << std::endl;

		if (file["Recon_VoxelSize"].isReal()) {
			file["Recon_VoxelSize"] >> voxel_length_;
	  	} else {
			std::cout << "option <Recon_VoxelSize> not found, setting to default 0.03" << std::endl;
			voxel_length_ = 0.03;
		}

		if (file["Recon_SDFTruncate"].isReal()) {
			file["Recon_SDFTruncate"] >> sdf_trunc_;
		} else {
			std::cout << "option <Recon_SDFTruncate> not found, setting to default voxel size * 5" << std::endl;
			sdf_trunc_ = voxel_length_ * 5.;
		}

		if (file["Recon_BlockSize"].isReal()) {
			file["Recon_BlockSize"] >> block_length_;
	  	} else {
			std::cout << "option <Recon_BlockSize> not found, setting to default 2.0" << std::endl;
			block_length_ = 2.0;
		}

		if (file["Recon_MaxDepth"].isReal()) {
			file["Recon_MaxDepth"] >> max_depth_;
	  	} else {
			std::cout << "option <Recon_MaxDepth> not found, setting to default 2.5" << std::endl;
			max_depth_ = 2.5;
		}
	}

	SegmentedMesh::SegmentedMesh(std::string& recon_config, OkvisSLAMSystem& slam, CameraSetup* camera, bool blocking/*= true*/) {
		this->Setup(slam, camera);
		Initialize(recon_config, blocking);
	}

	SegmentedMesh::SegmentedMesh(std::string& recon_config) {
		Initialize(recon_config, false);
	}

	SegmentedMesh::SegmentedMesh() {
		Initialize(std::string(""), false);
	}

	void SegmentedMesh::Initialize(std::string& recon_config, bool blocking) {
		readConfig(recon_config);
		blocking_ = blocking;
		if (blocking) {
			Eigen::Vector3i * temp = &current_block;
			temp = NULL;
		}
		active_volume = new open3d::integration::ScalableTSDFVolume(voxel_length_,
               sdf_trunc_,      
				color_type_);
		do_integration_ = true;

		std::vector<std::vector<Eigen::Vector3d>> mesh_vertices_vec;
		std::vector<std::vector<Eigen::Vector3d>> mesh_colors_vec;
		std::vector<std::vector<Eigen::Vector3i>> mesh_triangles_vec;
		std::vector<Eigen::Matrix4d> mesh_transforms_vec;
		std::vector<int> mesh_enabled_vec;

		this->mesh_vertices = std::make_shared<std::vector<std::vector<Eigen::Vector3d>>>(mesh_vertices_vec);
		this->mesh_colors = std::make_shared<std::vector<std::vector<Eigen::Vector3d>>>(mesh_colors_vec);
		this->mesh_triangles = std::make_shared<std::vector<std::vector<Eigen::Vector3i>>>(mesh_triangles_vec);
		this->mesh_transforms = std::make_shared<std::vector<Eigen::Matrix4d>>(mesh_transforms_vec);
		this->mesh_enabled = std::make_shared<std::vector<int>>(mesh_enabled_vec);

	}

	void SegmentedMesh::Setup(OkvisSLAMSystem& slam, CameraSetup* camera) {

		FrameAvailableHandler updateFrameCounter([&, this] (MultiCameraFrame::Ptr frame) {
			if (this->frame_counter_ > 1000000) {
				this->frame_counter_ = 0;
			}
			this->frame_counter_++;

			if (this->do_integration_ && this->frame_counter_ % this->extraction_frame_stride_ == 0) {
				this->UpdateOutputVectors();
			}
		});

		slam.AddFrameAvailableHandler(updateFrameCounter, "update frame counter");


		KeyFrameAvailableHandler updateKFHandler([&, this](MultiCameraFrame::Ptr frame) {
			MapKeyFrame::Ptr kf = frame->keyframe_;
			this->SetLatestKeyFrame(kf);
		});

		slam.AddKeyFrameAvailableHandler(updateKFHandler, "updatekfhandler");

		SparseMapCreationHandler spcHandler([&, this](int active_map_index) {
			this->SetActiveMapIndex(active_map_index);
			this->StartNewBlock();
		});

		slam.AddSparseMapCreationHandler(spcHandler, "mesh sp creation");

		SparseMapMergeHandler spmHandler([&, this](int merged_map_index, int active_map_index) {
			for (auto completed_mesh : completed_meshes) {
				if (completed_mesh->mesh_map_index == merged_map_index) {
					completed_mesh->mesh_map_index = active_map_index;
				}
			}
			this->SetActiveMapIndex(active_map_index);
			this->StartNewBlock();
		});

		slam.AddSparseMapMergeHandler(spmHandler, "mesh merge");

		std::vector<float> intrinsics = camera->getColorIntrinsics();
		auto size = camera->getImageSize();

		auto intr = open3d::camera::PinholeCameraIntrinsic(size.width, size.height, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);

		this->camera_width_ = size.width;
		this->camera_height_ = size.height;

		FrameAvailableHandler tsdfFrameHandler([&, this, intr](MultiCameraFrame::Ptr frame) {
			if (!this->do_integration_ || this->frame_counter_ % this->integration_frame_stride_ != 0) {
				return;
			}

			std::cout << "Integrating frame number: " << frame->frameId_ << std::endl;

			cv::Mat color_mat;
			cv::Mat depth_mat;


			frame->getImage(color_mat, 3);
			frame->getImage(depth_mat, 4);

			auto rgbd_image = generateRGBDImageFromCV(color_mat, depth_mat, this->max_depth_, this->camera_width_, this->camera_height_);

			this->Integrate(*rgbd_image, intr, frame->T_WC(3).inverse());
		});

		slam.AddFrameAvailableHandler(tsdfFrameHandler, "tsdfframe");
	}

	void SegmentedMesh::Reset() {
		active_volume->Reset();
	}

	void SegmentedMesh::SetIntegrationEnabled(bool enabled) {
		this->do_integration_ = enabled;
	}

	void SegmentedMesh::Integrate(
		const open3d::geometry::RGBDImage &image,
		const open3d::camera::PinholeCameraIntrinsic &intrinsic,
		const Eigen::Matrix4d &extrinsic) { //T_WS.inverse()
		
		if ((std::chrono::system_clock::now() - latest_loop_closure).count() < time_threshold) {
			printf("too recent of a loop closure, skipping integration.\n");
			return;
		}

		Eigen::Matrix4d transform_kf_coords = Eigen::Matrix4d::Identity();
		
		if (blocking_) {

			if (active_volume_keyframe == NULL) {
				std::cout << "Error: No keyframes have been passed in for the blocking algorithm, either call SetLatestKeyFrame or set blocking to false." << std::endl;
				return;
			}

			UpdateActiveVolume(extrinsic);
			transform_kf_coords = extrinsic * active_volume_keyframe->T_WC(3); //F_WS.inverse()*K_WS --- F_WS*K_WS.inverse() 
		} else {
			transform_kf_coords = extrinsic;
		}
		
		active_volume->Integrate(image, intrinsic, transform_kf_coords);
	}

	void SegmentedMesh::StartNewBlock() {

		if (!blocking_) {
			std::cout << "Error: Attempted to start new block but blocking was disabled" << std::endl;
			return;
		}

		start_new_block = true;
	}


	void SegmentedMesh::SetActiveMapIndex(int map_index) {

		if (!blocking_) {
			std::cout << "Error: Attempted to set active map index but blocking was disabled" << std::endl;
			return;
		}

		active_map_index = map_index;
	}

	void SegmentedMesh::UpdateActiveVolume(Eigen::Matrix4d extrinsic) {

		if (!blocking_) {
			std::cout << "Error: Attempted to update active volume but blocking was disabled" << std::endl;
			return;
		}

		//locate which block we are in, origin = center of block (l/2, l/2, l/2)
		auto block_loc = LocateBlock(Eigen::Vector3d(extrinsic(0, 3) - block_length_ / 2, extrinsic(1, 3) - block_length_ / 2, extrinsic(2, 3) - block_length_ / 2));

		if (&current_block == NULL) {
			current_block = block_loc;
		}

		//return if we are still in the same current_block
		if (!start_new_block && block_loc.isApprox(current_block)) {
			return;
		}

		start_new_block = false;

		std::lock_guard<std::mutex> guard(keyFrameLock);

		printf("EXITED BLOCK, NEW BLOCK\n");

		auto completed_mesh = std::make_shared<MeshUnit>();

		auto mesh = active_volume->ExtractTriangleMesh();
		completed_mesh->mesh = mesh;
		completed_mesh->keyframe = active_volume_keyframe;
		completed_mesh->block_loc = current_block;
		completed_mesh->mesh_map_index = active_volume_map_index;
		completed_meshes.push_back(completed_mesh);


		active_volume = new open3d::integration::ScalableTSDFVolume(voxel_length_, sdf_trunc_, color_type_);
		active_volume_keyframe = latest_keyframe;
		active_volume_map_index = active_map_index;

		current_block = block_loc;

		UpdateOutputVectors();
	}

	std::tuple<std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>>, 
			std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>>,
			std::shared_ptr<std::vector<std::vector<Eigen::Vector3i>>>, 
			std::shared_ptr<std::vector<Eigen::Matrix4d>>, std::shared_ptr<std::vector<int>>> SegmentedMesh::GetOutputVectors() {
		return std::make_tuple(mesh_vertices, mesh_colors, mesh_triangles, mesh_transforms, mesh_enabled);
	}

	//combines 2 meshes, places in mesh 1
	void SegmentedMesh::CombineMeshes(std::shared_ptr<open3d::geometry::TriangleMesh>& output_mesh, std::shared_ptr<open3d::geometry::TriangleMesh> mesh_to_combine) {

		std::vector<Eigen::Vector3d> vertices = mesh_to_combine->vertices_;
		std::vector<Eigen::Vector3d> colors = mesh_to_combine->vertex_colors_;
		std::vector<Eigen::Vector3i> triangles = mesh_to_combine->triangles_;

		size_t tri_count = output_mesh->vertices_.size();

		output_mesh->vertices_.insert(output_mesh->vertices_.end(), vertices.begin(), vertices.end());
		output_mesh->vertex_colors_.insert(output_mesh->vertex_colors_.end(), colors.begin(), colors.end());
		for (auto triangle : triangles) {
			Eigen::Vector3i triangle_;
			triangle_(0) = triangle(0) + tri_count;
			triangle_(1) = triangle(1) + tri_count;
			triangle_(2) = triangle(2) + tri_count;
			output_mesh->triangles_.push_back(triangle_);
		}
	}

	// running pointer of current keyframe
	void SegmentedMesh::SetLatestKeyFrame(MapKeyFrame::Ptr frame) {

		if (!blocking_) {
			std::cout << "Error: Attempted to update latest keyframe but blocking was disabled" << std::endl;
			return;
		}

		latest_keyframe = frame;

		if (active_volume_keyframe == NULL) {
			printf("init first keyframe\n");
			active_volume_keyframe = latest_keyframe;
		}
	}

	//returns a vector of mesh in its own local coords and associated viewing transform (including active volume)
	std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>> SegmentedMesh::GetTriangleMeshes() {
		std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>> ret;
		if (blocking_) {
			
			for (auto completed_mesh : completed_meshes) {
				ret.push_back(std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>(completed_mesh->mesh, completed_mesh->keyframe->T_WC(3)));
			}

			ret.push_back(std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>(ExtractCurrentTriangleMesh(), active_volume_keyframe->T_WC(3)));
			
		} else {
			ret.push_back(std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>(ExtractCurrentTriangleMesh(), Eigen::Matrix4d::Identity()));
		}

		return ret;

			
	}



	//returns all meshes placed into one TriangleMesh (vertices, vertex colors, triangles)
	std::shared_ptr<open3d::geometry::TriangleMesh>
		SegmentedMesh::ExtractTotalTriangleMesh() {

			printf("extracting triangle meshes\n");

			if (blocking_) {
				auto mesh_output = std::make_shared<open3d::geometry::TriangleMesh>();

				for (auto mesh_unit : completed_meshes) {
					
					//printf("adding a mesh\n");

					auto completed_mesh = mesh_unit->mesh;
					auto transformed_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

					auto vertices = completed_mesh->vertices_;
					std::vector<Eigen::Vector3d> transformed_vertices;

					for (Eigen::Vector3d vertex : vertices) {

						Eigen::Vector4d v(vertex(0), vertex(1), vertex(2), 1.0);
						v = mesh_unit->keyframe->T_WC(3) * v;
						Eigen::Vector3d transformed_v(v(0), v(1), v(2));

						transformed_vertices.push_back(transformed_v);
					}

					transformed_mesh->vertices_ = transformed_vertices;
					transformed_mesh->vertex_colors_ = completed_mesh->vertex_colors_;
					transformed_mesh->triangles_ = completed_mesh->triangles_;

					CombineMeshes(mesh_output, transformed_mesh);

				}

				
				auto mesh = active_volume->ExtractTriangleMesh();
				for (int i = 0; i < mesh->vertices_.size(); ++i) {
					Eigen::Vector4d v(mesh->vertices_[i](0), mesh->vertices_[i](1), mesh->vertices_[i](2), 1.0);
					v = active_volume_keyframe->T_WC(3) * v;
					Eigen::Vector3d transformed_v(v(0), v(1), v(2));
					mesh->vertices_[i] = transformed_v;
				}

				CombineMeshes(mesh_output, mesh);

				return mesh_output;
			} else {
				return ExtractCurrentTriangleMesh();
			}
	}

	std::shared_ptr<open3d::geometry::TriangleMesh>
		SegmentedMesh::ExtractCurrentTriangleMesh() {
		return active_volume->ExtractTriangleMesh();
				
				
	}

	std::shared_ptr<open3d::geometry::PointCloud>
		SegmentedMesh::ExtractCurrentVoxelPointCloud() {
		return active_volume->ExtractVoxelPointCloud();
	}

	std::vector<int> SegmentedMesh::get_kf_ids() {

		std::vector<int> t;

		if (!blocking_) {
			std::cout << "Error: Tried to retrieve keyframe id's but blocking disabled" << std::endl;
			return t;
		}

		for (auto mesh_unit : completed_meshes) {
			t.push_back(mesh_unit->keyframe->frameId_);
		}
		return t;
	}


	void SegmentedMesh::AddRenderMutex(std::mutex* render_mutex, std::string render_mutex_key) {
		render_mutexes.insert(std::pair<std::string, std::mutex*>(render_mutex_key, render_mutex));
	}

	void SegmentedMesh::RemoveRenderMutex(std::string render_mutex_key) {
		auto iter = this->render_mutexes.find(render_mutex_key);
		if (iter != this->render_mutexes.end()) {
			this->render_mutexes.erase(iter);
		}
	}


	void SegmentedMesh::UpdateOutputVectors() {

		for (auto render_mutex : render_mutexes) {
			std::lock_guard<std::mutex> guard(*(render_mutex.second));
		}

		if (blocking_) {

			//don't have any key frames, don't have any blocks
			if (active_volume_keyframe == NULL) {
				return;
			}

			//clear out active mesh, clear mesh enabled, only run if vectors are populated
			if (mesh_vertices->size() > 0) {
				mesh_vertices->pop_back();
				mesh_colors->pop_back();
				mesh_triangles->pop_back();
				mesh_transforms->pop_back();
				mesh_enabled->clear();
			}

			//always update transforms in case of loop closure
			mesh_transforms->clear();
			for (int i = 0; i < completed_meshes.size(); i++) {
				auto mesh_unit = completed_meshes[i];
				mesh_transforms->push_back(mesh_unit->keyframe->T_WC(3));
			}

			//updated completed meshes not in the vectors
			if (completed_meshes.size() > mesh_vertices->size()) {
				for (int i = mesh_vertices->size(); i < completed_meshes.size(); i++) {

					auto mesh_unit = completed_meshes[i];

					mesh_vertices->push_back(mesh_unit->mesh->vertices_);
					mesh_colors->push_back(mesh_unit->mesh->vertex_colors_);
					mesh_triangles->push_back(mesh_unit->mesh->triangles_);

				}
			}

			auto mesh = active_volume->ExtractTriangleMesh();

			mesh_vertices->push_back(mesh->vertices_);
			mesh_colors->push_back(mesh->vertex_colors_);
			mesh_triangles->push_back(mesh->triangles_);
			mesh_transforms->push_back(active_volume_keyframe->T_WC(3));


			for (int i = 0; i < completed_meshes.size(); i++) {
				if (completed_meshes[i]->mesh_map_index == active_map_index) {
					mesh_enabled->push_back(1);
				} else {
					mesh_enabled->push_back(0);
				}
			}

			//active volume is always visible
			mesh_enabled->push_back(1);

		} else {
			auto mesh = active_volume->ExtractTriangleMesh();

			//clear out active mesh, clear mesh enabled, only run if vectors are populated
			if (mesh_vertices->size() > 0) {
				mesh_vertices->pop_back();
				mesh_colors->pop_back();
				mesh_triangles->pop_back();
				mesh_transforms->pop_back();
				mesh_enabled->clear();
			}

			mesh_vertices->push_back(mesh->vertices_);
			mesh_colors->push_back(mesh->vertex_colors_);
			mesh_triangles->push_back(mesh->triangles_);
			mesh_transforms->push_back(Eigen::Matrix4d::Identity());
			mesh_enabled->push_back(1);
		}
	}

	void SegmentedMesh::WriteMeshes() {

		if (blocking_) {
			//extract active volume
			auto completed_mesh = std::make_shared<MeshUnit>();

			auto mesh = active_volume->ExtractTriangleMesh();
			completed_mesh->mesh = mesh;
			completed_mesh->keyframe = active_volume_keyframe;
			completed_mesh->block_loc = current_block;
			completed_mesh->mesh_map_index = active_volume_map_index;
			completed_meshes.push_back(completed_mesh);

			std::map<int, std::shared_ptr<open3d::geometry::TriangleMesh>> mesh_map;

			for (int i = 0; i < completed_meshes.size(); i++) {
				auto mesh = completed_meshes[i]->mesh;

				std::vector<Eigen::Vector3d> vertices = mesh->vertices_;
				std::vector<Eigen::Vector3d> transformed_vertices;

				for (Eigen::Vector3d vertex : vertices) {

					Eigen::Vector4d v(vertex(0), vertex(1), vertex(2), 1.0);
					v = completed_meshes[i]->keyframe->T_WC(3) * v;
					Eigen::Vector3d transformed_v(v(0), v(1), v(2));

					transformed_vertices.push_back(transformed_v);
				}

				mesh->vertices_ = transformed_vertices;

				int index = completed_meshes[i]->mesh_map_index;

				if (mesh_map.count(index) != 0) {	
					CombineMeshes(mesh_map[index], mesh);
				} else {
					mesh_map[index] = mesh;
				}
			}

			int i = 0;
			for (auto iter = mesh_map.begin(); iter != mesh_map.end(); iter++) {
				open3d::io::WriteTriangleMeshToPLY("mesh" + std::to_string(i++) + ".ply", *(iter->second), false, false, true, true, false, false);
			}

		} else {
			auto mesh = active_volume->ExtractTriangleMesh();
			open3d::io::WriteTriangleMeshToPLY("mesh.ply", *mesh, false, false, true, true, false, false);
		}
	}

}