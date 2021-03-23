#include "SegmentedMesh.h"

namespace ark {

	SegmentedMesh::SegmentedMesh(double voxel_length,
	               double sdf_trunc,
	               open3d::pipelines::integration::TSDFVolumeColorType color_type,
	               double block_length)
	:	block_length_(block_length),
		voxel_length_(voxel_length),
		sdf_trunc_(sdf_trunc),
		color_type_(color_type){
			Eigen::Vector3i * temp = &current_block;
			temp = NULL;
			active_volume = new open3d::pipelines::integration::ScalableTSDFVolume(voxel_length_,
	               sdf_trunc_,
	               color_type_);
		}

	void SegmentedMesh::Reset() {
		active_volume->Reset();
	}

	void SegmentedMesh::Integrate(
		const open3d::geometry::RGBDImage &image,
		const open3d::camera::PinholeCameraIntrinsic &intrinsic,
		const Eigen::Matrix4d &extrinsic) { //T_WS.inverse()
		
		if ((std::chrono::system_clock::now() - latest_loop_closure).count() < time_threshold) {
			printf("too recent of a loop closure, skipping integration.\n");
			return;
		}

		UpdateActiveVolume(extrinsic);

		Eigen::Matrix4d transform_kf_coords = Eigen::Matrix4d::Identity();
		transform_kf_coords = extrinsic * active_volume_keyframe->T_WC(3); //F_WS.inverse()*K_WS --- F_WS*K_WS.inverse() 
		
		active_volume->Integrate(image, intrinsic, transform_kf_coords);

	}

	void SegmentedMesh::StartNewBlock() {
		start_new_block = true;
	}


	void SegmentedMesh::SetActiveMapIndex(int map_index) {
		active_map_index = map_index;
	}

	void SegmentedMesh::DeleteMapsAfterIndex(int map_index) {
		std::map<int, int> index_map;

		for (int i = 0; i < completed_meshes.size(); i++) {
			if (completed_meshes[i]->mesh_map_index <= active_map_index) {
				continue;
			}
			if (index_map.count(completed_meshes[i]->mesh_map_index) != 0) {
				completed_meshes[i]->mesh_map_index = index_map[completed_meshes[i]->mesh_map_index];
			} else {
				index_map[completed_meshes[i]->mesh_map_index] = archive_index;
				completed_meshes[i]->mesh_map_index = archive_index;
				archive_index--;
			}
		}

		active_map_index = map_index;
	}

	void SegmentedMesh::UpdateActiveVolume(Eigen::Matrix4d extrinsic) {

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


		active_volume = new open3d::pipelines::integration::ScalableTSDFVolume(voxel_length_, sdf_trunc_, color_type_);
		active_volume_keyframe = latest_keyframe;
		active_volume_map_index = active_map_index;

		current_block = block_loc;
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
		latest_keyframe = frame;

		if (active_volume_keyframe == NULL) {
			printf("init first keyframe\n");
			active_volume_keyframe = latest_keyframe;
		}
	}

	//returns a vector of mesh in its own local coords and associated viewing transform (including active volume)
	std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>> SegmentedMesh::GetTriangleMeshes() {
		std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>> ret;
		for (auto completed_mesh : completed_meshes) {
			ret.push_back(std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>(completed_mesh->mesh, completed_mesh->keyframe->T_WC(3)));
		}

		ret.push_back(std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, Eigen::Matrix4d>(ExtractCurrentTriangleMesh(), active_volume_keyframe->T_WC(3)));

		return ret;
	}



	//returns all meshes placed into one TriangleMesh (vertices, vertex colors, triangles)
	std::shared_ptr<open3d::geometry::TriangleMesh>
		SegmentedMesh::ExtractTotalTriangleMesh() {

			printf("extracting triangle meshes\n");

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
		for (auto mesh_unit : completed_meshes) {
			t.push_back(mesh_unit->keyframe->frameId_);
		}
		return t;
	}

	//only updates the active mesh
	void SegmentedMesh::Render(std::vector<std::vector<Eigen::Vector3d>> &mesh_vertices, std::vector<std::vector<Eigen::Vector3d>> &mesh_colors, 
		std::vector<std::vector<Eigen::Vector3i>> &mesh_triangles, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &mesh_transforms, std::vector<int> &mesh_enabled) { // Moon : Rule 2.c?? fixed.Cause 4 = Cause 2.b + Cause 3. This line might be wrong, const + & or & alone might be added

		//don't have any key frames, don't have any blocks
		if (active_volume_keyframe == NULL) {
			return;
		}

		//clear out active mesh, clear mesh enabled, only run if vectors are populated
		if (mesh_vertices.size() > 0) {
			mesh_vertices.pop_back();
			mesh_colors.pop_back();
			mesh_triangles.pop_back();
			mesh_transforms.pop_back();
			mesh_enabled.clear();
		}

		//always update transforms in case of loop closure
		mesh_transforms.clear();
		for (int i = 0; i < completed_meshes.size(); i++) {
			auto mesh_unit = completed_meshes[i];
			mesh_transforms.push_back(mesh_unit->keyframe->T_WC(3));
		}

		//updated completed meshes not in the vectors
		if (completed_meshes.size() > mesh_vertices.size()) {
			for (int i = mesh_vertices.size(); i < completed_meshes.size(); i++) {

				auto mesh_unit = completed_meshes[i];

				mesh_vertices.push_back(mesh_unit->mesh->vertices_);
				mesh_colors.push_back(mesh_unit->mesh->vertex_colors_);
				mesh_triangles.push_back(mesh_unit->mesh->triangles_);

			}
		}

		auto mesh = active_volume->ExtractTriangleMesh();

		mesh_vertices.push_back(mesh->vertices_);
		mesh_colors.push_back(mesh->vertex_colors_);
		mesh_triangles.push_back(mesh->triangles_);
		mesh_transforms.push_back(active_volume_keyframe->T_WC(3));


		for (int i = 0; i < completed_meshes.size(); i++) {
			if (completed_meshes[i]->mesh_map_index == active_map_index) {
				mesh_enabled.push_back(1);
			} else {
				mesh_enabled.push_back(0);
			}
		}

		//active volume is always visible
		mesh_enabled.push_back(1);

	}

	void SegmentedMesh::WriteMeshes() {

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

                std::cout << "writing meshes" << std::endl;

		int i = 0;
		for (auto iter = mesh_map.begin(); iter != mesh_map.end(); iter++) {
			open3d::io::WriteTriangleMeshToPLY("mesh" + std::to_string(i++) + ".ply", *(iter->second), false, false, true, true, false, false);
		}
	}


}