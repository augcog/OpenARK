#include "D435iCamera.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
//#include <direct.h>
#include <thread>
#include "glfwManager.h"
#include "Util.h"
#include "SaveFrame.h"
#include "Types.h"
#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Integration/MovingTSDFVolume.h"
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"
#include <map>

using namespace ark;

std::shared_ptr<open3d::geometry::RGBDImage> generateRGBDImageFromCV(cv::Mat color_mat, cv::Mat depth_mat) {

	int height = 480;
	int width = 640;

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

	auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*color_im, *depth_im, 1000.0, 2.3, false);
	return rgbd_image;
}

//TODO: check ScalableTSDFVolume.cpp and UniformTSDFVolume.cpp, implement deintegration
//void deintegrate(int frame_id, SaveFrame * saveFrame, open3d::integration::ScalableTSDFVolume * tsdf_volume, open3d::camera::PinholeCameraIntrinsic intr) {
//	RGBDFrame frame = saveFrame->frameLoad(frame_id);
//
//	if (frame.frameId == -1) {
//		cout << "deintegration failed with frameload fail " << frame_id << endl;
//		return;
//	}
//
//	auto rgbd_image = generateRGBDImageFromCV(frame.imRGB, frame.imDepth);
//
//	cv::Mat pose = frame.mTcw.inv();
//
//	Eigen::Matrix4d eigen_pose;
//
//	for (int i = 0; i < 4; i++) {
//		for (int k = 0; k < 4; k++) {
//			eigen_pose(i, k) = pose.at<float>(i, k);
//		}
//	}
//
//	tsdf_volume->Deintegrate(*rgbd_image, intr, eigen_pose);
//}


//TODO: loop closure handler calling deintegration
int main(int argc, char **argv)
{

	if (argc > 5) {
		std::cerr << "Usage: ./" << argv[0] << " configuration-yaml-file [vocabulary-file] [skip-first-seconds]" << std::endl
			<< "Args given: " << argc << std::endl;
		return -1;
	}

	google::InitGoogleLogging(argv[0]);

	okvis::Duration deltaT(0.0);
	if (argc == 5) {
		deltaT = okvis::Duration(atof(argv[4]));
	}

	// read configuration file
	std::string configFilename;
	if (argc > 1) configFilename = argv[1];
	else configFilename = util::resolveRootPath("config/d435i_intr.yaml");

	std::string vocabFilename;
	if (argc > 2) vocabFilename = argv[2];
	else vocabFilename = util::resolveRootPath("config/brisk_vocab.bn");

	OkvisSLAMSystem slam(vocabFilename, configFilename);

	std::string frameOutput;
	if (argc > 3) frameOutput = argv[3];
	else frameOutput = "./frames/";

	cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

	SaveFrame * saveFrame = new SaveFrame(frameOutput);

	//setup display
	if (!MyGUI::Manager::init())
	{
		fprintf(stdout, "Failed to initialize GLFW\n");
		return -1;
	}

	printf("Camera initialization started...\n");
	fflush(stdout);
	D435iCamera camera;
	camera.start();

	printf("Camera-IMU initialization complete\n");
	fflush(stdout);

	//run until display is closed
	okvis::Time start(0.0);
	int id = 0;



	int frame_counter = 1;
	bool do_integration = true;

	FrameAvailableHandler saveFrameHandler([&saveFrame, &frame_counter, &do_integration](MultiCameraFrame::Ptr frame) {
		if (!do_integration || frame_counter % 3 != 0) {
			return;
		}
		cv::Mat imRGB;
		cv::Mat imDepth;

		frame->getImage(imRGB, 3);

		frame->getImage(imDepth, 4);

		Eigen::Matrix4d transform(frame->T_WS());


		saveFrame->frameWrite(imRGB, imDepth, transform, frame->frameId_);
	});

	slam.AddFrameAvailableHandler(saveFrameHandler, "saveframe");

	open3d::integration::MovingTSDFVolume * tsdf_volume = new open3d::integration::MovingTSDFVolume(0.025, 0.05, open3d::integration::TSDFVolumeColorType::RGB8, 2);

	//intrinsics need to be set by user (currently does not read d435i_intr.yaml)
	auto intr = open3d::camera::PinholeCameraIntrinsic(640, 480, 615.885, 616.099, 323.081, 237.475);

	FrameAvailableHandler tsdfFrameHandler([&tsdf_volume, &frame_counter, &do_integration, intr](MultiCameraFrame::Ptr frame) {
		if (!do_integration || frame_counter % 3 != 0) {
			return;
		}

		cout << "Integrating frame number: " << frame->frameId_ << endl;

		cv::Mat color_mat;
		cv::Mat depth_mat;


		frame->getImage(color_mat, 3);
		frame->getImage(depth_mat, 4);

		auto rgbd_image = generateRGBDImageFromCV(color_mat, depth_mat);

		tsdf_volume->Integrate(*rgbd_image, intr, frame->T_WS().inverse());
	});

	slam.AddFrameAvailableHandler(tsdfFrameHandler, "tsdfframe");

	MyGUI::MeshWindow mesh_win("Mesh Viewer", 1200, 1200);
	MyGUI::Mesh mesh_obj("mesh");

	mesh_win.add_object(&mesh_obj);

	std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>,
		Eigen::Matrix4d>> vis_mesh;

	FrameAvailableHandler meshHandler([&tsdf_volume, &frame_counter, &do_integration, &vis_mesh, &mesh_obj](MultiCameraFrame::Ptr frame) {
		/*if (!do_integration || frame_counter % 30 != 1) {
			return;
		}*/

		if (frame_counter % 30 != 1)
			return;
			
		vis_mesh = tsdf_volume->GetTriangleMeshes();

		printf("new mesh extracted, sending to mesh obj\n");

		int number_meshes = mesh_obj.get_number_meshes();

		//if (false) {
		//only update the active mesh
		if (number_meshes == vis_mesh.size()) {
			auto active_mesh = vis_mesh[vis_mesh.size() - 1];
			mesh_obj.update_active_mesh(active_mesh.first->vertices_, active_mesh.first->vertex_colors_, active_mesh.first->triangles_, active_mesh.second);

			std::vector<Eigen::Matrix4d> mesh_transforms;

			for (int i = 0; i < vis_mesh.size(); ++i) {
				mesh_transforms.push_back(vis_mesh[i].second);
			}
				
			mesh_obj.update_transforms(mesh_transforms);
		}
		else {
			std::vector<std::vector<Eigen::Vector3d>> mesh_vertices;
			std::vector<std::vector<Eigen::Vector3d>> mesh_colors;
			std::vector<std::vector<Eigen::Vector3i>> mesh_triangles;
			std::vector<Eigen::Matrix4d> mesh_transforms;

			for (int i = 0; i < vis_mesh.size(); ++i) {
				auto mesh = vis_mesh[i];
				mesh_vertices.push_back(mesh.first->vertices_);
				mesh_colors.push_back(mesh.first->vertex_colors_);
				mesh_triangles.push_back(mesh.first->triangles_);
				mesh_transforms.push_back(mesh.second);

				//cout << mesh.first->vertices_.size() << endl;
				//cout << mesh.first->triangles_.size() << endl;
			}


			mesh_obj.update_mesh_vector(mesh_vertices, mesh_colors, mesh_triangles, mesh_transforms);
		}	


	});

	slam.AddFrameAvailableHandler(meshHandler, "meshupdate");

	/*std::shared_ptr<open3d::geometry::TriangleMesh> vis_mesh;
	FrameAvailableHandler meshHandler([&tsdf_volume, &frame_counter, &do_integration, &vis_mesh, &mesh_obj](MultiCameraFrame::Ptr frame) {
		if (!do_integration || frame_counter % 30 != 1) {
			return;
		}
		vis_mesh = tsdf_volume->ExtractCurrentTriangleMesh();
		mesh_obj.update_mesh(vis_mesh->vertices_, vis_mesh->vertex_colors_, vis_mesh->triangles_);
	});

	slam.AddFrameAvailableHandler(meshHandler, "meshupdate");*/

	FrameAvailableHandler viewHandler([&mesh_obj, &tsdf_volume, &mesh_win, &frame_counter](MultiCameraFrame::Ptr frame) {
		Eigen::Affine3d transform(frame->T_WS());
		mesh_obj.set_transform(transform.inverse());
	});
	
	slam.AddFrameAvailableHandler(viewHandler, "viewhandler");

	KeyFrameAvailableHandler updateKFHandler([&tsdf_volume](MultiCameraFrame::Ptr frame) {
		MapKeyFrame::Ptr kf = frame->keyframe_;
		tsdf_volume->SetLatestKeyFrame(kf->T_WS(), kf->frameId_);
	});

	slam.AddKeyFrameAvailableHandler(updateKFHandler, "updatekfhandler");

	LoopClosureDetectedHandler loopHandler([&tsdf_volume, &slam, &frame_counter](void) {

		printf("loop closure detected!!!!!!!\n");
		std::shared_ptr<open3d::geometry::TriangleMesh> write_mesh = tsdf_volume->ExtractTotalTriangleMesh();
		open3d::io::WriteTriangleMeshToPLY("mesh_" + std::to_string(frame_counter) + ".ply", *write_mesh, false, false, true, true, false, false);

		std::vector<int> frameIdOut;
		std::vector<Eigen::Matrix4d> traj;

		slam.getMappedTrajectory(frameIdOut, traj);

		std::map<int, Eigen::Matrix4d> keyframemap;

		for (int i = 0; i < frameIdOut.size(); i++) {
			keyframemap.insert(std::pair<int, Eigen::Matrix4d>(frameIdOut[i], traj[i]));
		}

		tsdf_volume->UpdateKeyFrames(keyframemap);
	});

	slam.AddLoopClosureDetectedHandler(loopHandler, "loophandler");

	// thread *app = new thread(application_thread);

	cv::namedWindow("image");

	while (MyGUI::Manager::running()) {

		//printf("test\n");
		//Update the display
		MyGUI::Manager::update();

		//Get current camera frame
		MultiCameraFrame::Ptr frame(new MultiCameraFrame);
		camera.update(*frame);

		//Get or wait for IMU Data until current frame 
		//std::cout << "frame: " << frame.timestamp_ << std::endl;
		std::vector<ImuPair> imuData;
		camera.getImuToTime(frame->timestamp_, imuData);
		//std::cout << "numimu: " << imuData.size() << std::endl;

		//Add data to SLAM system
		slam.PushIMU(imuData);
		slam.PushFrame(frame);


		frame_counter++;

		cv::Mat imRGB;
		frame->getImage(imRGB, 3);

		cv::Mat imBGR;

		cv::cvtColor(imRGB, imBGR, CV_RGB2BGR);

		cv::imshow("image", imBGR);

		int k = cv::waitKey(4);
		if (k == ' ') {
			do_integration = !do_integration;
			if (do_integration) {
				std::cout << "----INTEGRATION ENABLED----" << endl;
			}
			else {
				std::cout << "----INTEGRATION DISABLED----" << endl;
			}
		}
		
		if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC

	}

	cout << "updating transforms" << endl;

	std::vector<int> frameIdOut;
	std::vector<Eigen::Matrix4d> traj;

	slam.getMappedTrajectory(frameIdOut, traj);

	std::map<int, Eigen::Matrix4d> keyframemap;

	for (int i = 0; i < frameIdOut.size(); i++) {
		keyframemap.insert(std::pair<int, Eigen::Matrix4d>(frameIdOut[i], traj[i]));
	}

	saveFrame->updateTransforms(keyframemap);

	cout << "getting mesh" << endl;

	//make sure to add these back later |

	std::shared_ptr<open3d::geometry::TriangleMesh> write_mesh = tsdf_volume->ExtractTotalTriangleMesh();

	//const std::vector<std::shared_ptr<const open3d::geometry::Geometry>> mesh_vec = { mesh };

	//open3d::visualization::DrawGeometries(mesh_vec);

	open3d::io::WriteTriangleMeshToPLY("mesh.ply", *write_mesh, false, false, true, true, false, false);

	printf("\nTerminate...\n");
	// Clean up
	slam.ShutDown();
	printf("\nExiting...\n");
	return 0;
}
