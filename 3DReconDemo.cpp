#include "D435iCamera.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
//#include <direct.h>
#include <thread>
#include "glfwManager.h"
#include "Util.h"
#include "SaveFrame.h"
#include "Types.h"
#include "SegmentedMesh.h"

using namespace ark;

float voxel_size, block_size, max_depth;
bool save_frames;
int mesh_view_width, mesh_view_height;

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

	//change the second-to-last parameter here to set maximum distance
	auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*color_im, *depth_im, 1000.0, max_depth, false);
	return rgbd_image;
}

void readConfig(std::string& recon_config) {

	cv::FileStorage file(recon_config, cv::FileStorage::READ);

	std::cout << "Add entries to the intr.yaml to configure 3drecon parameters." << std::endl;

	if (file["Recon_VoxelSize"].isReal()) {
		file["Recon_VoxelSize"] >> voxel_size;
  	} else {
		std::cout << "option <3dRecon_VoxelSize> not found, setting to default 0.03" << std::endl;
		voxel_size = 0.03;
	}

	if (file["Recon_BlockSize"].isReal()) {
		file["Recon_BlockSize"] >> block_size;
  	} else {
		std::cout << "option <3dRecon_BlockSize> not found, setting to default 2.0" << std::endl;
		block_size = 2.0;
	}

	if (file["Recon_MaxDepth"].isReal()) {
		file["Recon_MaxDepth"] >> max_depth;
  	} else {
		std::cout << "option <3dRecon_MaxDepth> not found, setting to default 2.5" << std::endl;
		max_depth = 2.5;
	}

	if (file["Recon_SaveFrames"].isInt()) {
		int save;
		file["Recon_SaveFrames"] >> save;
		save_frames = (bool)save;
  	} else {
		std::cout << "option <3dRecon_SaveFrames> not found, setting to default true" << std::endl;
		save_frames = true;
	}

	if (file["Recon_MeshWinWidth"].isInt()) {
		file["Recon_MeshWinWidth"] >> mesh_view_width;
  	} else {
		std::cout << "option <3dRecon_MeshWinWidth> not found, setting to default 1000px" << std::endl;
		mesh_view_width = 1000;
	}

	if (file["Recon_MeshWinHeight"].isInt()) {
		file["Recon_MeshWinHeight"] >> mesh_view_height;
  	} else {
		std::cout << "option <3dRecon_MeshWinHeight> not found, setting to default 1000px" << std::endl;
		mesh_view_height = 1000;
	}

}

int main(int argc, char **argv)
{

	if (argc > 5) {
		std::cerr << "Usage: ./" << argv[0] << " [configuration-yaml-file] [vocabulary-file] [frame output directory]" << std::endl
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

	std::string frameOutput;
	if (argc > 3) frameOutput = argv[3];
	else frameOutput = "./frames/";

	OkvisSLAMSystem slam(vocabFilename, configFilename);

	readConfig(configFilename);

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
	int max_map_index = 0;
	int current_map_index = 0;

	KeyFrameAvailableHandler saveFrameHandler([&saveFrame, &frame_counter, &do_integration, &current_map_index](MultiCameraFrame::Ptr frame) {
		if (!do_integration || frame_counter % 3 != 0) {
			return;
		}

		cv::Mat imRGB;
		cv::Mat imDepth;

		frame->getImage(imRGB, 3);

		frame->getImage(imDepth, 4);

		Eigen::Matrix4d transform(frame->T_WC(3));

		saveFrame->frameWriteMapped(imRGB, imDepth, transform, frame->frameId_, current_map_index);
	});
	
	if (save_frames) {
		slam.AddKeyFrameAvailableHandler(saveFrameHandler, "saveframe");
	}

	SegmentedMesh * mesh = new SegmentedMesh(voxel_size, voxel_size * 5, open3d::pipelines::integration::TSDFVolumeColorType::RGB8, block_size);

	std::vector<float> intrinsics = camera.getColorIntrinsics();
	auto intr = open3d::camera::PinholeCameraIntrinsic(640, 480, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);

	FrameAvailableHandler tsdfFrameHandler([&mesh, &frame_counter, &do_integration, intr](MultiCameraFrame::Ptr frame) {
		if (!do_integration || frame_counter % 3 != 0) {
			return;
		}

		cout << "Integrating frame number: " << frame->frameId_ << endl;

		cv::Mat color_mat;
		cv::Mat depth_mat;


		frame->getImage(color_mat, 3);
		frame->getImage(depth_mat, 4);

		auto rgbd_image = generateRGBDImageFromCV(color_mat, depth_mat);

		mesh->Integrate(*rgbd_image, intr, frame->T_WC(3).inverse());
	});

	slam.AddFrameAvailableHandler(tsdfFrameHandler, "tsdfframe");

	MyGUI::MeshWindow mesh_win("Mesh Viewer", mesh_view_width, mesh_view_height);
	MyGUI::Mesh mesh_obj("mesh", mesh);

	mesh_win.add_object(&mesh_obj);

	std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>,
		Eigen::Matrix4d>> vis_mesh;

	FrameAvailableHandler meshHandler([&mesh_obj, &frame_counter, &do_integration](MultiCameraFrame::Ptr frame) {
		if (!do_integration || frame_counter % 30 != 1) {
			return;
		}

		mesh_obj.update_meshes();

	});

	slam.AddFrameAvailableHandler(meshHandler, "meshupdate");

	FrameAvailableHandler viewHandler([&mesh_obj, &mesh_win, &frame_counter, &do_integration](MultiCameraFrame::Ptr frame) {
		Eigen::Affine3d transform(frame->T_WC(3));
		mesh_obj.set_transform(transform.inverse());
		if (mesh_win.clicked()) {
			do_integration = !do_integration;
			if (do_integration) {
				std::cout << "----INTEGRATION ENABLED----" << endl;
			}
			else {
				std::cout << "----INTEGRATION DISABLED----" << endl;
			}
		}
	});
	
	slam.AddFrameAvailableHandler(viewHandler, "viewhandler");

	KeyFrameAvailableHandler updateKFHandler([&mesh](MultiCameraFrame::Ptr frame) {
		MapKeyFrame::Ptr kf = frame->keyframe_;
		mesh->SetLatestKeyFrame(kf);
	});

	slam.AddKeyFrameAvailableHandler(updateKFHandler, "updatekfhandler");

	SparseMapDeletionHandler spdHandler([&mesh, &max_map_index, &current_map_index](int active_map_index) {
		mesh->DeleteMapsAfterIndex(active_map_index);
		mesh->StartNewBlock();
		current_map_index = active_map_index;
	});

	slam.AddSparseMapDeletionHandler(spdHandler, "mesh sp deletion");

	SparseMapCreationHandler spcHandler([&mesh, &max_map_index, &current_map_index](int active_map_index) {
		mesh->SetActiveMapIndex(active_map_index);
		mesh->StartNewBlock();
		current_map_index = max_map_index + 1;
		max_map_index++;
	});

	slam.AddSparseMapCreationHandler(spcHandler, "mesh sp creation");

	cv::namedWindow("image");

	while (MyGUI::Manager::running()) {

		//Update the display
		MyGUI::Manager::update();

		//Get current camera frame
		MultiCameraFrame::Ptr frame(new MultiCameraFrame);
		camera.update(*frame);

		//Get or wait for IMU Data until current frame 
		std::vector<ImuPair> imuData;
		camera.getImuToTime(frame->timestamp_, imuData);

		//Add data to SLAM system
		slam.PushIMU(imuData);
		slam.PushFrame(frame);


		frame_counter++;

		cv::Mat imRGB;
		frame->getImage(imRGB, 3);

		cv::Mat imBGR;

		cv::cvtColor(imRGB, imBGR, CV_RGB2BGR);

		cv::imshow("image", imBGR);

		int k = cv::waitKey(2);
		
		if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC

	}

	cout << "updating transforms" << endl;

	std::vector<int> frameIdOut;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> traj; // Moon, Rule 2 , fixed

	slam.getMappedTrajectory(frameIdOut, traj);

	std::map<int, Eigen::Matrix4d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix4d>>> keyframemap; // Moon, Rule 2, fixed

	for (int i = 0; i < frameIdOut.size(); i++) {
		keyframemap[frameIdOut[i]] = traj[i];
	}

	saveFrame->updateTransforms(keyframemap);

	mesh->WriteMeshes();

	printf("\nTerminate...\n");
	// Clean up
	slam.ShutDown();
	printf("\nExiting...\n");
	return 0;
}
