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
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"

using namespace ark;

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

	//Window for displaying the path
	
	
	/*
	MyGUI::ARCameraWindow ar_win("AR Viewer", 640 * 2.5, 480 * 2.5, GL_RGB, GL_UNSIGNED_BYTE, 6.16403320e+02, 6.16171021e+02, 3.18104584e+02, 2.33643127e+02, 0.01, 100);
	traj_win.set_pos(640 * 2.5, 100);
	ar_win.set_pos(0, 100);
	MyGUI::Path path1("path1", Eigen::Vector3d(1, 0, 0));
	MyGUI::Axis axis1("axis1", .1);
	MyGUI::Axis axis2("axis2", 1);
	MyGUI::Grid grid1("grid1", 10, 1);
	traj_win.add_object(&path1);
	traj_win.add_object(&axis1);
	traj_win.add_object(&axis2);
	traj_win.add_object(&grid1);
	ar_win.add_object(&axis1);
	std::vector<MyGUI::Object*> cubes;
	std::vector<Eigen::Matrix4d> T_K_cubes;
	std::vector<MapKeyFrame::Ptr> K_cubes;


	//Recieves output from SLAM system and displays to the screen
	FrameAvailableHandler handler([&path1, &axis2, &ar_win, &cubes, &T_K_cubes, &K_cubes](MultiCameraFrame::Ptr frame) {
		Eigen::Affine3d transform(frame->T_WC(3));
		path1.add_node(transform.translation());
		axis2.set_transform(transform);
		ar_win.set_camera(transform);
		ar_win.set_image(frame->images_[3]);
		if (ar_win.clicked()) {

			std::string cube_name = std::string("CubeNum") + std::to_string(cubes.size());
			MyGUI::Object* obj = new MyGUI::Cube(cube_name, 0.1, 0.1, 0.1);
			obj->set_transform(transform);
			cubes.push_back(obj);
			T_K_cubes.push_back(frame->T_KS_);
			K_cubes.push_back(frame->keyframe_);
			std::cout << "Adding cube " << cube_name << std::endl;
			ar_win.add_object(obj); //NOTE: this is bad, should change objects to shared_ptr
		}
	});
	slam.AddFrameAvailableHandler(handler, "mapping");

	KeyFrameAvailableHandler kfHandler([](MultiCameraFrame::Ptr frame) {
		frame->saveSimple("map_images/");

	});
	//slam.AddKeyFrameAvailableHandler(kfHandler, "saving");

	LoopClosureDetectedHandler loopHandler([&slam, &path1, &cubes, &T_K_cubes, &K_cubes](void) {
		std::vector<Eigen::Matrix4d> traj;
		slam.getTrajectory(traj);
		path1.clear();
		for (size_t i = 0; i<traj.size(); i++) {
			path1.add_node(traj[i].block<3, 1>(0, 3));
		}
		for (size_t i = 0; i<cubes.size(); i++) {
			if (K_cubes[i] != nullptr)
				cubes[i]->set_transform(Eigen::Affine3d(K_cubes[i]->T_WS()*T_K_cubes[i]));

		}

	});
	//slam.AddLoopClosureDetectedHandler(loopHandler, "trajectoryUpdate");
	

	*/

	//run until display is closed
	okvis::Time start(0.0);
	int id = 0;


	SaveFrame * saveFrame = new SaveFrame(frameOutput);

	FrameAvailableHandler saveFrameHandler([&saveFrame](MultiCameraFrame::Ptr frame) {

		cv::Mat imRGB;
		cv::Mat imDepth;

		frame->getImage(imRGB, 3);

		frame->getImage(imDepth, 4);

		Eigen::Matrix4d transform(frame->T_WC(3));


		saveFrame->frameWrite(imRGB, imDepth, transform, frame->frameId_);
	});

	slam.AddFrameAvailableHandler(saveFrameHandler, "saveframe");

	open3d::integration::ScalableTSDFVolume * tsdf_volume = new open3d::integration::ScalableTSDFVolume(0.05, 0.05, open3d::integration::TSDFVolumeColorType::RGB8);

	auto intr = open3d::camera::PinholeCameraIntrinsic(640, 480, 612.081, 612.307, 318.254, 237.246);

	int frame_counter = 1;

	cv::namedWindow("image");
	//cv::namedWindow("image2");
	//cv::namedWindow("image3");

	//mkdir("frames2");

	FrameAvailableHandler tsdfFrameHandler([&tsdf_volume, &frame_counter, intr](MultiCameraFrame::Ptr frame) {

		if (frame_counter % 3 == 0) {

			cout << "Integrating frame number: " << frame_counter << endl;

			int height = 480;
			int width = 640;

			cv::Mat color_mat;
			cv::Mat depth_mat;

			frame->getImage(color_mat, 3);
			frame->getImage(depth_mat, 4);

			//cv::imshow("image3", depth_mat);

			auto color_im = std::make_shared<open3d::geometry::Image>();
			color_im->Prepare(width, height, 3, sizeof(uint8_t));

			uint8_t *pi = (uint8_t *)(color_im->data_.data());

			for (int i = 0; i < height; i++) {
				for (int k = 0; k < width; k++) {

					//cout << i << " " << k << endl;

					
					cv::Vec3b pixel = color_mat.at<cv::Vec3b>(i, k);

					*pi++ = pixel[0];
					*pi++ = pixel[1];
					*pi++ = pixel[2];
				}
			}


			auto depth_im = std::make_shared<open3d::geometry::Image>();
			depth_im->Prepare(width, height, 1, sizeof(uint16_t));

			//cout << depth_mat.size << endl;
			
			uint16_t * p = (uint16_t *)depth_im->data_.data();

			for (int i = 0; i < height; i++) {
				for (int k = 0; k < width; k++) {
					*p++ = depth_mat.at<uint16_t>(i, k);
				}
			}


			auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*color_im, *depth_im, 1000.0, 2.3, false);

			/*
			cv::Mat show_mat = cv::Mat(cv::Size(width, height), CV_8UC3);

			for (int i = 0; i < height; i++) {
				for (int k = 0; k < width; k++) {

					//cout << i << " " << k << endl;

					uint8_t *p = (uint8_t *)(color_im->data_.data() + (i * height + k) * 3 * sizeof(uint8_t));

					cv::Vec3b pixel = show_mat.at<cv::Vec3b>(i, k);

					pixel[0] = *p++;
					pixel[1] = *p++;
					pixel[2] = *p++;
				}
			}

			cout << "xd2" << endl;

			//cout << show_mat.size << endl;

			cv::Mat showMatconv;
			cv::cvtColor(show_mat, showMatconv, CV_RGB2BGR);
			cv::imshow("image2", showMatconv); */

			tsdf_volume->Integrate(*rgbd_image, intr, frame->T_WC(3).inverse());

			//std::shared_ptr<const open3d::geometry::Geometry> mesh = tsdf_volume->ExtractTriangleMesh();

			//cout << mesh->IsEmpty() << endl;

		}
	});

	slam.AddFrameAvailableHandler(tsdfFrameHandler, "tsdfframe");

	MyGUI::ObjectWindow mesh_win("Mesh Viewer", 600, 600);
	MyGUI::Mesh mesh_obj("mesh");

	mesh_win.add_object(&mesh_obj);


	std::shared_ptr<open3d::geometry::TriangleMesh> vis_mesh;

	FrameAvailableHandler meshHandler([&tsdf_volume, &frame_counter, &vis_mesh, &mesh_obj](MultiCameraFrame::Ptr frame) {
		if (frame_counter % 30 == 1) {
			
			vis_mesh = tsdf_volume->ExtractTriangleMesh();

			cout << "num vertices: " << vis_mesh->vertices_.size() << endl;
			cout << "num triangles: " << vis_mesh->triangles_.size() << endl;

			mesh_obj.update_mesh(vis_mesh->vertices_, vis_mesh->vertex_colors_, vis_mesh->triangles_);

		}
		

	});

	slam.AddFrameAvailableHandler(meshHandler, "meshupdate");

	FrameAvailableHandler viewHandler([&mesh_obj, &tsdf_volume](MultiCameraFrame::Ptr frame) {
		Eigen::Affine3d transform(frame->T_WC(3));
		mesh_obj.set_transform(transform.inverse());
	});
	
	slam.AddFrameAvailableHandler(viewHandler, "viewhandler");

	// thread *app = new thread(application_thread);


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
		if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC

	}

	cout << "getting mesh" << endl;

	//std::shared_ptr<const open3d::geometry::Geometry> mesh = tsdf_volume->ExtractTriangleMesh();

	std::shared_ptr<open3d::geometry::TriangleMesh> write_mesh = tsdf_volume->ExtractTriangleMesh();

	//const std::vector<std::shared_ptr<const open3d::geometry::Geometry>> mesh_vec = { mesh };

	//open3d::visualization::DrawGeometries(mesh_vec);

	open3d::io::WriteTriangleMeshToPLY("mesh.ply", *write_mesh, false, false, true, true, false, false);

	printf("\nTerminate...\n");
	// Clean up
	slam.ShutDown();
	printf("\nExiting...\n");
	return 0;
}
