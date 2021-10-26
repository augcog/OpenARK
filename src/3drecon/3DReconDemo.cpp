#include "D435iCamera.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
//#include <direct.h>
#include <thread>
#include "util/glfwManager.h"
#include "util/Util.h"
#include "SaveFrame.h"
#include "Types.h"
#include "SegmentedMesh.h"

using namespace ark;

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

	//readConfig(configFilename);

	cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

	bool save_frames = true;
	int mesh_view_width = 1000, mesh_view_height = 1000;

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

	SegmentedMesh * mesh = new SegmentedMesh(configFilename, slam, &camera);

	MyGUI::MeshWindow mesh_win("Mesh Viewer", mesh_view_width, mesh_view_height);
	MyGUI::Mesh mesh_obj("mesh", mesh);

	mesh_win.add_object(&mesh_obj);

	FrameAvailableHandler viewHandler([&mesh, &mesh_obj, &mesh_win, &do_integration](MultiCameraFrame::Ptr frame) {
		Eigen::Affine3d transform(frame->T_WC(3));
		mesh_obj.set_transform(transform.inverse());

		if (mesh_win.clicked()) {
			do_integration = !do_integration;
			mesh->SetIntegrationEnabled(do_integration);

			if (do_integration) {
				std::cout << "----INTEGRATION ENABLED----" << endl;
			}
			else {
				std::cout << "----INTEGRATION DISABLED----" << endl;
			}
		}
	});
	
	slam.AddFrameAvailableHandler(viewHandler, "viewhandler");

	if (save_frames) {
		KeyFrameAvailableHandler saveFrameHandler([&saveFrame, &frame_counter, &do_integration](MultiCameraFrame::Ptr frame) {
			if (!do_integration || frame_counter % 3 != 0) {
				return;
			}

			cv::Mat imRGB;
			cv::Mat imDepth;

			frame->getImage(imRGB, 3);
			frame->getImage(imDepth, 4);

			Eigen::Matrix4d transform(frame->T_WC(3));

			saveFrame->frameWrite(imRGB, imDepth, transform, frame->frameId_);
		});
	
		slam.AddKeyFrameAvailableHandler(saveFrameHandler, "saveframe");
	}

	cv::namedWindow("image");

	while (MyGUI::Manager::running()) {

		//Update the display
		MyGUI::Manager::update();

		//Get current camera frame
		MultiCameraFrame::Ptr frame(new MultiCameraFrame);
		camera.update(frame);

		//Get or wait for IMU Data until current frame 
		std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>> imuData;
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
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> traj;

	slam.getMappedTrajectory(frameIdOut, traj);

	std::map<int, Eigen::Matrix4d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix4d>>> keyframemap;

	for (int i = 0; i < frameIdOut.size(); i++) {
		keyframemap[frameIdOut[i]] = traj[i];
	}
	saveFrame->updateTransforms(keyframemap);

	std::vector<int> active_frames;
	slam.getActiveFrames(active_frames);
	saveFrame->writeActiveFrames(active_frames);

	mesh->WriteMeshes();

	printf("\nTerminate...\n");
	// Clean up
	slam.ShutDown();
	printf("\nExiting...\n");
	return 0;
}
