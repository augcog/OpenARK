#include <iostream>
#include <thread>
#include <atomic>
#include <ctime>
#include <csignal>
#include <exception>
#include "glfwManager.h"
#include "concurrency.h"
#include "MockD435iCamera.h"
#include "Util.h"
#include "OkvisSLAMSystem.h"

using namespace ark;
using boost::filesystem::path;

void signal_handler(int signum)
{
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGILL, signal_handler);
    std::signal(SIGABRT, signal_handler);
    std::signal(SIGSEGV, signal_handler);
    std::signal(SIGTERM, signal_handler);
    if (argc > 5)
    {
        std::cerr << "Usage: ./" << argv[0] << " [configuration-yaml-file] [vocabulary-file] [skip-first-seconds] [data_path]" << std::endl
                  << "Args given: " << argc << std::endl;
        return -1;
    }

    google::InitGoogleLogging(argv[0]);

    // read configuration file
    std::string configFilename;
    if (argc > 1)
        configFilename = argv[1];
    else
        configFilename = util::resolveRootPath("config/d435i_intr.yaml");

    std::string vocabFilename;
    if (argc > 2)
        vocabFilename = argv[2];
    else
        vocabFilename = util::resolveRootPath("config/brisk_vocab.bn");

    okvis::Duration deltaT(0.0);
    if (argc > 3)
    {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    path dataPath{"./data_path_25-10-2019 16-47-28"};
    if (argc > 4)
    {
        dataPath = path(argv[4]);
    }

    OkvisSLAMSystem slam(vocabFilename, configFilename);

    //setup display
    if (!MyGUI::Manager::init())
    {
        fprintf(stdout, "Failed to initialize GLFW\n");
        return -1;
    }

    printf("Camera initialization started...\n");
    fflush(stdout);
    MockD435iCamera camera(dataPath);

    printf("Camera-IMU initialization complete\n");
    fflush(stdout);

    //Window for displaying the path
    MyGUI::CameraWindow traj_win("Traj Viewer", 640 * 2, 480 * 2);
    MyGUI::ARCameraWindow ar_win("AR Viewer", 640 * 2.5, 480 * 2.5, GL_LUMINANCE, GL_UNSIGNED_BYTE, 6.16403320e+02, 6.16171021e+02, 3.18104584e+02, 2.33643127e+02, 0.01, 100);
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
    std::vector<MyGUI::Object *> cubes;
    std::vector<Eigen::Matrix4d> T_K_cubes;
    std::vector<MapKeyFrame::Ptr> K_cubes;

    //Recieves output from SLAM system and displays to the screen
    FrameAvailableHandler handler([&path1, &axis2, &ar_win, &cubes, &T_K_cubes, &K_cubes](MultiCameraFrame::Ptr frame) {
        Eigen::Affine3d transform(frame->T_WC(3));
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        ar_win.set_camera(transform);
        ar_win.set_image(frame->images_[3]);
        if (ar_win.clicked())
        {

            std::string cube_name = std::string("CubeNum") + std::to_string(cubes.size());
            MyGUI::Object *obj = new MyGUI::Cube(cube_name, 0.1, 0.1, 0.1);
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
        for (size_t i = 0; i < traj.size(); i++)
        {
            path1.add_node(traj[i].block<3, 1>(0, 3));
        }
        for (size_t i = 0; i < cubes.size(); i++)
        {
            if (K_cubes[i] != nullptr)
                cubes[i]->set_transform(Eigen::Affine3d(K_cubes[i]->T_WS() * T_K_cubes[i]));
        }
    });
    slam.AddLoopClosureDetectedHandler(loopHandler, "trajectoryUpdate");
    //run until display is closed
    okvis::Time start(0.0);
    camera.start();
    int lastMapIndex = -1;

    while (MyGUI::Manager::running())
    {
        //Update the display
        MyGUI::Manager::update();

        try
        {
            //Get current camera frame
            MultiCameraFrame::Ptr frame(new MultiCameraFrame);
            camera.update(*frame);

            const auto frameId = frame->frameId_;
            if (frameId < 0) {
                std::cout << "Data end reached\n";
                break;
            }
            const auto &infrared = frame->images_[0];
            const auto &infrared2 = frame->images_[1];
            const auto &rgb = frame->images_[3];
            const auto &depth = frame->images_[4];


            //std::cout << "RGB: " << rgb.total() << "\n";
            //cv::imshow(std::string(camera.getModelName()) + " RGB", rgb);
            // tmp fix for the preview window in MyGUI
            //cv::cvtColor(rgb, rgb, CV_RGB2BGR); 
            cv::imshow(std::string(camera.getModelName()) + " Infrared", infrared);
            cv::imshow(std::string(camera.getModelName()) + " Depth", depth);
            //Get or wait for IMU Data until current frame
            //std::cout << "frame size: " << rgb.total() << std::endl;
            std::vector<ImuPair> imuData;
            camera.getImuToTime(frame->timestamp_, imuData);
            // std::cout << "numimu: " << imuData.size() << std::endl;
            // std::cout << "timestamp: " << std::setprecision(15) << frame->timestamp_ << std::endl;

            //Add data to SLAM system
            // printf("before slam imu\n");
            slam.PushIMU(imuData);
            // make it the same as real camera
            frame->images_.resize(4);
            slam.PushFrame(frame);
            // printf("after slam frame\n");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n'; 
        }
        catch (...)
        {
            std::cout << "ex catched\n";
        }
        const auto isReset = slam.okvis_estimator_->isReset();
        const auto mapIndex = slam.getActiveMapIndex();
        if (mapIndex != lastMapIndex) {
            lastMapIndex = mapIndex;
            std::cout << "Mapnumber : " << mapIndex << "\n";
        }
        // std::cout << "slam reset? : " << isReset << "\n";
        if (isReset) {
            //traj_win.msg_ = " *Reseting*";
            path1.clear();
        } else {
            //traj_win.msg_ = " ";
        }
        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27)
            break; // 27 is ESC
    }
    printf("\nTerminate...\n");
    // Clean up
    slam.ShutDown();
    printf("\nExiting...\n");
    return 0;
}