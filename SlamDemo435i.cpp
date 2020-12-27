#include "D435iCamera.h"
#include "OkvisSLAMSystem.h"
#include <iostream>
#include <thread>
#include "glfwManager.h"
#include "Util.h"

using namespace ark;

int main(int argc, char **argv)
{

    if (argc >4 ) {
        std::cerr << "Usage: ./" << argv[0] << " configuration-yaml-file [vocabulary-file] [skip-first-seconds]" << std::endl
        <<"Args given: " << argc << std::endl;
        return -1;
    }

    const bool hideInactiveMaps = true;

    google::InitGoogleLogging(argv[0]);

    okvis::Duration deltaT(0.0);
    if (argc == 4) {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    // read configuration file
    std::string configFilename;
    if (argc > 1) configFilename = argv[1];
    else configFilename = util::resolveRootPath("config/d435i_intr.yaml");

    std::string vocabFilename;
    if (argc > 2) vocabFilename = argv[2];
    else vocabFilename = util::resolveRootPath("config/brisk_vocab.bn");

    OkvisSLAMSystem slam(vocabFilename, configFilename);
    cv::FileStorage configFile(configFilename, cv::FileStorage::READ);

    //setup display
    if (!MyGUI::Manager::init())
    {
       fprintf(stdout, "Failed to initialize GLFW\n");
       return -1;
    }


    printf("Camera initialization started...\n");
    fflush(stdout);
    CameraParameter cameraParameter;
    if (configFile["emitterPower"].isReal()) {
        configFile["emitterPower"] >> cameraParameter.emitterPower;
    }
    D435iCamera camera(cameraParameter);
    camera.start();

    printf("Camera-IMU initialization complete\n");
    fflush(stdout);

    //Window for displaying the path
    MyGUI::CameraWindow traj_win("Traj Viewer", 640*2,480*2);
    MyGUI::ARCameraWindow ar_win("AR Viewer", 640*2.5,480*2.5, GL_RGB, GL_UNSIGNED_BYTE,  6.16403320e+02, 6.16171021e+02, 3.18104584e+02, 2.33643127e+02,0.01,100);
    traj_win.set_pos(640*2.5,100);
    ar_win.set_pos(0,100);
    std::map<int, MyGUI::Path *> pathMap;
    MyGUI::Axis axis1("axis1", .1);
    MyGUI::Axis axis2("axis2", 1);
    MyGUI::Grid grid1("grid1", 10, 1);
    traj_win.add_object(&axis1);
    traj_win.add_object(&axis2);
    traj_win.add_object(&grid1);
    ar_win.add_object(&axis1);
    std::vector<MyGUI::Object *> cubes;
    std::vector<Eigen::Matrix4d> T_K_cubes;
    std::vector<MapKeyFrame::Ptr> K_cubes;
    printf("Window initialization complete\n");
    int lastMapIndex_path = 0;
    //Recieves output from SLAM system and displays to the screen
    FrameAvailableHandler handler([&](MultiCameraFrame::Ptr frame) {
        Eigen::Affine3d transform(frame->T_WC(3));
        const auto mapIndex = slam.getActiveMapIndex();

        if (pathMap.find(mapIndex) == pathMap.end()) {
            string name = "path"+std::to_string(mapIndex);
            pathMap[mapIndex] = new MyGUI::Path{name, Eigen::Vector3d(1, 0, 0)};
            traj_win.add_object(pathMap[mapIndex]);
        }
        if (mapIndex < lastMapIndex_path) {
            // pathMap[lastMapIndex_path]->clear();
            auto it = pathMap.cbegin();
            while(it != pathMap.cend()) {
                auto curr = it++;
                if (it->first > mapIndex) {
                    it->second->clear();
                }
            }
        }
        if (lastMapIndex_path != mapIndex) {
            if (hideInactiveMaps) {
                pathMap[lastMapIndex_path]->clear();
            }
            lastMapIndex_path = mapIndex;
        }
        pathMap[mapIndex]->add_node(transform.translation());
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

    LoopClosureDetectedHandler loopHandler([&](void) {
        std::vector<Eigen::Matrix4d> traj;
        slam.getTrajectory(traj);
        const auto mapIndex = slam.getActiveMapIndex();
        pathMap[mapIndex]->clear();
        for (size_t i = 0; i < traj.size(); i++)
        {
            pathMap[mapIndex]->add_node(traj[i].block<3, 1>(0, 3));
        }
        std::cout << "Loop Trajectory: \n";
        /*for (const auto &node: pathMap[mapIndex]->nodes) {
            std::cout << node;
        }*/
        for (size_t i = 0; i < cubes.size(); i++)
        {
            if (K_cubes[i] != nullptr)
                cubes[i]->set_transform(Eigen::Affine3d(K_cubes[i]->T_WS() * T_K_cubes[i]));
        }
    });
    slam.AddLoopClosureDetectedHandler(loopHandler, "trajectoryUpdate");
    
    SparseMapMergeHandler mergeHandler([&](int deletedIndex, int currentIndex) {
        auto it = pathMap.cbegin();
        while(it != pathMap.cend()) {
            auto curr = it++;
            curr->second->clear();
        }
        lastMapIndex_path = currentIndex;
        for (size_t i = 0; i <= currentIndex; i++)
        {
            std::vector<Eigen::Matrix4d> traj;
            slam.getMap(i)->getTrajectory(traj);
            pathMap[i]->clear();
            if (!hideInactiveMaps || i == currentIndex) {
                for (size_t j = 0; j < traj.size(); j++) {
                    pathMap[i]->add_node(traj[j].block<3, 1>(0, 3));
                }
            }
        }
    });
    slam.AddSparseMapMergeHandler(mergeHandler, "trajectoryReset");
    //run until display is closed
    okvis::Time start(0.0);
    // camera.start();
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

            std::vector<ImuPair> imuData;
            camera.getImuToTime(frame->timestamp_, imuData);

            //Add data to SLAM system
            slam.PushIMU(imuData);
            slam.PushFrame(frame);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n'; 
        }
        catch (...)
        {
            std::cout << "An exception caught.\n";
        }
        const auto isReset = slam.okvis_estimator_->isReset();
        const auto mapIndex = slam.getActiveMapIndex();
        if (mapIndex != lastMapIndex) {
            lastMapIndex = mapIndex;
            std::cout << "Mapnumber : " << mapIndex << "\n";
        }

        if (isReset) {
            traj_win.msg_ = " *Reseting*";
        } else {
            traj_win.msg_ = " ";
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
