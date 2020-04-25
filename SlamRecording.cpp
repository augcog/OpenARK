#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ceres/ceres.h>
#include <nanoflann.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/archive/text_oarchive.hpp>

// OpenARK Libraries
#include "Version.h"
#include "D435iCamera.h"
#include "Util.h"

#include "Core.h"
#include "Visualizer.h"

using namespace ark;
using boost::filesystem::path;

std::string getTimeTag()
{
    std::ostringstream oss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return oss.str();
}

void saveImg(int id, const cv::Mat &img, const path imgDir)
{   
    cout << "writing image" << endl;
    std::stringstream fileName;
    fileName << std::setw(5) << std::setfill('0') << std::to_string(id) << ".png";
    const std::string dst = (imgDir / fileName.str()).string();
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(CV_IMWRITE_PNG_STRATEGY);
    compression_params.push_back(CV_IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY);
    cv::imwrite(dst, img, compression_params);
}

int main(int argc, char **argv)
{
    printf("Welcome to OpenARK v %s Slam Recording Tool\n\n", VERSION);
    printf("CONTROLS:\nQ or ESC to stop recording and begin writing dataset to disk,\nSPACE to start/pause"
           "(warning: if pausing in the middle, may mess up timestamps)\n\n");

    const path directory_path =
        argc > 1 ? argv[1] : std::string("./data_path_") + getTimeTag(); // modify this
    const std::string configFilename = argc > 2 ? argv[2] : util::resolveRootPath("config/d435i_intr.yaml");
    path depth_path = directory_path / "depth/";
    path infrared_path = directory_path / "infrared/";
    path infrared2_path = directory_path / "infrared2/";
    path rgb_path = directory_path / "rgb/";
    path timestamp_path = directory_path / "timestamp.txt";
    path intrin_path = directory_path / "intrin.bin";
    path meta_path = directory_path / "meta.txt";
    path imu_path = directory_path / "imu.txt";
    std::vector<path> pathList{directory_path, depth_path, infrared_path, infrared2_path, rgb_path};
    for (const auto &p : pathList)
    {
        if (!boost::filesystem::exists(p))
        {
            boost::filesystem::create_directories(p);
        }
    }

    std::vector<MultiCameraFrame> frameList;
    std::vector<ImuPair> imuList;

    cv::FileStorage configFile(configFilename, cv::FileStorage::READ);
    CameraParameter cameraParameter;
    if (configFile["emitterPower"].isReal()) {
        configFile["emitterPower"] >> cameraParameter.emitterPower;
    }
    D435iCamera camera(cameraParameter);
    camera.start();

    std::vector<ImuPair> imuBuffer;
    std::vector<ImuPair> imuDispose;
    std::atomic_bool paused = true;
    std::atomic_bool quit = false;
    single_consumer_queue<std::shared_ptr<MultiCameraFrame>> img_queue;
    std::thread writingThread([&]() {
        std::ofstream imu_ofs(imu_path.string());
        std::ofstream timestamp_ofs(timestamp_path.string());
        {
            std::ofstream intrin_ofs(intrin_path.string());
            boost::archive::text_oarchive oa(intrin_ofs);
            oa << camera.getDepthIntrinsics();

            std::ofstream meta_ofs(meta_path.string());
            meta_ofs << "depth " << camera.getDepthScale();
        }
        auto frame = std::make_shared<MultiCameraFrame>();
        auto lastImuTs = -1.0;
        const auto timeGapReportThreshold = 1e7;
        while (true)
        {
            if (!img_queue.try_dequeue(&frame))
            {
                if(quit)
                    break;
                // boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
                continue;
            }
            const auto frameId = frame->frameId_;
            const auto &infrared = frame->images_[0];
            const auto &infrared2 = frame->images_[1];
            const auto &depth = frame->images_[4];
            //const auto &rgb = frame->images_[3];

            saveImg(frameId, infrared, infrared_path);
            saveImg(frameId, infrared2, infrared2_path);
            saveImg(frameId, depth, depth_path);
            //saveImg(frameId, rgb, rgb_path);

            timestamp_ofs << frameId << " " << std::setprecision(15) << frame->timestamp_ << "\n";
            if(!quit)
                cout << "Writing frame: " << frameId << endl;
            else
                cout << "Writing leftover frame: " << frameId << endl;
            imuBuffer.clear();
            camera.getImuToTime(frame->timestamp_, imuBuffer);
            for (const auto &imuPair : imuBuffer)
            {
                auto ts = imuPair.timestamp;
                if ((ts - lastImuTs) > timeGapReportThreshold) {
                    cout << "Timestamp gap in imu: " << (ts - lastImuTs) << " at time: " << ts << "\n";
                }
                lastImuTs = ts;
                imu_ofs << "ts " << std::setprecision(15) << ts << "\n"
                        << "gy " << imuPair.gyro[0] << " " << imuPair.gyro[1] << " " << imuPair.gyro[2] << "\n"
                        << "ac " << imuPair.accel[0] << " " << imuPair.accel[1] << " " << imuPair.accel[2] << "\n";
            }
        }
        imu_ofs.close();
    });

    while (true)
    {
        // 0: infrared
        // 1: infrared2
        // 2: depth(point cloud)
        // 3: rgb
        // 4: depth raw
        auto frame = std::make_shared<MultiCameraFrame>();
        camera.update(*frame);

        const auto rgb = frame->images_[3].clone();
        const auto ir1 = frame->images_[0].clone();

        cv::cvtColor(rgb, rgb, CV_RGB2BGR);
        cv::imshow(camera.getModelName() + " IR", ir1);

        if (paused)
        {
            // clear the imu data in camera?
            // extract all imu data ever since pause??
            camera.getImuToTime(frame->timestamp_, imuDispose);
            imuDispose.clear();
            const std::string NO_SIGNAL_STR = "PAUSED";
            const cv::Scalar RECT_COLOR = cv::Scalar(0, 160, 255);
            const int RECT_WID = 120, RECT_HI = 40;

            for (auto &img : frame->images_)
            {
                const cv::Point STR_POS(img.cols / 2 - 50, img.rows / 2 + 7);
                cv::Rect rect(img.cols / 2 - RECT_WID / 2,
                              img.rows / 2 - RECT_HI / 2,
                              RECT_WID, RECT_HI);
                cv::rectangle(img, rect, RECT_COLOR, -1);
                cv::putText(img, NO_SIGNAL_STR, STR_POS, 0, 0.8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }
        else
        {
            img_queue.enqueue(frame);
        }

        // visualize results
        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27)
        {
            // 27 is ESC
            quit = true;
            break;
        }
        else if (k == ' ')
        {
            paused = !paused;
        }
    }
    writingThread.join();
    return 0;
}