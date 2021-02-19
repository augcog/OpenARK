#include "stdafx.h"
#include "Version.h"
#include "MockD435iCamera.h"
#include "Visualizer.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <boost/archive/text_iarchive.hpp>

/** RealSense SDK2 Cross-Platform Depth Camera Backend **/
namespace ark
{
MockD435iCamera::MockD435iCamera(path dir) : dataDir(dir), imuTxtPath(dir / "imu.txt"), metaTxtPath(dir / "meta.txt"), intrinFilePath(dir / "intrin.bin"), timestampTxtPath(dir / "timestamp.txt"), depthDir(dir / "depth/"),
                                             rgbDir(dir / "rgb/"), infraredDir(dir / "infrared/"), infrared2Dir(dir / "infrared2/"), firstFrameId(-1), startTime(0)
{
    width = 640;
    height = 480;
}

MockD435iCamera::~MockD435iCamera()
{
    imuStream.close();
    timestampStream.close();
}

void MockD435iCamera::start()
{
    imuStream = ifstream(imuTxtPath.string());
    timestampStream = ifstream(timestampTxtPath.string());
    {
        auto intrinStream = ifstream(intrinFilePath.string()); // Moon : This change might be wrong.
        boost::archive::text_iarchive ia(intrinStream);
        ia >> depthIntrinsics;

        std::cout << "depthIntrin: fx: " << depthIntrinsics.fx << " fy: " << depthIntrinsics.fy << " ppx: " << depthIntrinsics.ppx << " ppy: " << depthIntrinsics.ppy << '\n';

        auto metaStream = ifstream(metaTxtPath.string()); // Moon : This change might be wrong.
        std::string ph;
        metaStream >> ph >> scale;
        std::cout << "scale: " << scale << "\n";
        metaStream.close();
    }
}

bool MockD435iCamera::getImuToTime(double timestamp, std::vector<ImuPair> &data_out)
{
    std::string line1;
    std::string line2;
    std::string line3;

    double ts = 0;
    double gyro0, gyro1, gyro2, accel0, accel1, accel2;
    for (; !imuStream.eof() && ts < timestamp;)
    {
        // TODO: refactor
        bool allGood = true;
        allGood = allGood && std::getline(imuStream, line1);
        allGood = allGood && std::getline(imuStream, line2);
        allGood = allGood && std::getline(imuStream, line3);

        if (!allGood) {
            std::cout << "getImuToTime: unable to read imu data.\n";
            return false;
        }

        string placeholder;
        std::stringstream ss1(line1);
        std::stringstream ss2(line2);
        std::stringstream ss3(line3);
        ss1 >> placeholder >> ts;
        ss2 >> placeholder >> gyro0 >> gyro1 >> gyro2;
        ss3 >> placeholder >> accel0 >> accel1 >> accel2;
        ImuPair imu_out{ts, //convert to nanoseconds, for some reason gyro timestamp is in centiseconds
                        Eigen::Vector3d(gyro0, gyro1, gyro2),
                        Eigen::Vector3d(accel0, accel1, accel2)};
        data_out.emplace_back(imu_out);
    }
    return true;
};

const std::string MockD435iCamera::getModelName() const
{
    return "Mock";
}

cv::Size MockD435iCamera::getImageSize() const
{
    return cv::Size(width, height);
}
cv::Mat MockD435iCamera::loadImg(path filename)
{
    return imread(filename.string(), cv::IMREAD_COLOR);
}

void MockD435iCamera::project(const cv::Mat &depth_frame, cv::Mat &xyz_map)
{
    const uint16_t *depth_data = (const uint16_t *)depth_frame.data;

    rs2_intrinsics *dIntrin = &depthIntrinsics;

    const uint16_t *srcPtr;
    cv::Vec3f *destPtr;
    float srcPixel[2], destXYZ[3];

    for (int r = 0; r < height; ++r)
    {
        srcPtr = depth_data + r * dIntrin->width;
        destPtr = xyz_map.ptr<Vec3f>(r);
        srcPixel[1] = r;

        for (int c = 0; c < width; ++c)
        {
            if (srcPtr[c] == 0)
            {
                memset(&destPtr[c], 0, 3 * sizeof(float));
                continue;
            }
            srcPixel[0] = c;
            rs2_deproject_pixel_to_point(destXYZ, dIntrin, srcPixel, srcPtr[c]);
            memcpy(&destPtr[c], destXYZ, 3 * sizeof(float));
        }
    }
}

void MockD435iCamera::update(MultiCameraFrame &frame)
{
    std::string line;
    if (!std::getline(timestampStream, line))
    {
        std::cout << "Unable to read form data or data end reached\n";
        frame.frameId_ = -1;
        return;
    }
    auto ss = std::stringstream(line);
    int frameId;
    double timestamp;
    ss >> frameId >> timestamp;

    frame.frameId_ = frameId;
    frame.timestamp_ = timestamp;
    if (startTime == 0)
    {
        startTime = timestamp;
    }
    // reading img
    // TODO: not sure if this necessary
    std::stringstream fileNamess;
    // TODO: extract the naming function
    fileNamess << std::setw(5) << std::setfill('0') << std::to_string(frameId) << ".png";
    std::string fileName = fileNamess.str();

    std::vector<path> pathList{infraredDir, infrared2Dir, depthDir, infraredDir, depthDir};
    frame.images_.resize(pathList.size());

    // for (auto i = 0; i < pathList.size(); i++)
    // {
    //     frame.images_[i] = loadImg(pathList[i] / fileName);
    // }
    frame.images_[0] = cv::imread((pathList[0] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    frame.images_[1] = cv::imread((pathList[1] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    frame.images_[3] = cv::imread((pathList[3] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    frame.images_[4] = cv::imread((pathList[4] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);

    // project the point cloud at 2
    // TODO: should we mock the block time as well?
    //boost::this_thread::sleep_for(boost::chrono::milliseconds(33));
    // TODO: not sure if this necessary
    // printf("frame %d\n", frameId);
    // std::cout << "RGB Size: " << frame.images_[3].total() << " type: " << frame.images_[3].type() << "\n";
    // std::cout << "DEPTH Size: " << frame.images_[4].total() << " type: " << frame.images_[4].type() << "\n";

    frame.images_[2] = cv::Mat(cv::Size(width,height), CV_32FC3);
    project(frame.images_[4], frame.images_[2]);
    frame.images_[2] = frame.images_[2]*scale;

    // std::cout << "Depth cloud: " << frame.images_[2].total() << "\n";
}

} // namespace ark
