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
MockD435iCamera::MockD435iCamera(path dir, std::string& configFilename) : dataDir(dir), imuTxtPath(dir / "imu.txt"), metaTxtPath(dir / "meta.txt"), intrinFilePath(dir / "intrin.bin"), timestampTxtPath(dir / "timestamp.txt"), depthDir(dir / "depth/"),
                                             rgbDir(dir / "rgb/"), infraredDir(dir / "infrared/"), infrared2Dir(dir / "infrared2/"), firstFrameId(-1), startTime(0), configFilename(configFilename)
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

		// loading color intrinsics
		cv::FileStorage file;
		struct stat buffer;
		if (stat(configFilename.c_str(), &buffer) == 0) {
			file = cv::FileStorage(configFilename, cv::FileStorage::READ);
		}
		else {
			file = cv::FileStorage();
		}

		if (file["additional_cameras"][1]["focal_length"][0].isReal()) {
			file["additional_cameras"][1]["focal_length"][0] >> colorIntrinsics.fx;
			file["additional_cameras"][1]["focal_length"][1] >> colorIntrinsics.fy;
		}
		else {
			std::cout << "option [additional_cameras][1][focal_length][0] not found, setting to default 6.10873962e+02" << std::endl;
			colorIntrinsics.fx = 6.10873962e+02;
			colorIntrinsics.fy = 6.11282288e+02;
		}

		if (file["additional_cameras"][1]["principal_point"][0].isReal()) {
			file["additional_cameras"][1]["principal_point"][0] >> colorIntrinsics.ppx;
			file["additional_cameras"][1]["principal_point"][1] >> colorIntrinsics.ppy;
		}
		else {
			std::cout << "option [additional_cameras][1][principal_point][0] not found, setting to default 3.18763977e+02" << std::endl;
			colorIntrinsics.ppx = 3.18763977e+02;
			colorIntrinsics.ppy = 2.45752747e+02;
		}
		// color intrin read done

		// loading depth intrinsics
		if (file["additional_cameras"][0]["focal_length"][0].isReal()) {
			file["additional_cameras"][0]["focal_length"][0] >> depthIntrinsics.fx;
			file["additional_cameras"][0]["focal_length"][1] >> depthIntrinsics.fy;
		}
		else {
			std::cout << "option [additional_cameras][0][focal_length][0] not found, setting to default 6.10873962e+02" << std::endl;
			depthIntrinsics.fx = 3.83738525e+02;
			depthIntrinsics.fy = 3.83738525e+02;
		}

		if (file["additional_cameras"][0]["principal_point"][0].isReal()) {
			file["additional_cameras"][0]["principal_point"][0] >> depthIntrinsics.ppx;
			file["additional_cameras"][0]["principal_point"][1] >> depthIntrinsics.ppy;
		}
		else {
			std::cout << "option [additional_cameras][0][principal_point][0] not found, setting to default 3.18763977e+02" << std::endl;
			depthIntrinsics.ppx = 3.19075897e+02;
			depthIntrinsics.ppy = 2.38199295e+02;
		}

		if (file["additional_cameras"][0]["image_dimension"][0].isReal()) {
			file["additional_cameras"][0]["image_dimension"][0] >> depthIntrinsics.width;
			file["additional_cameras"][0]["image_dimension"][1] >> depthIntrinsics.height;
		}
		else {
			std::cout << "option [additional_cameras][0][image_dimension][0] not found, setting to default 640x480" << std::endl;
			depthIntrinsics.width = width;
			depthIntrinsics.height = height;
		}

		if (file["additional_cameras"][0]["distortion_coefficients"][0].isReal()) {
			// TODO: find a way to read coeffs. trivial method as above does not work
			depthIntrinsics.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY;
			memset(depthIntrinsics.coeffs, 0, sizeof(depthIntrinsics.coeffs));
		}
		else {
			std::cout << "option [additional_cameras][0][distortion_coefficients][0] not found, breakinggg" << std::endl;
			memset(depthIntrinsics.coeffs, 0, sizeof(depthIntrinsics.coeffs));
			depthIntrinsics.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY;
		}

		std::cout << "depthIntrin: fx: " << depthIntrinsics.fx << " fy: " << depthIntrinsics.fy << " ppx: " << depthIntrinsics.ppx << " ppy: " << depthIntrinsics.ppy << '\n';

		auto metaStream = ifstream(metaTxtPath.string());
        std::string ph;
        metaStream >> ph >> scale;
        std::cout << "scale: " << scale << "\n";
        metaStream.close();
    }
}

bool MockD435iCamera::getImuToTime(double timestamp, std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>> &data_out)
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

std::vector<float> MockD435iCamera::getColorIntrinsics() {
	return std::vector<float>{colorIntrinsics.fx, colorIntrinsics.fy, colorIntrinsics.ppx, colorIntrinsics.ppy};
}

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

void MockD435iCamera::update(MultiCameraFrame::Ptr frame)
{
    std::string line;
    if (!std::getline(timestampStream, line))
    {
        std::cout << "Unable to read form data or data end reached\n";
        frame->frameId_ = -1;
        return;
    }
    auto ss = std::stringstream(line);
    int frameId;
    double timestamp;
    ss >> frameId >> timestamp;

    frame->frameId_ = frameId;
    frame->timestamp_ = timestamp;
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

    std::vector<path> pathList{infraredDir, infrared2Dir, depthDir, rgbDir, depthDir};
    frame->images_.resize(pathList.size());

    // for (auto i = 0; i < pathList.size(); i++)
    // {
    //     frame.images_[i] = loadImg(pathList[i] / fileName);
    // }
    frame->images_[0] = cv::imread((pathList[0] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    frame->images_[1] = cv::imread((pathList[1] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
    frame->images_[3] = cv::imread((pathList[3] / fileName).string(), cv::IMREAD_COLOR);
    frame->images_[4] = cv::imread((pathList[4] / fileName).string(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);

    // project the point cloud at 2
    // TODO: should we mock the block time as well?
    //boost::this_thread::sleep_for(boost::chrono::milliseconds(33));
    // TODO: not sure if this necessary
    // printf("frame %d\n", frameId);
    // std::cout << "RGB Size: " << frame.images_[3].total() << " type: " << frame.images_[3].type() << "\n";
    // std::cout << "DEPTH Size: " << frame.images_[4].total() << " type: " << frame.images_[4].type() << "\n";

    frame->images_[2] = cv::Mat(cv::Size(width,height), CV_32FC3);
    project(frame->images_[4], frame->images_[2]);
    frame->images_[2] = frame->images_[2]*scale;

    // std::cout << "Depth cloud: " << frame.images_[2].total() << "\n";
}

} // namespace ark
