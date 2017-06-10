#include "SR300Camera.h"
#include "Visualizer.h"
#include <iostream>



/***
Private constructor for the SR300 Camera depth sensor
***/
using namespace std;



/***
Private constructor for the Intel RealSense SR300 camera depth sensor
***/
SR300Camera::SR300Camera(bool use_live_sensor)
{

	session->SetCoordinateSystem(Intel::RealSense::CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);
	X_DIMENSION = 640;
	Y_DIMENSION = 480;
	if (!sm)
	{
		wprintf_s(L"Unable to create the SenseManager\n");
	}
	cm = sm->QueryCaptureManager();
	Intel::RealSense::Status sts = Intel::RealSense::Status::STATUS_DATA_UNAVAILABLE;

	sm->EnableStream(Intel::RealSense::Capture::StreamType::STREAM_TYPE_DEPTH, X_DIMENSION, Y_DIMENSION, depth_fps);
	sts = sm->Init();
	if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
	{
		sm->Close();
		sm->EnableStream(Intel::RealSense::Capture::STREAM_TYPE_DEPTH);
		sts = sm->Init();
		if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
		{
			sm->Close();
			sts = sm->Init();
		}
	}
	device = cm->QueryDevice();
}


/***
Public deconstructor for the SR300 Camera depth sensor
***/
SR300Camera::~SR300Camera() {};

void SR300Camera::destroyInstance()
{
	printf("closing sensor\n");
	sm->Release();
	sm->Close();
	printf("sensor closed\n");
}


/***
Create xyzMap, zMap, ampMap, and flagMap from sensor input
***/
void SR300Camera::update()
{
	initilizeImages();
	fillInAmps();
	fillInZCoords();
	sm->ReleaseFrame();
}


/***
Reads the depth data from the sensor and fills in the matrix
***/
void SR300Camera::fillInZCoords()
{

	int num_pixels;
	vector<cv::Point3f>  xyzBuffer;

	int res;
	Intel::RealSense::Status sts = sm ->AcquireFrame(true);
	if (sts < Intel::RealSense::STATUS_NO_ERROR) {
		if (sts == Intel::RealSense::Status::STATUS_STREAM_CONFIG_CHANGED)
		{
			wprintf_s(L"Stream configuration was changed, re-initilizing\n");
			sm ->Close();
		}
	}
	sample = sm->QuerySample();
	Intel::RealSense::Image *depthMap = sample->depth;
	Intel::RealSense::Image::ImageData depthImage;
	depthMap->AcquireAccess(Intel::RealSense::Image::ACCESS_READ, &depthImage);
	cv::Mat img;
	Converter::ConvertPXCImageToOpenCVMat(depthMap, depthImage, &img);
	cv::imshow("Depth Image by OpenARK", Visualizer::visualizeDepthMap(img));
	Intel::RealSense::Image::ImageInfo imgInfo = depthMap->QueryInfo();
	depth_width = imgInfo.width;
	depth_height = imgInfo.height;
	num_pixels = depth_width * depth_height;
	Intel::RealSense::Projection * projection = device->CreateProjection();
	Intel::RealSense::Point3DF32 *pos3D = new Intel::RealSense::Point3DF32[num_pixels];
	sts = projection->QueryVertices(depthMap, &pos3D[0]);
	if (sts < Intel::RealSense::Status::STATUS_NO_ERROR)
	{
		wprintf_s(L"Projection was unsuccessful! \n");
		sm->Close();
	}
	xyzBuffer.clear();
	for (int k = 0; k < num_pixels; k++)
	{
			xyzBuffer.emplace_back(cv::Point3f(pos3D[k].x / 1000.0f, pos3D[k].y / 1000.0f, pos3D[k].z / 1000.0f));
	}
	xyzMap = cv::Mat(xyzBuffer, true).reshape(3, 480);
}


/***
Reads the amplitude data from the sensor and fills in the matrix
***/
void SR300Camera::fillInAmps()
{
	ampMap.data = NULL;
}


/***
Returns the X value at (i, j)
***/
float SR300Camera::getX(int i, int j) const
{
	int flat = j * depth_width * 3 + i * 3;
	return dists[flat];
}


/***
Returns the Y value at (i, j)
***/
float SR300Camera::getY(int i, int j) const
{
	int flat = j * depth_width * 3 + i * 3;
	return dists[flat + 1];
}


/***
Returns the Z value at (i, j)
***/
float SR300Camera::getZ(int i, int j) const
{
	int flat = j * depth_width * 3 + i * 3;
	return dists[flat + 2];
}