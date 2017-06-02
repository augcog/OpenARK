#define NOMINMAX
#define _WINSOCKAPI_  

#include "SR300Camera.h"

#include "util_render.h"
#include "Visualizer.h"
#include "Converter.h"

#include <iostream>



/***
Private constructor for the SR300 Camera depth sensor
***/
using namespace std;
//using namespace Intel::RealSense;



const int depth_fps = 30;
int depth_width;
int depth_height;
cv::Size bufferSize;
const PXCCapture::Sample *sample;
PXCSenseManager *sm = PXCSenseManager::CreateInstance();
PXCSession *session = sm->QuerySession();

PXCCapture::Device *device;
PXCCaptureManager *cm;

UtilRender renderc(L"Color"), renderd(L"Depth"), renderi(L"IR"), renderr(L"Right"), renderl(L"Left");
int num_pixels;
vector<cv::Point3f>  xyzBuffer;

SR300Camera::SR300Camera(bool use_live_sensor) {

	session->SetCoordinateSystem(PXCSession::CoordinateSystem::COORDINATE_SYSTEM_FRONT_DEFAULT);
	X_DIMENSION = 640;
	Y_DIMENSION = 480;
	
	if (!sm) {
		wprintf_s(L"Unable to create the SenseManager\n");
	}
	cm = sm->QueryCaptureManager();
	Status sts = STATUS_DATA_UNAVAILABLE; 
										
	sm->EnableStream(PXCCapture::StreamType::STREAM_TYPE_DEPTH,
		             X_DIMENSION, Y_DIMENSION, depth_fps);
	sts = sm->Init();
	if (sts < Status::STATUS_NO_ERROR) {
		sm->Close();
		sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH);
		sts = sm->Init();
		if (sts < Status::STATUS_NO_ERROR) {
			sm->Close();
			sts = sm->Init();
		}
	}
	device = cm->QueryDevice();
	//device->ResetProperties(PXCCapture::STREAM_TYPE_ANY);

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

void SR300Camera::fillInZCoords(){
	int res;
	Status sts = sm ->AcquireFrame(true);
	if (sts < STATUS_NO_ERROR) {
		if (sts == Status::STATUS_STREAM_CONFIG_CHANGED) {
			wprintf_s(L"Stream configuration was changed, re-initilizing\n");
			sm ->Close();
		}
	}
	sample = sm->QuerySample();
	PXCImage *depthMap = sample->depth;
	//renderd.RenderFrame(sample->depth);
	PXCImage::ImageData depthImage;
	depthMap->AcquireAccess(PXCImage::ACCESS_READ, &depthImage);
	cv::Mat img;
	Converter::ConvertPXCImageToOpenCVMat(depthMap, depthImage, &img);
	cv::imshow("Depth Image by OpenARK", Visualizer::visualizeDepthMap(img));
	PXCImage::ImageInfo imgInfo = depthMap->QueryInfo();
	depth_width = imgInfo.width;
	depth_height = imgInfo.height;
	num_pixels = depth_width * depth_height;
	PXCProjection * projection = device->CreateProjection();
	PXCPoint3DF32 *pos3D = new PXCPoint3DF32[num_pixels];
	//sts = projection->QueryVertices(depthMap, pos3D);
	sts = projection->QueryVertices(depthMap, &pos3D[0]);
	if (sts < Status::STATUS_NO_ERROR) {
		wprintf_s(L"Projection was unsuccessful! \n");
		sm->Close();
	}
	xyzBuffer.clear();
	for (int k = 0; k < num_pixels; k++) {
			//xyzBuffer.push_back(cv::Point3f(pos3D[k].x/1000.0f, pos3D[k].y/1000.0f, pos3D[k].z/1000.0f));
			xyzBuffer.emplace_back(cv::Point3f(pos3D[k].x / 1000.0f, pos3D[k].y / 1000.0f, pos3D[k].z / 1000.0f));
	}

	xyzMap = cv::Mat(xyzBuffer, true).reshape(3, 480);
	//cvtColor(xyzMap, xyzMap, CV_RGB2BGR);
	cv::namedWindow("XYZ Image by OpenARK", CV_WINDOW_AUTOSIZE);
	cv::imshow("XYZ Image by OpenARK", Visualizer::visualizeXYZMap(xyzMap));
/*	dists = new float[3 * numPixels]; // Dists contains XYZ values. needs to be 3x the size of numPixels
	amps = new float[numPixels];
    //frame = cvCreateImage(cvSize(depth_width, depth_height), 8, 3); // Create the frame
	frame = cv::Mat(cv::Size(depth_width, depth_height), CV_8UC3);

	KF.init(6, 3, 0);
	KF.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 1,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);
	measurement = cv::Mat_<float>::zeros(3, 1);

	//Initaite Kalman
	KF.statePre.setTo(0);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, cv::Scalar::all(.001)); // Adjust this for faster convergence - but higher noise
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, cv::Scalar::all(.1)); */	
}




/***
Reads the amplitude data from the sensor and fills in the matrix
***/
void SR300Camera::fillInAmps()
{

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