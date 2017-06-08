#include "DepthCamera.h"

void DepthCamera::computeClusters(double max_distance, double min_size)
{
	clusters.clear();
	cv::Mat depthMap = cv::Mat::zeros(depthMap.rows, depthMap.cols, depthMap.type());
	cv::medianBlur(xyzMap, depthMap, 3);
	cv::Mat mask = cv::Mat::zeros(depthMap.rows, depthMap.cols, depthMap.type());
	for (int r = depthMap.rows - 1; r >= 0; r--) {
		for (int c = 0; c < depthMap.cols; c++) {
			if (depthMap.at<cv::Vec3f>(r, c)[2] > 0.2) {
				mask = cv::Mat::zeros(depthMap.rows, depthMap.cols, depthMap.type());
				floodFill(c, r, depthMap, mask, max_distance);
				cv::Mat channels[3];
				cv::split(mask, channels);
				if (cv::countNonZero(channels[2]) > min_size) {
					cv::medianBlur(mask, mask, 3);
					clusters.push_back(mask.clone());
				}
			}
		}
	}
}

/***
Recursively performs floodfill on depthMap
***/
void DepthCamera::floodFill(int x, int y, cv::Mat& depthMap, cv::Mat& mask, double max_distance)
{
	if (x < 0 || x >= depthMap.cols || y < 0 || y >= depthMap.rows || depthMap.at<cv::Vec3f>(y, x)[2] == 0.0)
		return;
	if (closeEnough(x, y, depthMap, 4, max_distance)) {
		mask.at<cv::Vec3f>(y, x) = depthMap.at<cv::Vec3f>(y, x);
		depthMap.at<cv::Vec3f>(y, x)[0] = 0;
		depthMap.at<cv::Vec3f>(y, x)[1] = 0;
		depthMap.at<cv::Vec3f>(y, x)[2] = 0;
	}
	else {
		return;
	}

	floodFill(x + 1, y, depthMap, mask, max_distance);
	floodFill(x - 1, y, depthMap, mask, max_distance);
	floodFill(x, y + 1, depthMap, mask, max_distance);
	floodFill(x, y - 1, depthMap, mask, max_distance);
}

/***
Check whether candidate point is close enough to neighboring points
***/
bool DepthCamera::closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance)
{
	int num_close = 0;
	if (x - 1 < 0 || depthMap.at<cv::Vec3f>(y, x - 1)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x - 1)) < max_distance) {
		num_close++;
	}
	if (x + 1 >= depthMap.cols || depthMap.at<cv::Vec3f>(y, x + 1)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x + 1)) < max_distance) {
		num_close++;
	}
	if (y - 1 < 0 || depthMap.at<cv::Vec3f>(y - 1, x)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y - 1, x)) < max_distance) {
		num_close++;
	}
	if (y + 1 >= depthMap.rows || depthMap.at<cv::Vec3f>(y + 1, x)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y + 1, x)) < max_distance) {
		num_close++;
	}

	if (num_close >= num_neighbors) {
		return true;
	}

	return false;
}

/***
Remove noise on zMap and xyzMap based on INVALID_FLAG_VALUE and CONFIDENCE_THRESHOLD
***/
void DepthCamera::removeNoise() {

	for (int y = 0; y < xyzMap.rows; y++) {
		for (int x = 0; x < xyzMap.cols; x++) {
			if (ampMap.data != NULL) {
				if (xyzMap.at<cv::Vec3f>(y, x)[2] > 0.9f || ampMap.at<float>(y, x) < CONFIDENCE_THRESHHOLD) {
					xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
					xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
					xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
				}
			}
			else {
				if (xyzMap.at<cv::Vec3f>(y, x)[2] > 0.9f){
					xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
					xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
					xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
				}
			}
		}
	}


	cv::Mat channels[3];
	cv::split(xyzMap, channels);
	if ((float)cv::countNonZero(channels[2]) / (xyzMap.rows*xyzMap.cols) > 0.5) {
		badInput = true;
	}
	else {
		badInput = false;
	}

	return;
}

void DepthCamera::removePoints(std::vector<cv::Point2i> points)
{
	for (int i = 0; i < points.size(); i++) {
		int x = points[i].x;
		int y = points[i].y;
		xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
		xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
		xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
	}
	return;
}

int DepthCamera::getWidth()
{
	return X_DIMENSION;
}

int DepthCamera::getHeight()
{
	return Y_DIMENSION;
}

cv::Mat DepthCamera::getXYZMap()
{
	return xyzMap;
}

cv::Mat DepthCamera::getAmpMap()
{
	return ampMap;
}

cv::Mat DepthCamera::getFlagMap()
{
	return flagMap;
}

std::vector<cv::Mat> DepthCamera::getClusters()
{
	return clusters;
}

void DepthCamera::initilizeImages()
{
	cv::Size dimension = cv::Size(X_DIMENSION, Y_DIMENSION);
	ampMap = cv::Mat(dimension, CV_32FC1);
	xyzMap = cv::Mat(dimension, CV_32FC3);
	flagMap = cv::Mat(dimension, CV_8UC1);
	clusters.clear();

	return;
}

/***
write a frame into file located at "destination"
***/
bool DepthCamera::writeImage(std::string destination) {
	cv::FileStorage fs(destination, cv::FileStorage::WRITE);

	fs << "xyzMap" << xyzMap;
	fs << "ampMap" << ampMap;
	fs << "flagMap" << flagMap;

	fs.release();
	return true;
}

/***
Reads a frame from file located at "source"
***/
bool DepthCamera::readImage(std::string source) {
	cv::FileStorage fs;
	fs.open(source, cv::FileStorage::READ);

	initilizeImages();
	fs["xyzMap"] >> xyzMap;
	fs["ampMap"] >> ampMap;
	fs["flagMap"] >> flagMap;

	fs.release();

	if (xyzMap.rows == 0 || ampMap.rows == 0 || flagMap.rows == 0) {
		return false;
	}
	else {
		return true;
	}
}