#include "DepthCamera.h"
#include <iostream>

DepthCamera::~DepthCamera()
{
}

void DepthCamera::update()
{
}

void DepthCamera::destroyInstance()
{
}

void DepthCamera::computeClusters(double max_distance, double min_size, int floodfill_interval)
{
    clusters.clear();

    cv::Mat depthMap;

    cv::Mat eKernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::Mat dKernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(1, 2));
    cv::erode(xyzMap, depthMap, eKernel);
    depthMap = xyzMap.clone();

    //cv::medianBlur(xyzMap, depthMap, 3);

    cv::Mat mask = cv::Mat::zeros(depthMap.rows, depthMap.cols, depthMap.type());

    for (auto r = depthMap.rows - 1; r >= 0; r-=floodfill_interval)
    {
        for (auto c = 0; c < depthMap.cols; c+=floodfill_interval)
        {
            if (depthMap.at<cv::Vec3f>(r, c)[2] > 0.2)
            {
                mask.setTo(cv::Scalar(0, 0, 0));
                int pts = floodFill(c, r, depthMap, mask, max_distance);

                if (pts > min_size)
                {
                    //cv::medianBlur(mask, mask, 3);
                    cv::dilate(mask, mask, dKernel);
                    clusters.push_back(mask.clone());
                }
            }
        }
    }
}

/***
Performs floodfill on depthMap
***/
int DepthCamera::floodFill(int seed_x, int seed_y, cv::Mat& depthMap, cv::Mat& mask, double max_distance)
{
    static std::vector<cv::Point> stk;

    if (stk.size() <= depthMap.rows * depthMap.cols) {
        stk.resize(depthMap.rows * depthMap.cols + 1);
    }

    stk[0] = cv::Point(seed_x, seed_y);
    int stkPtr = 1;

    int total = 0;

    while (stkPtr) {
        int x = stk[--stkPtr].x, y = stk[stkPtr].y;

        if (x < 0 || x >= depthMap.cols || y < 0 || y >= depthMap.rows || depthMap.at<cv::Vec3f>(y, x)[2] < 0.1)
            continue;

        total += (depthMap.at<cv::Vec3f>(y, x)[2] > 0.2);

        mask.at<cv::Vec3f>(y, x) =  depthMap.at<cv::Vec3f>(y, x);
        depthMap.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);

        cv::Point nxtPoints[4] = { cv::Point(x + 1, y), cv::Point(x - 1, y), cv::Point(x, y + 1), cv::Point(x, y - 1) };

        for (int i = 0; i < sizeof nxtPoints / sizeof(nxtPoints[0]); ++i) {
            cv::Point adjPt = nxtPoints[i];

            if (adjPt.x < 0 || adjPt.x >= depthMap.cols || adjPt.y < 0 || adjPt.y >= depthMap.rows ||
                depthMap.at<cv::Vec3f>(adjPt.y, adjPt.x)[2] == 0)
                continue;

            double dist = Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x),
                                                    depthMap.at<cv::Vec3f>(adjPt.y, adjPt.x));

            if (dist < max_distance) {
                stk[stkPtr++] = adjPt;
            }
        }
    }

    return total;
}

///***
//Check whether candidate point is close enough to neighboring points
//***/
//bool DepthCamera::closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance)
//{
//	auto num_close = 0;
//	if (x - 1 < 0 || depthMap.at<cv::Vec3f>(y, x - 1)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x - 1)) < max_distance) {
//		num_close++;
//	}
//	if (x + 1 >= depthMap.cols || depthMap.at<cv::Vec3f>(y, x + 1)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x + 1)) < max_distance) {
//		num_close++;
//	}
//	if (y - 1 < 0 || depthMap.at<cv::Vec3f>(y - 1, x)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y - 1, x)) < max_distance) {
//		num_close++;
//	}
//	if (y + 1 >= depthMap.rows || depthMap.at<cv::Vec3f>(y + 1, x)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y + 1, x)) < max_distance) {
//		num_close++;
//	}
//
//	if (num_close >= num_neighbors) {
//		return true;
//	}
//
//	return false;
//}

/***
Remove noise on zMap and xyzMap based on INVALID_FLAG_VALUE and CONFIDENCE_THRESHOLD
***/
void DepthCamera::removeNoise()
{

    for (auto y = 0; y < xyzMap.rows; y++)
    {
        for (auto x = 0; x < xyzMap.cols; x++)
        {
            if (ampMap.data != nullptr)
            {
                if (xyzMap.at<cv::Vec3f>(y, x)[2] > 0.9f || ampMap.at<float>(y, x) < CONFIDENCE_THRESHHOLD)
                {
                    xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
                    xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
                    xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
                }
            }
            else
            {
                if (xyzMap.at<cv::Vec3f>(y, x)[2] > 0.9f)
                {
                    xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
                    xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
                    xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
                }
            }
        }
    }

    cv::Mat channels[3];
    cv::split(xyzMap, channels);

    if (static_cast<float>(cv::countNonZero(channels[2])) / (xyzMap.rows*xyzMap.cols) > 0.5)
    {
        badInput = true;
    }
    else {
        badInput = false;
    }
}

void DepthCamera::removePoints(std::vector<cv::Point2i> points)
{
    for (auto i = 0; i < points.size(); i++)
    {
        auto x = points[i].x;
        auto y = points[i].y;
        xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
        xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
        xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
    }
}

int DepthCamera::getWidth() const
{
    return X_DIMENSION;
}

int DepthCamera::getHeight() const
{
    return Y_DIMENSION;
}

cv::Mat DepthCamera::getXYZMap() const
{
    return xyzMap;
}

cv::Mat DepthCamera::getAmpMap() const
{
    return ampMap;
}

cv::Mat DepthCamera::getFlagMap() const
{
    return flagMap;
}

std::vector<cv::Mat> DepthCamera::getClusters() const
{
    return clusters;
}

void DepthCamera::initializeImages()
{
    auto dimension = cv::Size(X_DIMENSION, Y_DIMENSION);
    ampMap = cv::Mat(dimension, CV_32FC1);
    xyzMap = cv::Mat(dimension, CV_32FC3);
    flagMap = cv::Mat(dimension, CV_8UC1);
    clusters.clear();
}

/***
write a frame into file located at "destination"
***/
bool DepthCamera::writeImage(std::string destination) const
{
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
bool DepthCamera::readImage(std::string source)
{
    cv::FileStorage fs;
    fs.open(source, cv::FileStorage::READ);
    initializeImages();
    fs["xyzMap"] >> xyzMap;
    fs["ampMap"] >> ampMap;
    fs["flagMap"] >> flagMap;
    fs.release();

    if (xyzMap.rows == 0 || ampMap.rows == 0 || flagMap.rows == 0)
    {
        return false;
    }

    return true;
}
