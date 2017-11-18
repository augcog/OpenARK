#include "stdafx.h"
#include "DepthCamera.h"

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
    // size 1x2 works well (by trial and error...)
    cv::Mat dKernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(1, 2));

    cv::erode(xyzMap, depthMap, eKernel);
    depthMap = xyzMap.clone();

    cv::Mat mask = cv::Mat::zeros(depthMap.rows, depthMap.cols, depthMap.type());
    std::vector<cv::Point> pts(depthMap.rows * depthMap.cols + 1);

    for (int r = depthMap.rows - 1; r >= 0; r-=floodfill_interval)
    {
        cv::Vec3f * ptr = depthMap.ptr<cv::Vec3f>(r);
        for (int c = 0; c < depthMap.cols; c+=floodfill_interval)
        {
            if (ptr[c][2] > 0.2)
            {
                int comp_size = floodFill(c, r, depthMap, mask, max_distance, &pts);

                if (comp_size > min_size)
                {
                    cv::Mat cluster;
                    cv::dilate(mask, cluster, dKernel);
                    clusters.push_back(cluster);
                }

                for (int i = 0; i < comp_size; ++i) {
                    mask.at<cv::Vec3f>(pts[i]) = cv::Vec3f(0, 0, 0);
                }
            }
        }
    }
}

/***
Performs floodfill on depthMap
***/
int DepthCamera::floodFill(int seed_x, int seed_y, cv::Mat& depthMap, cv::Mat& mask, double max_distance, std::vector <cv::Point> * output_points)
{
    static std::vector<cv::Point> stk;

    if (stk.size() <= depthMap.rows * depthMap.cols) {
        // permanently allocate the space for a stack
        stk.resize(depthMap.rows * depthMap.cols + 1);
    }

    stk[0] = cv::Point(seed_x, seed_y);
    int stkPtr = 1, total = 0;

    while (stkPtr) {
        int x = stk[--stkPtr].x, y = stk[stkPtr].y;

        if (x < 0 || x >= depthMap.cols || y < 0 || y >= depthMap.rows || depthMap.at<cv::Vec3f>(y, x)[2] < 0.1)
            continue;

        mask.at<cv::Vec3f>(y, x) =  depthMap.at<cv::Vec3f>(y, x);
        depthMap.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);

        // set output point
        if (output_points) (*output_points)[total++] = cv::Point(x, y);

        cv::Point nxtPoints[4] = 
                 { cv::Point(x + 1, y), cv::Point(x - 1, y), cv::Point(x, y + 1), cv::Point(x, y - 1) };

        for (int i = 0; i < sizeof nxtPoints / sizeof(nxtPoints[0]); ++i) {
            cv::Point adjPt = nxtPoints[i];

            if (adjPt.x < 0 || adjPt.x >= depthMap.cols || adjPt.y < 0 || adjPt.y >= depthMap.rows ||
                depthMap.at<cv::Vec3f>(adjPt.y, adjPt.x)[2] < 0.1)
                continue;

            double dist = Util::euclideanDistance3D(depthMap.at<cv::Vec3f>(y, x),
                                                    depthMap.at<cv::Vec3f>(adjPt.y, adjPt.x));

            if (dist < max_distance) {
                stk[stkPtr++] = adjPt;
            }
        }
    }

    return total;
}

/***
Remove noise on zMap and xyzMap based on INVALID_FLAG_VALUE and CONFIDENCE_THRESHOLD
***/
void DepthCamera::removeNoise()
{
    int nonZero = 0;
    for (int r = 0; r < xyzMap.rows; ++r)
    {
        cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);
        float * ampptr = ampMap.ptr<float>(r);

        for (int c = 0; c < xyzMap.cols; ++c)
        {
            if (ptr[c][2] > 0.0f) {
                ++nonZero;
                if (ptr[c][2] > 0.9f && (ampMap.data == nullptr || ampptr[c] < CONFIDENCE_THRESHHOLD)) {
                    ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
                }
            }
        }
    }

    badInput = (static_cast<float>(nonZero) / (xyzMap.rows*xyzMap.cols) > 0.9);
}

void DepthCamera::removePoints(std::vector<cv::Point2i> points)
{
    for (int i = 0; i < points.size(); i++)
    {
        int x = points[i].x, y = points[i].y;
        xyzMap.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);
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
