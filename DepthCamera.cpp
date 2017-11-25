#include "stdafx.h"
#include "DepthCamera.h"
#include "Hand.h"
#include "Object3D.h"

DepthCamera::~DepthCamera() { }

bool DepthCamera::nextFrame(bool removeNoise)
{
    objects.clear();
    clusters.clear();
    clusterAreas.clear();
    update();
    if (!badInput() && removeNoise) this->removeNoise();
    return !badInput();
}

void DepthCamera::computeClusters(double max_distance, int min_points,
                                  double min_size, double max_size, int floodfill_interval)
{
    clusters.clear();
    clusterAreas.clear();

    //const cv::Mat dKernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_amount, dilate_amount));

    cv::Mat xyzMap = this->xyzMap.clone();

    //cv::dilate(this->xyzMap, xyzMap, dKernel1);


    cv::Mat mask = cv::Mat::zeros(xyzMap.rows, xyzMap.cols, xyzMap.type());
    std::vector<cv::Point> pts(xyzMap.rows * xyzMap.cols + 1);

    static const cv::Vec3f zeros = cv::Vec3f(0, 0, 0);

    for (int r = 0; r < xyzMap.rows; ++r)
    {
        cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);

        for (int c = 0; c < xyzMap.cols; ++c)
        {
            if (ptr[c][2] > 0)
            {
                int points_in_comp = floodFill(c, r, xyzMap, &pts, max_distance, mask);
                
                if (points_in_comp > min_points)
                {   
                    double area = Util::surfaceArea(this->xyzMap, pts, false, points_in_comp);
                    //cv::Mat cluster;
                    if (area > min_size && area < max_size) {
                        clusters.push_back(mask.clone());
                        clusterAreas.push_back(area);
                    }
                }

                for (int i = 0; i < points_in_comp; ++i) {
                    mask.at<cv::Vec3f>(pts[i]) = zeros;
                }
            }
        }
    }
}

std::vector<Object3D> DepthCamera::queryObjects(double min_size,
 double max_size,
                                  double max_distance, int min_points, int floodfill_interval)
{
    if (objects.size() == 0) {
        //const cv::Mat dKernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_amount, dilate_amount));

        cv::Mat origXyzMap, xyzMap;
        origXyzMap = this->xyzMap.clone();
        xyzMap = origXyzMap.clone();

        std::vector<cv::Point> allPoints(xyzMap.rows * xyzMap.cols + 1);

        const cv::Vec3f zeros = cv::Vec3f(0, 0, 0);

        for (int r = 0; r < xyzMap.rows; r += floodfill_interval)
        {
            cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);

            for (int c = 0; c < xyzMap.cols; c += floodfill_interval)
            {
                if (ptr[c][2] > 0)
                {
                    int points_in_comp = floodFill(c, r, xyzMap, &allPoints, max_distance);

                    if (points_in_comp > min_points)
                    {
                        Object3D obj = Object3D(allPoints, origXyzMap, false, points_in_comp, min_size, max_size);
                        objects.push_back(obj);
                    }
                }
            }
        }
    }
    return objects;
}

std::vector<Object3D> DepthCamera::queryHands(double hand_high_confidence_thresh, double max_distance, int min_points,
    double min_size, double max_size, int floodfill_interval) {

    queryObjects(min_size, max_size, max_distance, min_points, floodfill_interval);

    std::vector<Object3D> result;

    float bestHandDist = FLT_MAX, handObjectIndex = -1;

    for (int i = 0; i < objects.size(); ++i) {
        Object3D obj = objects[i];

        if (obj.hasHand) {
            float distance = obj.getDepth();

            if (distance < bestHandDist) {
                handObjectIndex = i;
                bestHandDist = distance;
            }

            if (obj.getHand().svm_confidence > hand_high_confidence_thresh) {
                if (handObjectIndex == i) handObjectIndex = -1; // avoid duplicate hand
                result.push_back(obj);
            }
        }
    }

    if (handObjectIndex != -1) {
        result.push_back(objects[handObjectIndex]);
    }

    return result;
}

bool DepthCamera::operator()(bool removeNoise)
{
    return nextFrame(removeNoise);
}

bool DepthCamera::operator >> (cv::Mat & img)
{
    if (!nextFrame()) return false;
    assert(img.rows == 0 || img.channels == 1 || img.channels == 3);

    if (img.rows == 0 || img.type() == CV_32FC3) {
        img = getXYZMap();
        return true;
    }
    else if (img.channels() == 1 && hasIRImage()) {
        img = getIRImage();
        return true;
    }
    else if (hasRGBImage()) {
        img = getRGBImage();
        return true;
    }
    
    return false;
}

bool DepthCamera::operator >> (std::vector<Object3D> & objs)
{
    if (!nextFrame()) return false;
    objs = queryObjects();
    return true;
}

bool DepthCamera::badInput()
{
    return _badInput;
}

/**
 * Performs floodfill on depthMap
 */
int DepthCamera::floodFill(int seed_x, int seed_y, cv::Mat& depthMap,
                          std::vector <cv::Point> * output_points, double max_distance, cv::Mat& mask)
{
    static std::vector<cv::Point> stk;
    static std::vector<cv::Vec3f> stkXyz;

    if (stk.size() <= depthMap.rows * depthMap.cols) {
        // permanently allocate the space for a stack
        stk.resize(depthMap.rows * depthMap.cols + 1);
        stkXyz.resize(depthMap.rows * depthMap.cols + 1);
    }

    static const cv::Point nxtPoints[] = 
                 {
                   cv::Point(-6, 0),  cv::Point(-1, 0),
                   cv::Point(0, -6), cv::Point(0, -1), cv::Point(0, 1), cv::Point(0, 6),
                   cv::Point(1, 0), cv::Point(6, 0),   
                 };

    static const int nNxtPoints = (sizeof nxtPoints) / (sizeof nxtPoints[0]);
    //static const cv::Vec3f zeros = cv::Vec3f(0, 0, 0);

    stk[0] = cv::Point(seed_x, seed_y);
    stkXyz[0] = depthMap.at<cv::Vec3f>(stk[0]);
    int stkPtr = 1, total = 0;

    // while stack is not empty
    while (stkPtr > 0) {
        cv::Point pt = stk[--stkPtr];
        cv::Vec3f & xyz = stkXyz[stkPtr];

        if (!Util::pointInImage(depthMap, pt)) continue;

        //ushort ir = (irImage.cols ? irImage.at<ushort>(pt) : 1); 

        if (mask.rows) mask.at<cv::Vec3f>(pt) = cv::Vec3f(xyz);
        depthMap.at<cv::Vec3f>(pt)[2] = 0;

        // add point to output vector, if one is provided
        if (output_points) (*output_points)[total++] = pt;

        for (int i = 0; i < nNxtPoints; ++i) {
            // go to each adjacent point
            cv::Point adjPt = pt + nxtPoints[i];
            if (!Util::pointInImage(depthMap, adjPt)) continue; 

            cv::Vec3f & adjXyz = depthMap.at<cv::Vec3f>(adjPt);
            if(adjXyz[2] == 0) continue;

            // Disabling IR segmentation
            //ushort adjIr;
            //if (irImage.cols) {
            //    if (!Util::pointInImage(irImage, adjPt)) continue; 
            //    adjIr = irImage.at<ushort>(adjPt);
            //}
            //else adjIr = 1;

            double dist = Util::euclideanDistance3D(xyz, adjXyz);
            //ushort irDiff = abs(ir - adjIr);

            double scaled_dist = max_distance;
            if (abs(nxtPoints[i].x + nxtPoints[i].y) != 1)
                scaled_dist *= 4;

            if (dist < scaled_dist /*&& irDiff < ir_distance*/) {
                stkXyz[stkPtr] = cv::Vec3f(adjXyz);
                stk[stkPtr++] = adjPt;
                depthMap.at<cv::Vec3f>(adjPt)[2] = 0;
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

    _badInput = (static_cast<float>(nonZero) / (xyzMap.rows*xyzMap.cols) > 0.9);
}

//void DepthCamera::removePoints(std::vector<cv::Point2i> points)
//{
//    for (int i = 0; i < points.size(); i++)
//    {
//        int x = points[i].x, y = points[i].y;
//        xyzMap.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);
//    }
//}

int DepthCamera::getWidth() const
{
    return X_DIMENSION;
}

int DepthCamera::getHeight() const
{
    return Y_DIMENSION;
}

const cv::Mat DepthCamera::getXYZMap() const
{
    return xyzMap;
}

const cv::Mat DepthCamera::getAmpMap() const
{
    return ampMap;
}

const cv::Mat DepthCamera::getFlagMap() const
{
    return flagMap;
}

std::vector<cv::Mat> DepthCamera::getClusters() const
{
    return clusters;
}

std::vector<double> DepthCamera::getClusterAreas() const
{
    return clusterAreas;
}

void DepthCamera::initializeImages()
{
    cv::Size dimension = cv::Size(X_DIMENSION, Y_DIMENSION);
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

bool DepthCamera::hasRGBImage() const {
    // Assume no RGB image, unless overridden
    return false;
}

const cv::Mat & DepthCamera::getRGBImage() {
    if (!hasRGBImage()) throw;
    return rgbImage;
}

bool DepthCamera::hasIRImage() const
{
    // Assume not available
    return false;
}

const cv::Mat & DepthCamera::getIRImage()
{
    if (!hasIRImage()) throw;
    return irImage;
}
