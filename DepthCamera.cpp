#include "stdafx.h"
#include "DepthCamera.h"

DepthCamera::~DepthCamera()
{
}

void DepthCamera::nextFrame(bool removeNoise)
{
    objects.clear();
    clusters.clear();
    clusterAreas.clear();
    update();
    if (removeNoise) this->removeNoise();
}

void DepthCamera::computeClusters(double max_distance, double ir_distance, int min_points,
                                  double min_size, double max_size, int dilate_amount, int floodfill_interval)
{
    clusters.clear();
    clusterAreas.clear();

    const cv::Mat dKernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_amount, dilate_amount));

    cv::Mat xyzMap;
    cv::dilate(this->xyzMap, xyzMap, dKernel1);

    cv::Mat irImage;
    if (hasIRImage()) {
        irImage = this->irImage;
        //cv::erode(this->irImage, irImage, ireKernel);
        //cv::dilate(irImage, irImage, irdKernel);
    }
    else {
        irImage = cv::Mat::ones(xyzMap.rows, xyzMap.cols, CV_8U);
    }

    cv::Mat mask = cv::Mat::zeros(xyzMap.rows, xyzMap.cols, xyzMap.type());
    std::vector<cv::Point> pts(xyzMap.rows * xyzMap.cols + 1);

    const cv::Vec3f zeros = cv::Vec3f(0, 0, 0);
    int total = 0;

    for (int r = xyzMap.rows - 1; r >= 0; r -= floodfill_interval)
    {
        cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);

        for (int c = 0; c < xyzMap.cols; c += floodfill_interval)
        {
            total += 1;
            if (ptr[c][2] > 0)
            {
                int points_in_comp = floodFill(c, r, xyzMap, &pts, max_distance, irImage, ir_distance, mask);
                
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

std::vector<Object3D> DepthCamera::queryObjects(double max_distance, double ir_distance, int min_points,
                                  double min_size, double max_size, int dilate_amount, int floodfill_interval)
{
    if (objects.size() == 0) {
        const cv::Mat dKernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_amount, dilate_amount));

        cv::Mat xyzMap;
        cv::dilate(this->xyzMap, xyzMap, dKernel1);

        cv::Mat irImage;
        if (hasIRImage()) {
            irImage = this->irImage;
            //cv::erode(this->irImage, irImage, ireKernel);
            //cv::dilate(irImage, irImage, irdKernel);
        }
        else {
            irImage = cv::Mat::ones(xyzMap.rows, xyzMap.cols, CV_8U);
        }

        //cv::Mat mask = cv::Mat::zeros(xyzMap.rows, xyzMap.cols, xyzMap.type());
        std::vector<cv::Point> allPoints(xyzMap.rows * xyzMap.cols + 1);

        const cv::Vec3f zeros = cv::Vec3f(0, 0, 0);
        int total = 0;

        for (int r = xyzMap.rows - 1; r >= 0; r -= floodfill_interval)
        {
            cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);

            for (int c = 0; c < xyzMap.cols; c += floodfill_interval)
            {
                total += 1;

                if (ptr[c][2] > 0)
                {
                    int points_in_comp = floodFill(c, r, xyzMap, &allPoints, max_distance, irImage, ir_distance);

                    if (points_in_comp > min_points)
                    {
                        std::vector<cv::Point> points;
                        points.reserve(points_in_comp);

                        for (int i = 0; i < points_in_comp; ++i) {
                            points.push_back(allPoints[i]);
                        }
                        
                        Object3D obj = Object3D(points, this->xyzMap, false, min_size, max_size);
                        objects.push_back(obj);
                    }
                }
            }
        }
    }
    return objects;
}

std::vector<Hand> DepthCamera::queryHands(double max_distance, double ir_distance, int min_points,
    double min_size, double max_size, int dilate_amount, int floodfill_interval) {

    queryObjects(max_distance, ir_distance, min_points, min_size, max_size, dilate_amount, floodfill_interval);

    std::vector<Hand> result;

    float bestHandDist = FLT_MAX, handObjectIndex = -1;
    for (int i = 0; i < objects.size(); ++i) {
        Object3D obj = objects[i];


        if (obj.hasHand) {
            float distance = obj.getDepth();

            if (distance < bestHandDist) {
                handObjectIndex = i;
                bestHandDist = distance;
            }

            return result;
        }
    }

    if (handObjectIndex != -1) result.push_back(objects[handObjectIndex].getHand());

    return result;
}
/**
 * Performs floodfill on depthMap
 */
int DepthCamera::floodFill(int seed_x, int seed_y, cv::Mat& depthMap,
                          std::vector <cv::Point> * output_points, double max_distance,
                          cv::Mat& irImage, double ir_distance, cv::Mat& mask)
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
                   cv::Point(1, 0), cv::Point(-1, 0), cv::Point(0, 1), cv::Point(0, -1) ,
                   cv::Point(5, 0), cv::Point(-5, 0), cv::Point(0, 5), cv::Point(0, -5) 
                 };

    static const int nNxtPoints = (sizeof nxtPoints) / (sizeof nxtPoints[0]);

    stk[0] = cv::Point(seed_x, seed_y);
    stkXyz[0] = depthMap.at<cv::Vec3f>(stk[0]);
    int stkPtr = 1, total = 0;

    // while stack is not empty
    while (stkPtr > 0) {
        cv::Point pt = stk[--stkPtr];
        cv::Vec3f xyz = stkXyz[stkPtr];

        if (!Util::pointInImage(depthMap, pt)) continue;

        ushort ir = (irImage.cols ? irImage.at<ushort>(pt) : 1); 

        if (mask.rows) mask.at<cv::Vec3f>(pt) = xyz;
        depthMap.at<cv::Vec3f>(pt)[2] = 0;

        // add point to output vector, if one is provided
        if (output_points) 
            (*output_points)[total++] = cv::Point(pt);

        for (int i = 0; i < nNxtPoints; ++i) {
            // go to each adjacent point
            cv::Point adjPt = pt + nxtPoints[i];
            if (!Util::pointInImage(depthMap, adjPt)) continue; 

            cv::Vec3f adjXyz = depthMap.at<cv::Vec3f>(adjPt);
            if(adjXyz[2] == 0) continue;

            if (irImage.cols && !Util::pointInImage(irImage, adjPt)) continue; 
            ushort adjIr = (irImage.cols ? irImage.at<ushort>(adjPt) : 1); 

            double dist = Util::euclideanDistance3D(xyz, adjXyz);
            ushort irDiff = abs(ir - adjIr);

            double scaled_dist = max_distance * (abs(nxtPoints[i].x) + abs(nxtPoints[i].y));

            if (dist < scaled_dist && irDiff < ir_distance) {
                stkXyz[stkPtr] = adjXyz;
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
                if (ptr[c][2] > 0.95f && (ampMap.data == nullptr || ampptr[c] < CONFIDENCE_THRESHHOLD)) {
                    ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
                }
            }
        }
    }

    badInput = (static_cast<float>(nonZero) / (xyzMap.rows*xyzMap.cols) > 0.9);
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