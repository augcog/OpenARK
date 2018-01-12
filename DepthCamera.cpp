#include "stdafx.h"
#include "version.h"
#include "DepthCamera.h"
#include "Hand.h"
#include "Object3D.h"

namespace ark {
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

        std::vector<Point2i> pts(xyzMap.rows * xyzMap.cols + 1);
        std::vector<Point3f> pts_xyz(xyzMap.rows * xyzMap.cols + 1);

        static const Point3f zeros = Point3f(0, 0, 0);

        for (int r = 0; r < xyzMap.rows; ++r)
        {
            Point3f * ptr = xyzMap.ptr<Point3f>(r);

            for (int c = 0; c < xyzMap.cols; ++c)
            {
                if (ptr[c][2] > 0)
                {
                    int points_in_comp =
                        floodFill(c, r, xyzMap, &pts, &pts_xyz, max_distance, mask);

                    if (points_in_comp > min_points)
                    {
                        double area = util::surfaceArea(this->xyzMap, pts, pts_xyz,
                            false, points_in_comp);

                        if (area > min_size && area < max_size) {
                            clusters.push_back(mask.clone());
                            clusterAreas.push_back(area);
                        }
                    }

                    for (int i = 0; i < points_in_comp; ++i) {
                        mask.at<Point3f>(pts[i]) = zeros;
                    }
                }
            }
        }
    }

    void DepthCamera::construct3DObject(
        std::vector<Object3D> * output_objects,
        std::vector<Point2i> * points_ij,
        std::vector<Point3f> * points_xyz, cv::Mat * depth_map,
        const ObjectParams * params) {

        Object3D obj = Object3D(*points_ij, *points_xyz, *depth_map,
            params);
        (*output_objects).push_back(obj);
    }

    std::vector<Object3D> & DepthCamera::queryObjects(const ObjectParams * params)
    {
        // default parameters
        if (params == nullptr) params = &ObjectParams();

        // do clustering using flood fill
        if (objects.size() == 0) {
            cv::Mat xyzMap = this->xyzMap.clone();

            std::vector<Point2i> allIjPoints(xyzMap.rows * xyzMap.cols + 1);
            std::vector<Point3f> allXyzPoints(xyzMap.rows * xyzMap.cols + 1);

            const Point3f zeros = Point3f(0, 0, 0);

            for (int r = 0; r < xyzMap.rows; r += params->clusterInterval)
            {
                Point3f * ptr = xyzMap.ptr<Point3f>(r);

                for (int c = 0; c < xyzMap.cols; c += params->clusterInterval)
                {
                    if (ptr[c][2] > 0)
                    {
                        int points_in_comp = floodFill(c, r, xyzMap,
                            &allIjPoints, &allXyzPoints, params->clusterMaxDistance);

                        if (points_in_comp > params->clusterMinPoints)
                        {
                            // if matching required conditions, construct 3D object
                            Object3D obj = Object3D(allIjPoints, allXyzPoints, this->xyzMap,
                                params, false, points_in_comp);
                            objects.push_back(obj);
                        }
                    }
                }
            }
        }

        return objects;
    }

    std::vector<Object3D> & DepthCamera::queryHands(const ObjectParams * params) {
        if (params == nullptr) params = &ObjectParams();

        queryObjects(params);

        hands.clear();

        float bestHandDist = FLT_MAX, handObjectIndex = -1;

        for (int i = 0; i < objects.size(); ++i) {
            Object3D obj = objects[i];

            if (obj.hasHand) {
                float distance = obj.getDepth();

                if (distance < bestHandDist) {
                    handObjectIndex = i;
                    bestHandDist = distance;
                }

                if (obj.getHand().svm_confidence > params->handSVMHighConfidenceThresh) {
                    if (handObjectIndex == i) handObjectIndex = -1; // avoid duplicate hand
                    hands.push_back(obj);
                }
            }
        }

        if (handObjectIndex != -1) {
            hands.push_back(objects[handObjectIndex]);
        }

        return hands;
    }

    bool DepthCamera::operator()(bool removeNoise)
    {
        return nextFrame(removeNoise);
    }

    bool DepthCamera::operator >> (cv::Mat & img)
    {
        if (!nextFrame()) return false;

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
        std::vector <Point2i> * output_ij_points,
        std::vector <Point3f> * output_xyz_points,
        double max_distance,
        cv::Mat& mask)
    {
        /*
        Listing of adjacent points to go to in each floodfill step ((6, 0) means to go right 6).
        Goes to 6,0, etc to fill in small gaps
        */
        static const Point2i nxtPoints[] =
        {
            //Point2i(-6, 0),  
            Point2i(-1, 0),
            //Point2i(0, -6), 
            Point2i(0, -1),
            Point2i(0, 1),
            //Point2i(0, 6),
            Point2i(1, 0),
            //Point2i(6, 0),   
        };

        static const int nNxtPoints = (sizeof nxtPoints) / (sizeof nxtPoints[0]);

        // stack for storing the 2d and 3d points
        static std::vector<std::pair<Point2i, Point3f> > stk;

        // permanently allocate memory for our stack
        if (stk.size() <= depthMap.rows * depthMap.cols) {
            stk.resize(depthMap.rows * depthMap.cols + 1);
        }

        // add seed to stack
        Point2i seed = Point2i(seed_x, seed_y);
        stk[0] = std::make_pair(seed, depthMap.at<Point3f>(seed));

        // pointer to top (first empty index) of stack
        int stkPtr = 1;
        // counts the total number of points visited
        int total = 0;

        // begin DFS
        while (stkPtr > 0) {
            // pop current point from stack
            Point2i pt = stk[--stkPtr].first;
            Point3f & xyz = stk[stkPtr].second;

            if (!util::pointInImage(depthMap, pt)) continue;
            if (mask.rows) mask.at<Point3f>(pt) = Point3f(xyz);
            depthMap.at<Point3f>(pt)[2] = 0;

            // put point into the output vectors if provided
            if (output_ij_points) (*output_ij_points)[total] = pt;
            if (output_xyz_points) (*output_xyz_points)[total] = xyz;

            // increment the total # of points
            ++total;

            // go to each adjacent point
            for (int i = 0; i < nNxtPoints; ++i) {
                Point2i adjPt = pt + nxtPoints[i];

                // stop if outside bound of image
                if (!util::pointInImage(depthMap, adjPt)) continue;

                Point3f & adjXyz = depthMap.at<Point3f>(adjPt);

                // stop if already visited
                if (adjXyz[2] == 0) continue;

                // compute 3D distance
                double dist = util::euclideanDistance(xyz, adjXyz);
                //scaled_dist_thresh = max_distance;

         //// scale distance threshold for points with higher distance
         //if (abs(nxtPoints[i].x + nxtPoints[i].y) != 1)
             //scaled_dist_thresh *= 4;

         // update & go to if point is close enough
                if (dist < max_distance) {
                    stk[stkPtr++] = std::make_pair(adjPt, Point3f(adjXyz));
                    depthMap.at<Point3f>(adjPt)[2] = 0;
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
            Point3f * ptr = xyzMap.ptr<Point3f>(r);
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

    int DepthCamera::getWidth() const
    {
        return X_DIMENSION;
    }

    int DepthCamera::getHeight() const
    {
        return Y_DIMENSION;
    }

    cv::Size DepthCamera::getSize() const
    {
        return cv::Size(getWidth(), getHeight());
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
}
