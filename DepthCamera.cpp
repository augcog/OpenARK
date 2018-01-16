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
        std::vector<Vec3f> pts_xyz(xyzMap.rows * xyzMap.cols + 1);

        static const Vec3f zeros = Vec3f(0, 0, 0);

        for (int r = 0; r < xyzMap.rows; ++r)
        {
            Vec3f * ptr = xyzMap.ptr<Vec3f>(r);

            for (int c = 0; c < xyzMap.cols; ++c)
            {
                if (ptr[c][2] > 0)
                {
                    int points_in_comp =
                        util::floodFill(c, r, xyzMap, &pts, &pts_xyz, max_distance, &mask);

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
                        mask.at<Vec3f>(pts[i]) = zeros;
                    }
                }
            }
        }
    }

    void DepthCamera::construct3DObject(
        std::vector<Object3D> * output_objects,
        std::vector<Point2i> * points_ij,
        std::vector<Vec3f> * points_xyz, cv::Mat * depth_map,
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
            std::vector<Vec3f> allXyzPoints(xyzMap.rows * xyzMap.cols + 1);

            const Vec3f zeros = Vec3f(0, 0, 0);

            int clusterMinPoints;
            if (params->clusterMinPoints == -1)
                clusterMinPoints = xyzMap.rows * xyzMap.cols / 30;
            else
                clusterMinPoints = params->clusterMinPoints;

            for (int r = 0; r < xyzMap.rows; r += params->clusterInterval)
            {
                Vec3f * ptr = xyzMap.ptr<Vec3f>(r);

                for (int c = 0; c < xyzMap.cols; c += params->clusterInterval)
                {
                    if (ptr[c][2] > 0)
                    {
                        int points_in_comp = util::floodFill(c, r, xyzMap,
                            &allIjPoints, &allXyzPoints, params->clusterMaxDistance);

                        if (points_in_comp >= clusterMinPoints)
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

    /***
    Remove noise on zMap and xyzMap based on INVALID_FLAG_VALUE and CONFIDENCE_THRESHOLD
    ***/
    void DepthCamera::removeNoise()
    {
        int nonZero = 0;
        for (int r = 0; r < xyzMap.rows; ++r)
        {
            Vec3f * ptr = xyzMap.ptr<Vec3f>(r);
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
