#include "stdafx.h"
#include "Version.h"

#include "FrameObject.h"
#include "Hand.h"
#include "Visualizer.h"
#include "Util.h"

namespace ark {
    // Initialize default DetectionParams instance
    DetectionParams::Ptr DetectionParams::DEFAULT = DetectionParams::create();

    // Constructors
    FrameObject::FrameObject() { }

    FrameObject::FrameObject(const cv::Mat & depthMap, DetectionParams::Ptr params) {
        auto points = std::make_shared<std::vector<Point2i>>();
        auto points_xyz = std::make_shared<std::vector<Vec3f>>();

        for (int r = 0; r < depthMap.rows; ++r) {
            const Vec3f * ptr = depthMap.ptr<Vec3f>(r);
            for (int c = 0; c < depthMap.cols; ++c) {
                if (ptr[c][2] > 0) {
                    points->emplace_back(c, r);
                    points_xyz->push_back(ptr[c]);
                }
            }
        }

        initializeFrameObject(points, points_xyz, depthMap, params);
    }

    FrameObject::FrameObject(std::shared_ptr<std::vector<Point2i>> points_ij, 
        std::shared_ptr<std::vector<Vec3f>> points_xyz, const cv::Mat & depth_map,
        DetectionParams::Ptr params,
        bool sorted, int points_to_use) {
        initializeFrameObject(points_ij, points_xyz, depth_map, params, sorted, points_to_use);
    }

    Point2i FrameObject::findCenter(std::vector<Point2i> contour)
    {
        //using image moments to find center of mass of the object
        //Cx=M10/M00 and Cy=M01/M00
        cv::Moments M = cv::moments(contour, false);
        Point2i center = Point2i(static_cast<int>(M.m10) / M.m00, static_cast<int>(M.m01) / M.m00);
        return center;
    }

    int FrameObject::getContourScalingFactor() const {
        return 1;
    }

    const Point2i & FrameObject::getCenterIJ()
    {
        if (centerIj.x == INT_MAX)
            centerIj = util::findCentroid(xyzMap) + topLeftPt;
        return centerIj;
    }

    const Vec3f & FrameObject::getCenter()
    {
        if (centerXyz[0] == FLT_MAX)
            centerXyz =
            util::averageAroundPoint(xyzMap, getCenterIJ() - topLeftPt, params->xyzAverageSize);
        return centerXyz;
    }

    float FrameObject::getDepth()
    {
        if (avgDepth == -1) avgDepth = util::averageDepth(xyzMap);
        return avgDepth;
    }

    double FrameObject::getSurfArea() {
        if (surfaceArea == -1) {
            // lazily compute SA on demand
            surfaceArea = util::surfaceArea(fullMapSize, *points, *points_xyz);
        }
        return surfaceArea;
    }

    cv::Rect FrameObject::getBoundingBox() const
    {
        return cv::Rect(topLeftPt.x, topLeftPt.y, xyzMap.cols, xyzMap.rows);
    }



    const std::vector<Point2i> & FrameObject::getContour()
    {
        if (points != nullptr && points_xyz != nullptr && contour.size() == 0) {
            computeContour(xyzMap, points.get(), points_xyz.get(), topLeftPt, num_points);
        }

        return contour;
    }

    const std::vector<Point2i> & FrameObject::getConvexHull()
    {
        if (points != nullptr && convexHull.size() == 0) {
            if (getContour().size() > 1)
            {
                // Find convex hull
                cv::convexHull(contour, convexHull, false, true);
                cv::convexHull(contour, indexHull, false, false);
            }
        }

        return convexHull;
    }

    const cv::Mat & FrameObject::getDepthMap()
    {
        if (fullXyzMap.rows == 0) {
            // lazily compute full depth map on demand
            fullXyzMap = cv::Mat::zeros(fullMapSize, CV_32FC3);
            xyzMap.copyTo(fullXyzMap(getBoundingBox()));
        }

        return fullXyzMap;
    }

    // helper for performing morphological operations on gray map
    void FrameObject::morph(int erode_sz, int dilate_sz, bool dilate_first) {
        if (dilate_sz == -1) dilate_sz = erode_sz;

        cv::Mat eKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erode_sz, erode_sz)),
            dKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_sz, dilate_sz));

        if (dilate_first) cv::dilate(grayMap, grayMap, dKernel);
        cv::erode(grayMap, grayMap, eKernel);
        if (!dilate_first) cv::dilate(grayMap, grayMap, dKernel);
    }
    
    void FrameObject::computeContour(const cv::Mat & xyzMap, 
        const std::vector<cv::Point> * points,
        const std::vector<cv::Vec3f> * points_xyz,
        cv::Point topLeftPt,
        int num_points) {

        computeGrayMap(xyzMap, points, points_xyz, topLeftPt, num_points);

        std::vector<std::vector<Point2i> > contours;

        cv::Mat thresh;
        cv::threshold(grayMap, thresh, 25, 255, cv::THRESH_BINARY);
        cv::findContours(thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
            2 * topLeftPt);

        int maxId = -1;

        for (int i = 0; i < (int)contours.size(); ++i)
        {
            if (maxId < 0 || contours[i].size() > contours[maxId].size())
            {
                maxId = i;
            }
        }

        if (maxId >= 0) cv::approxPolyDP(contours[maxId], contour, 0.002, true);
        else contour.push_back(Point2i(0, 0));

        if (getContourScalingFactor() != 1.0f) {
            for (int i = 0; i < (int)contour.size(); ++i)
                contour[i] /= getContourScalingFactor();
        }
    }

    void FrameObject::computeGrayMap(const cv::Mat & xyzMap,
        const std::vector<cv::Point> * points,
        const std::vector<cv::Vec3f> * points_xyz,
        cv::Point topLeftPt, int num_points, int thresh) {


        if (xyzMap.rows == 0 || xyzMap.cols == 0 || points == nullptr || points_xyz == nullptr) return;

        int points_to_use = num_points;
        if (points_to_use < 0) points_to_use = (int)points->size();

        if (points->size() < points_to_use || points_xyz->size() < points_to_use) return;

        grayMap = cv::Mat::zeros(xyzMap.size(), CV_8U);
        
        for (int i = 0; i < points_to_use; ++i) {
            uchar val = (uchar)((*points_xyz)[i][2] * 256.0);
            if (val >= thresh) {
                grayMap.at<uchar>((*points)[i] - topLeftPt) = val;
            }
        }

        morph(params->contourImageErodeAmount, params->contourImageDilateAmount, false);

        for (int i = 1; i < getContourScalingFactor(); i <<= 1) {
            cv::pyrUp(grayMap, grayMap);
        }
    }

    void FrameObject::initializeFrameObject(VecP2iPtr points_ij, 
        VecV3fPtr points_xyz, const cv::Mat & depth_map, DetectionParams::Ptr params,
        bool sorted, int points_to_use)
    {
        if (params == nullptr) {
            params = DetectionParams::DEFAULT;
        }

        if (points_to_use < 0 || points_to_use >(int)points_ij->size())
            num_points = (int)points_ij->size();
        else
            num_points = points_to_use;

        this->points = points_ij;
        this->points_xyz = points_xyz;

        if (!sorted) {
            util::radixSortPoints(*points_ij, depth_map.cols, depth_map.rows,
                num_points, points_xyz.get());
        }

        cv::Rect bounding(depth_map.cols, (*points_ij)[0].y, -1, (*points_ij)[num_points - 1].y);

        for (int i = 0; i < num_points; ++i) {
            Point2i pt = (*points_ij)[i];
            bounding.x = std::min(pt.x, bounding.x);
            bounding.width = std::max(pt.x, bounding.width);
        }

        bounding.width -= bounding.x - 1;
        bounding.height -= bounding.y - 1;

        xyzMap = cv::Mat::zeros(bounding.size(), depth_map.type());
        topLeftPt = Point2i(bounding.x, bounding.y);
        fullMapSize = depth_map.size();

        for (int i = 0; i < num_points; ++i) {
            Point2i pt = (*points_ij)[i] - topLeftPt;
            xyzMap.at<Vec3f>(pt) = (*points_xyz)[i];
        }

        this->params = params;
    }

    FrameObject::~FrameObject() { }

    const std::vector<Point2i> & FrameObject::getPointsIJ() const
    {
        return *points;
    }

    const std::vector<Vec3f> & FrameObject::getPoints() const
    {
        return *points_xyz;
    }
}
