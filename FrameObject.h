#pragma once

#include "stdafx.h" 
#include "version.h"

// OpenARK headers
#include "version.h"
#include "ObjectParams.h"

namespace ark {
    /** Class representing a 3D object observed in a single frame */
    class FrameObject
    {
    public:
        /**
        * Constructs an empty FrameObject instance.
        */
        FrameObject();

        /**
        * Constructs an FrameObject instance based on an isolated point cloud.
        * Note: points not on the cluster must have 0 z-coordinate in the point cloud.
        * @param cluster_depth_map point cloud containing the object
        * @param params parameters for object/hand detection (if not specified, uses default params)
        */
        explicit FrameObject(const cv::Mat & cluster_depth_map,
            const ObjectParams * params = nullptr);

        /**
        * Construct an FrameObject instance from a vector of points.
        * @param [in] points vector of all points (in screen coordinates) belonging to the object
        * @param [in] depth_map the reference point cloud. (CAN contain points outside this object)
        * @param params parameters for object/hand detection (if not specified, uses default params)
        * @param sorted if true, assumes that 'points' is already ordered and skips sorting to save time.
        * @param points_to_use optionally, the number of points in 'points' to use for the object. By default, uses all points.
        */
        FrameObject(boost::shared_ptr<std::vector<Point2i>> points_ij,
            boost::shared_ptr<std::vector<Vec3f>> points_xyz,
            const cv::Mat & depth_map,
            const ObjectParams * params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );

        /**
        * Destructs an FrameObject instance.
        */
        ~FrameObject();

        /**
        * Gets a list of points in this object, in screen coordinates
        * @return list of points
        */
        const std::vector<Point2i> & getPointsIJ() const;

        /**
        * Gets a list of points in this object, in screen coordinates
        * @return list of points
        */
        const std::vector<Vec3f> & getPoints() const;

        /**
        * Gets approximate center of mass of object in screen coordinates
        * @return center of object in screen coordinates
        */
        const Point2i & getCenterIJ();

        /**
        * Gets approximate center of mass of object in real coordinates
        * @return center of object in real (xyz) coordinates
        */
        const Vec3f & getCenter();

        /**
        * Gets bounding box of object in screen coordinates
        * @return bounding box of object in screen coordinates
        */
        cv::Rect getBoundingBox() const;

        /**
        * Gets the average depth of the object
        * @return average z-coordinate of the object in meters
        */
        float getDepth();

        /**
        * Gets the visible surface area of the object
        * @return surface area in meters squared
        */
        double getSurfArea();

        /**
         * Get the depth map of the visible portion of this object
         * @return depth map of visible object
         */
        const cv::Mat & getDepthMap();

        /**
         * Find the largest 2D contour within this cluster
         * @return vector of points representing the largest 2D contour within this cluster
         */
        const std::vector<Point2i> & getContour();

        /**
        * Gets the 2D convex hull of this object
        * @return the 2D convex hull of this object, stored as a vector of points
        */
        const std::vector<Point2i> & getConvexHull();

    protected:
        /**
         * Stores ij coordinates of points in this cluster
         */
        boost::shared_ptr<std::vector<Point2i>> points = nullptr;

        /**
         * Stores xyz coordinates of points in this cluster
         */
        boost::shared_ptr<std::vector<Vec3f>> points_xyz = nullptr;

        /**
         * Stores number of points in 'points' used in the cluster
         */
        int num_points;

        /**
         * Top left point of bounding box of cluster
         */
        Point2i topLeftPt;

        /**
         * (Partial) XYZ map of cluster (CV_32FC3).
         */
        cv::Mat xyzMap;

        /**
         * Full XYZ map of cluster (CV_32FC3).
         */
        cv::Mat fullXyzMap = cv::Mat();

        /**
         * Grayscale image containing normalized depth (z) information from the regular xyzMap (CV_8U)
         * Note: 2x the size of xyzMap
         */
        cv::Mat grayMap;

        /**
         * Stores size of full xyz map
         */
        cv::Size fullMapSize;

        /**
         * Surface area in meters squared
         */
        double surfaceArea = -1;

        /**
         * Largest contour in object
         */
        std::vector<Point2i> contour;

        /**
         * Convex hull of object
         */
        std::vector<Point2i> convexHull;

        /**
         * Convex hull of this object, * with points stored as indices of the contour rather than Point2i
         */
        std::vector<int> indexHull;

        /**
         * Center of the object in screen coordinates.
         */
        Point2i centerIj = Point2i(INT_MAX, 0);

        /**
         * Center of the object in real coordinates.
         */
        Vec3f centerXyz = Vec3f(FLT_MAX, 0.0f, 0.0f);

        /**
         * Average depth of object
         */
        double avgDepth = -1.0;

        /** 
         * Amount the gray map is scaled by for contour finding.
         * Must be a power of 2, at least 1.
         * May be overridden in derived classes. The higher the scale, the
         * more time-consuming the contour detection process but the smoother the contour.
         * Default is 1 (no scaling).
         */
        virtual int getContourScalingFactor() const;

        /**
        * Find the center of mass of an object
        * @param contour contour of the object
        * @return point representing center of mass
        */
        static Point2i FrameObject::findCenter(std::vector<Point2i> contour);

        /**
        * Simplify a convex hull by clustering points within distance 'threshold'
        * @param convex_hull the convex hull
        * @param threshold maximum distance between points in a cluster
        * @return simplified convex hull
        */
        static std::vector<Point2i> FrameObject::clusterConvexHull(std::vector<Point2i> convex_hull, int threshold);

        /**
          * Perform erode-dilate morphological operations on the cluster's depth image
          * @param erode_sz size of erode kernel
          * @param dilate_sz size of dilate kernel (by default, takes same value as erodeAmt)
          * @param dilate_first if true, performs dilate before erode
          * @param gray_map if true, performs operations on the gray map instead of the xyz map
          */
        void morph(int erode_sz, int dilate_sz = -1, bool dilate_first = false, bool gray_map = false);

        /**
         * Compute the cluster's contour
         * @param input depth map containing points within bounding box of cluster
         * @param points points in cluster (absolute coordinates)
         * @param points_xyz xyz coords of points in cluster
         * @param topLeftPt top left point of cluster
         * @param num_points number of points in cluster
         * @param thresh the minimum z-coordinate of a point on the depth image below which the pixel will be zeroed out
         */
        void FrameObject::computeContour(const cv::Mat & xyzMap, 
            const std::vector<cv::Point> * points, 
            const std::vector<cv::Vec3f> * points_xyz,
            cv::Point topLeftPt, int num_points);

        /**
         * Compute the grayscale z-coordinate image of this cluster from the normal xyz map
         * @param input depth map containing points within bounding box of cluster
         * @param points points in cluster (absolute coordinates)
         * @param points_xyz xyz coords of points in cluster
         * @param topLeftPt top left point of cluster
         * @param num_points number of points in cluster
         * @param thresh the minimum z-coordinate of a point on the depth image below which the pixel will be zeroed out
         */
        void computeGrayMap(const cv::Mat & xyzMap, 
            const std::vector<cv::Point> * points, 
            const std::vector<cv::Vec3f> * points_xyz,
            cv::Point topLeftPt, int num_points,  int thresh = 0);

        /*
         * Parameters for object/hand detection
         */
        const ObjectParams * params = nullptr;

    private:
        /** Constructor helper */
        void initializeFrameObject(boost::shared_ptr<std::vector<Point2i>> points_ij,
            boost::shared_ptr<std::vector<Vec3f>> points_xyz,
            const cv::Mat & depth_map,
            const ObjectParams * params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );
    };

    /* Shared pointer for FrameObject **/
    typedef boost::shared_ptr<FrameObject> FrameObjectPtr;
}
