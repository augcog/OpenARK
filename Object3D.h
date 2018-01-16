#pragma once

#include "stdafx.h" 

// OpenARK headers
#include "version.h"
#include "Hand.h"
#include "Plane.h"
#include "ObjectParams.h"

// SVM integration
#include "HandClassifier.h"

namespace ark {
    class Object3D
    {
    public:
        /**
        * Whether the object is attached to the left edge of the frame.
        * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool leftEdgeConnected = false;

        /**
        * Whether the object is attached to the right edge of the frame.
        * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool rightEdgeConnected = false;

        /**
        * Constructs an empty Object3D instance.
        */
        Object3D();

        /**
        * Constructs an Object3D instance based on an isolated point cloud.
        * Note: points not on the cluster must have 0 z-coordinate in the point cloud.
        * @param cluster_depth_map point cloud containing the object
        * @param params parameters for object/hand detection (if not specified, uses default params)
        */
        explicit Object3D::Object3D(cv::Mat cluster_depth_map,
            const ObjectParams * params = nullptr);

        /**
        * Construct an Object3D instance from a vector of points.
        * @param [in] points vector of all points (in screen coordinates) belonging to the object
        * @param [in] depth_map the reference point cloud. (CAN contain points outside this object)
        * @param params parameters for object/hand detection (if not specified, uses default params)
        * @param sorted if true, assumes that 'points' is already ordered and skips sorting to save time.
        * @param points_to_use optionally, the number of points in 'points' to use for the object. By default, uses all points.
        */
        Object3D::Object3D(std::vector<Point2i> & points_ij,
            std::vector<Vec3f> & points_xyz,
            cv::Mat & depth_map,
            const ObjectParams * params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );

        /**
        * Destructs an Object3D instance.
        */
        ~Object3D();

        /**
        * Whether the object contains a hand.
        */
        bool hasHand = false;

        /**
        * Whether the object contains a plane.
        */
        bool hasPlane = false;

        /**
        * Whether the object contains a shape.
        * A shape is defined by anything that is not a plane or a hand
        */
        bool hasShape = false;

        /**
        * Gets instance of hand object if a hand is found.
        * @return instance of hand object
        */
        const Hand & getHand() const;

        /**
        * Gets instance of plane object is plane is found.
        * @return instance of plane object
        */
        const Plane & getPlane() const;

        /**
        * Gets instance of shape object.
        * @return instance of shape object
        */
        cv::Mat getShape() const;

        /**
        * Gets approximate center of mass of object in screen coordinates
        * @return center of object in screen coordinates
        */
        Point2i getCenterIj();

        /**
        * Gets approximate center of mass of object in real coordinates
        * @return center of object in real (xyz) coordinates
        */
        Vec3f getCenter();

        /**
        * Gets bounding box of object in screen coordinates
        * @return bounding box of object in screen coordinates
        */
        cv::Rect getBoundingBox() const;

        /**
        * Gets the average depth of the object
        * @returns average z-coordinate of the object in meters
        */
        float getDepth();

        /**
        * Gets the visible surface area of the object
        * @returns surface area in meters squared
        */
        double getSurfArea();

        /**
         * Get the depth map of the visible portion of this object
         * @returns depth map of visible object
         */
        const cv::Mat & getDepthMap();

        /**
         * Find the largest 2D contour within this cluster
         * @returns vector of points representing the largest 2D contour within this cluster
         */
        std::vector<Point2i> getContour();

        /**
        * Gets the 2D convex hull of this object
        * @return the 2D convex hull of this object, stored as a vector of points
        */
        std::vector<Point2i> getConvexHull();

    private:
        /**
         * Stores ij coordinates of points in this cluster
         */
        std::vector<Point2i> * points = nullptr;

        /**
         * Stores xyz coordinates of points in this cluster
         */
        std::vector<Vec3f> * points_xyz = nullptr;

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
         * Pointer to the hand instance.
         */
        boost::shared_ptr<Hand> hand = nullptr;

        /**
         * Pointer to the plane instance.
         */
        boost::shared_ptr<Plane> plane = nullptr;

        /**
         * The shape instance.
         */
        cv::Mat shape;

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
         * Determine whether the object is connected to an edge.
         */
        void checkEdgeConnected();

        /**
        * Check whether the object is a hand
        * @param cluster input cluster (defaults to &this->xyzMap)
        * @param points vector of points in cluster (defaults to this->points_xyz)
        * @param points_xyz vector of xyz coords of points in cluster (defaults to this->points_xyz)
        * @param num_points number of points
        * @param params parameters for hand detection (defaults to this->params)
        */
        boost::shared_ptr<Hand> checkForHand(const cv::Mat * cluster = nullptr,
            const std::vector<Point2i> * points = nullptr,
            const std::vector<Vec3f> * points_xyz = nullptr,
            cv::Point topLeftPt = cv::Point(INT_MAX, 0),
            int num_points = -1,
            const ObjectParams * params = nullptr);

        /**
        * Subroutine for computing the largest contour, convex hull, etc. for this 3D object.
        */
        void initializeObject();

        /**
        * Find the center of mass of an object
        * @param contour contour of the object
        * @returns point representing center of mass
        */
        static inline Point2i Object3D::findCenter(std::vector<Point2i> contour);

        /**
        * Simplify a convex hull by clustering points within distance 'threshold'
        * @param convex_hull the convex hull
        * @param threshold maximum distance between points in a cluster
        * @returns simplified convex hull
        */
        static std::vector<Point2i> Object3D::clusterConvexHull(std::vector<Point2i> convex_hull, int threshold);

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
        void Object3D::computeContour(const cv::Mat & xyzMap, const std::vector<cv::Point> * points, 
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
        void computeGrayMap(const cv::Mat & xyzMap, const std::vector<cv::Point> * points, 
            const std::vector<cv::Vec3f> * points_xyz, cv::Point topLeftPt, int num_points,  int thresh = 0);

        /*
         * Parameters for object/hand detection
         */
        const ObjectParams * params = nullptr;

        /**
        * SVM Hand classifier instance
        */
        static classifier::HandClassifier & handClassifier;
    };
}
