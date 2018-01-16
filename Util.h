#pragma once
#include "stdafx.h"
#include "version.h"

namespace ark {
    /**
     * Namespace containing generic static helper functions.
     */
    namespace util
    {
        /**
        * Splits a string into components based on delimeter
        * @param string_in string to split
        * @param delimeters c_str of delimeters to split at
        * @return vector of string components
        */
        std::vector<std::string> split(char* string_in, char* delimeters);

        /**
        * Generates a random RGB color.
        * @return random RGB color in Vec3b format
        */
        Vec3b colorGenerator();

        /**
        * Return the hypotenuse length.
        * @param a leg length
        * @param b leg length
        * return hypotenuse length
        */
        float normalize(float a, float b);

        /*
        * Compute the euclidean distance between (x1,y1) and (x2,y2)
        * @param pt1 (x1, y1)
        * @param pt2 (x2, y2)
        * @return the euclidean distance between the two points
        */
        template<class T>
        float euclideanDistance(cv::Point_<T> pt1, cv::Point_<T> pt2);

        /**
        * Get euclidean distance between two 3D points.
        * @param pt1 point 1
        * @param pt2 point 2
        * @return euclidean distance
        */
        template<class T>
        float euclideanDistance(cv::Vec<T, 3> pt1, cv::Vec<T, 3> pt2);

        /**
        * Estimate the euclidean distance per pixel around a point on a depth map
        * @param xyz_map the input depth map
        * @param pt the point
        * @param radius radius to average distance per pixel
        * @return distance per pixel
        */
        double euclideanDistancePerPixel(cv::Mat xyz_map, Point2i pt, int radius);

        /**
        * Removes points on img with indicies defined in points.
        * @param img the input image
        * @param points list of indicies (i,j) to be set to 0
        * @return processed image
        */
        cv::Mat removePoints(cv::Mat img, std::vector<Point2i> points);

        /**
        * Average all non-zero values around a point.
        * @param img base image to use
        * @param pt the point of interest
        * @param radius number of neighboring points to be used for computing the average
        * @return average (x,y,z) value of the point of interest
        */
        Vec3f averageAroundPoint(cv::Mat img, Point2i pt, int radius);

        /**
        * Determine whether (x,y) is a non-zero point in the matrix.
        * @param xyz_map Input image
        * @param x x-coordinate of the point
        * @param y y-coordinate of the point
        * @return true if (x.y) is non-zero
        */
        bool isMember(cv::Mat xyz_map, int x, int y);

        /**
        * Find the average depth of a depth image
        * @param xyz_map depth image
        * @returns average depth in meters
        */
        double averageDepth(cv::Mat xyz_map);

        /**
        * Find the centroid of the point cloud
        * @param xyz_map input point cloud
        * @return (x,y) coordinate of the centroid
        */
        Point2i findCentroid(cv::Mat xyz_map);

        /**
         * Performs floodfill on a depth map starting from seed point (x,y).
         * @param seed_x x-coordinate of the seed point
         * @param seed_y y-coordinate of the seed point
         * @param [in] zMap the xyzMap point cloud
         * @param max_distance the maximum euclidean distance allowed between neighbors
         * @param output_ij_points optionally, pointer to a vector for storing ij coords of
                  the points in the component. This vector should be AT LEAST the size of the xyz map
         * @param output_xyz_points optionally, pointer to a vector for storing xyz coords of
                  the points in the component. This vector should be AT LEAST the size of the xyz map
         * @param [out] mask optionally, output image containing points visited by the floodfill
         * @returns number of points in component
         */
        int floodFill(int seed_x, int seed_y, cv::Mat& depthMap,
            std::vector <Point2i> * output_ij_points = nullptr,
            std::vector <Vec3f> * output_xyz_points = nullptr,
            double max_distance = 0.005,
            cv::Mat * mask = nullptr);

        /**
        * Compute the angle in radians 'pointij' is at from the origin, going clockwise starting from the bottom
        * @param pointij input point in ij coordinates
        * @returns angle from origin, CW from bottom
        */
        double pointToAngle(Point2i pointij);

        /**
        * Converts a point into a value representing the direction it is at from the origin, going clockwise starting from the bottom.
        * Note: the value returned is not necessarily the slope. However, points ordered by this quantity are guarenteed to be in order of angle.
        * This function returns x/y if pointij is in the 3rd quadrant, FLT_MAX/2 - x/y if in 2nd quadrant,
                             FLT_MAX/2 + x/y if in 1st quadrant, and FLT_MAX - x/y if in 4th quadrant.
        * @param pointij input point in ij coordinates
        * @returns
        */
        double pointToSlope(Point2i pointij);

        /**
        * Compute the angle in radians between two points in ij coordinates, optionally through a third point (defaults to origin)
        * @param a first point in ij oordinates
        * @param b second point in ij coordinates
        * @param center optional center point (angle goes from a - center - b)
        * @returns angle between points
        */
        double angleBetweenPoints(Point2i a, Point2i b, Point2i center = Point2i(0, 0));

        /**
         * Compute the magnitude of a point.
         * @param pt input point
         * @returns magnitude of point
         */
        template <class T>
        double magnitude(cv::Point_<T> pt);

        /**
         * Compute the magnitude of a point.
         * @param pt input point
         * @returns magnitude of point
         */
        template <class T>
        double magnitude(cv::Point3_<T> pt);

        /**
         * Compute the magnitude of a point.
         * @param pt input point
         * @returns magnitude of point
         */
        template <class T, int n>
        double magnitude(cv::Vec<T, n> pt);

        /**
         * Compute the angle between two 3D vectors.
         * @param a vector 1
         * @param b vector 2
         * @param center optionally, vector to subtract both a and b by before computing angle
         * @returns angle between vectors
         */
        double angleBetween3DVec(Vec3f a, Vec3f b, Vec3f center = Vec3f(0, 0, 0));

        /**
         * Checks if a point is within the bounds of an image.
         * @param [in] img the image
         * @param pt the point
         */
        bool pointInImage(const cv::Mat & img, const Point2i pt);

        /**
         * Checks if a point is on the edge of a rectangle with top left point at (0,0) and size 'size'
         * @param size the size object
         * @param pt the point
         * @param margin_tb max number of pixels from the top/bottom edges to be considered on edge
         * @param margin_lr max number of pixels from the left/right edges to be considered on edge
         */
        bool pointOnEdge(const cv::Size size, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

        /**
         * Checks if a point is on the edge of a rectangle
         * @param rect the rectangle
         * @param pt the point
         * @param margin_tb max number of pixels from the top/bottom edges to be considered on edge
         * @param margin_lr max number of pixels from the left/right edges to be considered on edge
         * @param scale the scale of the point's coordinates relative to the scale of the image
         */
        bool pointOnEdge(const cv::Rect rect, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

        /**
         * Checks if a point is on the edge of an image
         * @param [in] img the image
         * @param pt the point
         * @param margin_tb max number of pixels from the top/bottom edges to be considered on edge
         * @param margin_lr max number of pixels from the left/right edges to be considered on edge
         */
        bool pointOnEdge(const cv::Mat & img, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

        /**
         * Computes the area of the triangle defined by three vertices
         * @param a first vertex
         * @param b second vertex
         * @param c third vertex
         * @returns area of triangle, in real meters squared
         */
        float triangleArea(Vec3f a, Vec3f b, Vec3f c = Vec3f(0, 0, 0));

        /**
         * Computes the area of the quadrangle defined by four vertices
         * @param pts the vertices of the quadrangle
         * @returns area of quadrangle, in real meters squared
         */
        float quadrangleArea(Vec3f pts[4]);

        /**
         * Computes the approximate surface area of all visible clusters on a depth map.
         * @param [in] depthMap the input depth map. All points with 0 z-coordinate will be excluded.
         * @returns surface area, in meters squared
         */
        double surfaceArea(cv::Mat & depthMap);

        /**
         * Computes the approximate surface area of a cluster containing the points specified
         * @param [in] depth_map the input depth map.
         * @param [in] points_ij ij coords of points in the cluster. (warning: may be reordered)
         * @param [in] points_xyz xyz coords of points in the cluster. (warning: may be reordered)
         * @param [in] sorted if true, assumes that 'cluster' is sorted and does not sort it again
         * @param [in] cluster_size number of points in this cluster. By default, uses all points in the 'cluster' vector.
         * @returns surface area, in meters squared
         */
        double surfaceArea(const cv::Mat & depth_map,
            std::vector<Point2i> & points_ij,
            std::vector<Vec3f> & points_xyz,
            bool sorted = false, int cluster_size = -1);

        /**
          * Approximates the surface area of a depth map cluster, using circles
          * of the smallest possible radius among adjacent points.
          * @param shape the depth map input
          * @returns surface area, meters squared
          */
        double surfaceAreaCircle(cv::Mat shape);

        /**
          * Approximates the surface area of a depth map cluster by triangulation
          * among adjacent points
          * @param shape the depth map input
          * @returns surface area, meters squared
          */
        double surfaceAreaTriangulate(cv::Mat shape);

        /**
          * Sort points by y and then x coordinate using radix sort
          * @param points [in] vector of points to sort
          * @param width width of overall depth image
          * @param height height of overall depth image
          * @param num_points number of points in'points' to use. By default, uses all.
          * @param points_xyz pointer to vector of associated xyz points. If provided, sorted along with ij points
         */
        void radixSortPoints(std::vector<Point2i> & points,
            int width, int height,
            int num_points = -1,
            std::vector<Vec3f> * points_xyz = nullptr);

        /**
         * Find a nonzero point on 'cluster' close to 'starting_point' by searching in a spiral from the starting point.
         * Used for snapping computed centroids, etc. to actual points on the object.
         * If the value at 'starting_point' is nonzero, then returns 'starting_point' without proceeding.
         * @param cluster the depth map representing the cluster
         * @param starting_point startin point of search
         * @param max_attempts maximum number of points to look at before cutting off the
                            search and simply returning the starting point
         * @returns first nonzero point on 'cluster' close to 'starting_point' encountered by travelling in a spiral
         */
        Point2i nearestPointOnCluster(const cv::Mat cluster, Point2i starting_point, int max_attempts = 500);

        /**
         * Find the center and radius of the largest inscribed circle within a certain object's contour
         * @param contour the cluster's contour
         * @param cluster ij coordinates of all points in the cluster
         * @param cluster_xyz xyz coordinates of all points in the cluster
         * @param cluster_size number of points in cluster to use (defaults to all)
         * @param top_dist_thresh maximum distance between the center of the circle and the top point in the cluster
         * @param[out] radius output pointer for the radius of circle
         * @param step step while iterating through cluster of points
         * @returns the center of the circle
         */
         Point2f largestInscribedCircle(const std::vector<Point2i> & contour,
            const std::vector<Point2i> & cluster,
            const std::vector<Vec3f> & cluster_xyz,
            int cluster_size = -1,
            float top_dist_thresh = FLT_MAX,
            double * radius = nullptr,
            int step = 256);

        /**
         * Find the curvature of a contour in radians near the specified point
         * @param contour the input contour
         * @param index the index of the target point within the contour
         * @param start the distance from the target point to begin averaging curvature
         * @param end the distance from the target point to stop averaging curvature
         * @returns curvature in radians at the point
         */
        double contourCurvature(const std::vector<Point2i> & contour, int index,
            int start = 2, int end = 5);

        /**
         * Compares two points (Point, Point2f, Vec3i or Vec3f),
         * first by x, then y, then z (if available). Used for sorting points.
         */
        template<class T>
        class PointComparer {
        public:
            PointComparer(bool reverse = false, bool compare_y_then_x = false) {
                this->reverse = reverse;
                this->compare_y_then_x = compare_y_then_x;
            }
            // Compare two points. Returns true if a is less than b.
            bool operator()(T a, T b);
        private:
            bool reverse = false, compare_y_then_x = false;
        };
    };
}
