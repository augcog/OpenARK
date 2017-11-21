#pragma once
#include "stdafx.h"

/**
* Class containing generic helper functions.
*/
class Util
{
public:
    /**
    * Splits a string into components based on delimeter
    * @param string_in string to split 
    * @param delimeters c_str of delimeters to split at
    * @return vector of string components
    */
    static std::vector<std::string> split(char* string_in, char* delimeters);

    /**
    * Generates a random RGB color.
    * @return random RGB color in Vec3b format
    */
    static cv::Vec3b colorGenerator2();

    /**
    * Get euclidean distance between two 2D points.
    * @param x1 x-coordinate of point 1
    * @param y1 y-coordinate of point 1
    * @param x2 x-coordinate of point 2
    * @param y2 y-coordinate of point 2
    * @return the euclidean distance
    */
    static int getDistanceT(int x1, int y1, int x2, int y2);

    /**
    * Return the hypotenuse length.
    * @param a leg length
    * @param b leg length
    * return hypotenuse length
    */
    static float normalize(float a, float b);

    /*
    * Compute the euclidean distance between (x1,y1) and (x2,y2)
    * @param p1 (x1, y1)
    * @param p2 (x2, y2)
    * @return the euclidean distance between the two points
    */
    static double euclideanDistance2D(cv::Point p1, cv::Point pt2);

    /**
    * Get euclidean distance between two 3D points.
    * @param pt1 point 1
    * @param pt2 point 2
    * @return euclidean distance
    */
    static double euclideanDistance3D(cv::Vec3f pt1, cv::Vec3f pt2);

    //TODO needs description
    static double euclideanDistancePerPixel(cv::Mat xyzMap, cv::Point pt, int radius);

    /**
    * Removes points on img with indicies defined in points.
    * @param img the input image
    * @param points list of indicies (i,j) to be set to 0
    * @return processed image
    */
    static cv::Mat removePoints(cv::Mat img, std::vector<cv::Point2i> points);

    /**
    * Average all non-zero values around a point.
    * @param img base image to use
    * @param pt the point of interest
    * @param radius number of neighboring points to be used for computing the average
    * @return average (x,y,z) value of the point of interest
    */
    static cv::Vec3f averageAroundPoint(cv::Mat img, cv::Point2i pt, int radius);

    /**
    * Determine whether (x,y) is a non-zero point in the matrix.
    * @param xyzMap Input image
    * @param x x-coordinate of the point
    * @param y y-coordinate of the point
    * @return true if (x.y) is non-zero
    */
    static bool isMember(cv::Mat xyzMap, int x, int y);

    /**
    * Find the average depth of a depth image
    * @param xyzMap depth image
    * @returns average depth in meters
    */
    static double Util::averageDepth(cv::Mat xyzMap);

    /**
    * Find the centroid of the point cloud
    * @param xyzMap input point cloud
    * @return (x,y) coordinate of the centroid
    */
    static cv::Point findCentroid(cv::Mat xyzMap);

    /**
    * Compute the angle formed by 3 points
    * @return the angle formed
    */
    static double triangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3);

    /**
    * Perform flood fill on a depth map from the given coordinates
    */
    static void floodFill(int x, int y, cv::Mat& depthMap, cv::Mat& mask, double max_distance);


    /**
    * Compute the angle in radians 'pointij' is at from the origin, going clockwise starting from the bottom
    * @param pointij input point in ij coordinates
    * @returns angle from origin, CW from bottom
    */
    static double pointToAngle(cv::Point pointij);

    /**
    * Converts a point into a value representing the direction it is at from the origin, going clockwise starting from the bottom.
    * Note: the value returned is not necessarily the slope. However, points ordered by this quantity are guarenteed to be in order of angle.
    * This function returns x/y if pointij is in the 3rd quadrant, FLT_MAX/2 - x/y if in 2nd quadrant, 
                         FLT_MAX/2 + x/y if in 1st quadrant, and FLT_MAX - x/y if in 4th quadrant.
    * @param pointij input point in ij coordinates
    * @returns  
    */
    static double pointToSlope(cv::Point pointij);

    /**
    * Compute the angle in radians between two points in ij coordinates, optionally through a third point (defaults to origin)
    * @param a first point in ij oordinates
    * @param b second point in ij coordinates
    * @param center optional center point (angle goes from a - center - b)
    * @returns angle between points
    */
    static double angleBetweenPoints(cv::Point a, cv::Point b, cv::Point center = cv::Point(0, 0));

    /**
     * Compute the magnitude of a 2D point.
     * @param vec input point
     * @returns magnitude of point
     */
    static double magnitude(cv::Point2f pt);

    /**
     * Compute the magnitude of a 2D point.
     * @param vec input point
     * @returns magnitude of point
     */
    static double magnitude(cv::Point pt);

    /**
     * Compute the magnitude of a 3D vector.
     * @param vec input vector
     * @returns magnitude of vector
     */
    static double magnitude(cv::Vec3f vec);

    /**
     * Compute the angle between two 3D vectors.
     * @param a vector 1
     * @param b vector 2
     * @param center optionally, vector to subtract both a and b by before computing angle
     * @returns angle between vectors
     */
    static double angleBetween3DVec(cv::Vec3f a, cv::Vec3f b, cv::Vec3f center = cv::Vec3f(0, 0, 0));


    /**
     * Checks if a point is within the bounds of an image.
     * @param [in] img the image
     * @pt the point
     */
    static bool pointInImage(const cv::Mat & img, const cv::Point pt, int scale = 1);

    /**
     * Computes the area of the triangle defined by three vertices
     * @param a first vertex
     * @param b second vertex
     * @param c third vertex
     * @returns area of triangle, in real meters squared
     */
    static double Util::triangleArea(cv::Vec3f a, cv::Vec3f b, cv::Vec3f c = cv::Vec3f(0, 0, 0));

    /**
     * Computes the area of the quadrangle defined by four vertices
     * @param pts the vertices of the quadrangle
     * @returns area of quadrangle, in real meters squared
     */
    static double quadrangleArea(cv::Vec3f pts[4]);

    /**
     * Computes the approximate surface area of all visible clusters on a depth map.
     * @param [in] depthMap the input depth map. All points with 0 z-coordinate will be excluded.
     * @returns surface area, in meters squared
     */
    static double surfaceArea(cv::Mat & depthMap);

    /**
     * Computes the approximate surface area of a cluster on a depth map, specified by a vector of points.
     * @param [in] depthMap the input depth map.
     * @param [in] cluster vector of points on the cluster. 
     * @param [in] clusterSize number of points in this cluster. By default, uses all points in the 'cluster' vector.
     * @returns surface area, in meters squared
     */
    static double surfaceArea(cv::Mat & depthMap, std::vector<cv::Point> & cluster, int clusterSize = -1);

    /**
     * Compares two points (Point, Point2f, Vec3i or Vec3f),
     * first by x, then y, then z (if available). Used for sorting points.
     */
    template<class T>
    class PointComparer{
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

private:
    static double distanceTwoPoints(double x1, double y1, double x2, double y2);
    static double otherAngleFind(double biggerAngle, double largestDistance, double smallDistance);
    static double biggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo);
    static bool closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance);
};