#pragma once
// OpenCV Libraries
#include "opencv2/imgproc/imgproc.hpp"

// Constants Redefined
#ifndef M_PI
	#define M_PI 3.14159265358979323846
	#define PI 3.14159265;
#endif

/**
* Class containing generic helper functions.
*/
class Util
{
public:

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
	static double euclidianDistance3D(cv::Vec3f pt1, cv::Vec3f pt2);

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

	/*
	* Find the centroid of the point cloud
	* @param xyzMap input point cloud
	* @return (x,y) coordinate of the centroid
	*/
	static cv::Point findCentroid(cv::Mat xyzMap);

	/*
	* Compute the angle formed by 3 points
	* @return the angle formed
	*/
	static double TriangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3);

	static void floodFill(int x, int y, cv::Mat& depthMap, cv::Mat& mask, double max_distance);


private:
	static double DistanceTwoPoints(double x1, double y1, double x2, double y2);
	static double otherAngleFind(double biggerAngle, double largestDistance, double smallDistance);
	static double BiggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo);
	static bool Util::closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance);

};