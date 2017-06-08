#pragma once
// C++ Libraries
#include <iostream>
#include <vector>
#include <cmath>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

class Hand {
public:


	// Public constructors
	/**
	* Default constructor for a hand object
	*/
	Hand();

	/**
	* Constructs a hand object based on point cloud and constraints.
	* @param xyzMap Input point cloud containing only the hand (nothing else can be in this point cluod)
	* @param angle_treshhold Sharpest allowable angle formed by finger tip and neighboring defects
	* @param cluster_thresh Maximum allowable distance between two finger tips
	*/
	Hand(cv::Mat xyzMap, float angle_treshhold, int cluster_thresh = 30);

	/**
	* Deconstructor for the hand object
	*/
	~Hand();

	// Public variables

	/**
	* Maximum allowable distance between adjacent finger tips
	*/
	int CLUSTER_THRESHOLD;

	/**
	* (x,y,z) position of all detected finger tips
	*/
	std::vector<cv::Vec3f> fingers_xyz;

	/**
	* (i,j) coordinates of all detected finger tips
	*/
	std::vector<cv::Point2i> fingers_ij;

	/**
	* (x,y,z) position of hand centroid
	*/
	cv::Vec3f centroid_xyz;

	/**
	* (i,j) coordinates of hand centroid
	*/
	cv::Point2i centroid_ij;

	/**
	* (x,y,z) position of detected defects
	*/
	std::vector<cv::Vec3f> defects_xyz;

	/**
	* (i,j) position of detected defects
	*/
	std::vector<cv::Point2i> defects_ij;

	// Public functions

	/**
	* Determine whether one of the fingers is in contact with a tracked object.
	* @param equation Regression equation of the object
	* @param threshold Thickness of the plane modeled by the regression equation
	* @return TRUE if hand is contacting a tracked object. FALSE otherwise
	*/
	bool touchObject(std::vector<double> &equation, const double threshold);

private:
	// Private constructors
	/**
	* Find the contour with max number of vertices.
	**/
	std::vector<cv::Point> findComplexContour(std::vector< std::vector<cv::Point> > contours);

	/**
	* Groups vertices within a threshold and selects the median of each group.
	**/
	std::vector<cv::Point> clusterConvexHull(std::vector<cv::Point> convexHull, int threshold);

	/**
	* Find the centroid of contour using 0th and 1st degree moments
	**/
	cv::Point findCenter(std::vector<cv::Point> contour);

	// Private variables
	float ANGLE_THRESHHOLD;

	// Private functions
	void analyzeHand(cv::Mat xyzMap);
};
