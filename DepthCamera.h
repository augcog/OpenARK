#pragma once

// C++ Libraries
#include <iostream>

// OpenCV Libraries
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

// OpenARK Libraries
#include "Util.h"

/**
 * Class defining general behavior of a depth camera.
 * Any depth camera should be able to generate a XYXMap, AmpMap (confidence), and FlagMap.
 */
class DepthCamera
{
public:
	/**
	 * Update the depth camera by one frame.
	 * This function should be overriden with a concrete implementation depending on the specific depth camera
	 */
	virtual void update() = 0;

	/**
	 * Closes and exists the depth camera.
	 * This function should be overriden with a concrete implementation depending on the specific depth camera
	 */
	virtual void destroyInstance() = 0;
	
	/**
	 * Performs euclidean clustering to separate discrete objects in the input point cloud.
	 * @param max_distance the maximum allowed distance to be clustered
	 * @param min_size the minimum number of points a valid cluster should have
	 */
	void computeClusters(double max_distance, double min_size);

	/**
	 * Reads a sample frame from file.
	 * @param source the directory which the frame file is stored
	 */
	bool readImage(std::string source);

	/**
	 * Writes the current frame into file.
	 * @param destination the directory which the frame should be written to
	 */
	bool writeImage(std::string destination);

	/**
	 * Removes noise from the XYZMap based on confidence provided in the AmpMap and FlagMap.
	 */
	void removeNoise();

	/**
	 * Removes all points defined by the coordinates in the points vector from the XYZMap.
	 * @param [in] points the list of coordinates where data from the XYZMap should be removed
	 */
	void removePoints(std::vector<cv::Point2i> points);

	/**
	 * Return the current XYZMap.
	 */
	cv::Mat getXYZMap();

	/**
	 * Return the current AmpMap
	 */
	cv::Mat getAmpMap();

	/**
	 * Return the current FlagMap.
	 */
	cv::Mat getFlagMap();

	/**
	 * Returns the width of the frame in pixels.
	 */
	int getWidth();

	/**
	 * Returns the height of the frame in pixels.
	 */
	int getHeight();

	/**
	 * Returns all the clusters (discrete objects) in the current frame.
	 * @see computeClusters
	 * @return vector of matrixes with each matrix corresponding to a cluster in no particular order
	 */
	std::vector<cv::Mat> getClusters();

	bool badInput;
	
	
protected:
	/**
	 * Initializes all variables used by the generic depth camera.
	 * Sets xyzMap, ampMap, flagMap, and clusters to empty
	 */
	void initilizeImages();

	/**
	 * Performs floodfill starting from seed point (x,y).
	 * @param x x-coordinate of the seed point
	 * @param y y-coordinate of the seed point
	 * @param [in] zMap the xyzMap point cloud
	 * @param [out] mask the resulting region of the floodfill
	 * @param max_distance the maximum euclidean distance allowed between neighbors
	 */
	void floodFill(int x, int y, cv::Mat& zMap, cv::Mat& mask, double max_distance);

	/**
	 * Analyze the candidate point with its neighbors to determine whether they belong to the same cluster.
	 * @param x x-coordinate of the candidate point
	 * @param y y-coordinate of the candidate point
	 * @param [in] depthMap the xyzMap point cloud of the scene
	 * @param num_neighbors the number of neighbors to consider
	 * @param max_distance the maximum euclidean distance allowed between neighbors
	 */
	bool closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance);

	/**
	 * Stores the (x,y,z) data of every point in the observable world.
	 * Matrix type CV_32FC3
	 */
	cv::Mat xyzMap;

	/**
	 * Stores the confidence value of each corresponding point in the world.
	 * Matrix type CV_32FC1
	 */
	cv::Mat ampMap;

	/**
	 * Stores additional information about the points in the world.
	 * Matrix type CV_8UC1
	 */
	cv::Mat flagMap;
	
	/**
	 * Stores the each individual cluster in its individual XYZMap.
	 */
	std::vector<cv::Mat> clusters;

	/**
	 * Value that determines the validity of a point in respect to the ampMap.
	 * This value varies from sensor to sensor so it should be define when constructing child class
	 */
	double CONFIDENCE_THRESHHOLD;

	/**
	 * Value that determines the validity of a point in respect to the flagMap.
	 * This value varies from sensor to sensor so it should be defined when constructing child class
	 */
	int INVALID_FLAG_VALUE;

	/**
	 * The image width resolution (pixels) that the depth sensor produces.
	 */
	int X_DIMENSION = 176;

	/**
	 * The image height resolution (pixels) that the depth sensor produces.
	 */
	int Y_DIMENSION = 120;
};