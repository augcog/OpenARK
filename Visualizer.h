#pragma once
//OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// PCL Libraries
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/cloud_viewer.h>

/**
* Utility class containing various conversions and visualization techniques.
*/
class Visualizer
{
public:
	/**
	* Visualization for xyzMap.
	* @param [in] xyzMap input point cloud matrix
	* @return a CV_8UC3 representation of the xyzMap
	*/
	static cv::Mat visualizeXYZMap(cv::Mat &xyzMap);

	/**
	* Visualization for hand object.
	* @param [in] xyzMap the base image to draw on
	* @param [in] finger (i,j) coordinates of the finger
	* @param [in] centroid (i,j) coordinates of the centroid
	* @return a CV_8UC3 matrix with the hand points drawn
	*/
	static cv::Mat visualizeHand(cv::Mat xyzMap, cv::Point2i finger, cv::Point2i centroid);

	/**
	* Visualization for PCL point cloud.
	* @param [in] cloud PCL point cloud to be visualized
	*/
	static void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/**
	* Visualization for polygon mesh.
	* Visualize a PCL point cloud as a polygon mesh
	* @param [in] cloud PCL point cloud to be visualized
	*/
	static void visulizePolygonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/**
	* Visualization for plane regression.
	* @param [in] input_mat the base xyzMap on which to draw the visualization
	* @param [in] equation equation of the plane
	* @param threshold maximum error distance (mm) allowed for points to be considered covered by regression equation
	* @param clicked whether the finger is currently current contacting the regression equation. Default is FALSE
	* @return a CV_8UC3 representation of the matrix with the regression plane drawn
	*/
	static cv::Mat visualizePlaneRegression(cv::Mat &input_mat, std::vector<double> &equation, const double threshold, bool clicked = false);

	/*
	* Visualize points that lie on the plane.
	* @param input_mat the input point cloud
	* @indicies (i,j) coordinates of the points belonging to the plane
	*/
	static void visualizePlanePoints(cv::Mat &input_mat, std::vector<cv::Point2i> indicies);
	static cv::Mat visualizeDepthMap(cv::Mat &depthMap);

private:

	/**
	* Visualization for a generic matrix.
	* @param [in] input matrix to be visualized
	* @return a CV_8UC3 representation of the input matrix
	*/
	static cv::Mat visualizeMatrix(cv::Mat &input);

	/**
	* Visualization for a depth map matrix (i,j,z).
	* @param [in] depthMap matrix to be visualized
	* @return a CV_8UC3 representation of the input matrix
	*/


	/**
	* PCL point cloud viewer
	*/
	static pcl::visualization::PCLVisualizer viewer;
};