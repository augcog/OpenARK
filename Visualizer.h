#pragma once

// OpenARK headers
#include "Hand.h"

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
    * @param [in] hand the hand object
    * @return a CV_8UC3 matrix with the hand drawn on it
    */
    static cv::Mat visualizeHand(cv::Mat xyzMap, const Hand hand);

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

    /**
    * Visualize points that lie on the plane.
    * @param input_mat the input point cloud
    * @param indicies (i,j) coordinates of the points belonging to the plane
    */
    static void visualizePlanePoints(cv::Mat &input_mat, std::vector<cv::Point2i> indicies);

    /**
    * Visualize a depth map
    * @param depthMap the depth map
    * @returns visualization
    */
    static cv::Mat visualizeDepthMap(cv::Mat &depthMap);

    /**
     * Initializes & opens the PCL visualizer
     * @returns true on success, false if visualizer already open
     */
    static bool initPCLViewer();
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
    static pcl::visualization::PCLVisualizer * viewer;

};