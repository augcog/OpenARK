#include "Visualizer.h"

pcl::visualization::PCLVisualizer Visualizer::viewer = pcl::visualization::PCLVisualizer("Point Cloud");

/***
Maps matrix values to [0, 255] for viewing
***/
cv::Mat Visualizer::visualizeMatrix(cv::Mat &input)
{
	cv::Mat img;
	cv::normalize(input, img, 0, 255, cv::NORM_MINMAX, CV_8UC3);
	return img;
}

/***
RGB depth map visualization
***/
cv::Mat Visualizer::visualizeDepthMap(cv::Mat &depthMap)
{
	cv::Mat img;
	cv::normalize(depthMap, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::applyColorMap(img, img, cv::COLORMAP_HOT);
	return img;
}

cv::Mat Visualizer::visualizeXYZMap(cv::Mat &xyzMap)
{
	cv::Mat channels[3];
	cv::split(xyzMap, channels);
	return visualizeDepthMap(channels[2]);
}

cv::Mat Visualizer::visualizeHand(cv::Mat xyzMap, cv::Point2i finger, cv::Point2i centroid)
{
	cv::Mat displayImg;
	if (xyzMap.type() == CV_32FC3) {
		displayImg = Visualizer::visualizeXYZMap(xyzMap);
	}
	else {
		displayImg = xyzMap;
	}

	cv::circle(displayImg, cv::Point(finger.x, finger.y), 2, cv::Scalar(0, 255, 255), 2);
	cv::circle(displayImg, cv::Point(centroid.x, centroid.y), 2, cv::Scalar(0, 255, 0), 2);
	return displayImg;
}

void Visualizer::visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	viewer.setBackgroundColor(0, 0, 0);
	if (!viewer.updatePointCloud(cloud))
		viewer.addPointCloud(cloud);
	viewer.spinOnce();
}

cv::Mat Visualizer::visualizePlaneRegression(cv::Mat &input_mat, std::vector<double> &equation, const double threshold, bool clicked)
{
	cv::Mat output_mat;
	if (input_mat.type() == CV_32FC3) {
		output_mat = Visualizer::visualizeXYZMap(input_mat);
	}
	else {
		output_mat = input_mat;
	}

	if (equation.size() < 3)
	{
		return output_mat;
	}
	int rowSize = input_mat.rows;
	int colSize = input_mat.cols;
	cv::Scalar color;
	if (clicked) {
		color = cv::Scalar(255, 255, 0);
	}
	else {
		color = cv::Scalar(0, 255, 0);
	}

	int pointsDetected = 0;
	for (int r = 0; r < rowSize; r++) {
		for (int c = 0; c < colSize; c++) {
			double x = input_mat.at<cv::Vec3f>(r, c)[0];
			double y = input_mat.at<cv::Vec3f>(r, c)[1];
			double z = input_mat.at<cv::Vec3f>(r, c)[2];

			if (z == 0) {
				continue;
			}

			double z_hat = equation[0] * x + equation[1] * y + equation[2];
			double r_squared = (z - z_hat) * (z - z_hat);

			if (r_squared < threshold) {
				cv::circle(output_mat, cv::Point(c, r), 1, color, -1);
				pointsDetected++;
			}
		}
	}
	return output_mat;
}

void Visualizer::visualizePlanePoints(cv::Mat &input_mat, std::vector<cv::Point2i> indicies)
{
	for (int i = 0; i < indicies.size(); i++) {
		input_mat.at<uchar>(indicies[i].y, indicies[i].x) = (uchar)255;
	}
}

void Visualizer::visulizePolygonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (cloud.get()->width == 0) {
		return;
	}
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	viewer.setBackgroundColor(0, 0, 0);
	if (!viewer.updatePolygonMesh(triangles))
		viewer.addPolygonMesh(triangles);
	viewer.spinOnce();
}