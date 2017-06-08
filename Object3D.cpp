#include "Object3D.h"
#include "Visualizer.h"
#include "Util.h"

Object3D::Object3D()
{

}

Object3D::Object3D(cv::Mat cluster) {
	// Step 1: Initialize variables
	rightEdgeConnected = false;
	leftEdgeConnected = false;
	hasHand = false;
	hasPlane = false;
	hasShape = false;

	// Step 1: determine whether cluster is hand

	if (checkForHand(cluster, 0.005, 0.25)) {
		hand = Hand(cluster, 50);
		hasHand = true;
		return;
	}


	// Step 2: determine whether there is a plane

	plane = new Plane(cluster);
	// Step 2.1 If there is plane, remove plane and look for hand
	std::vector<cv::Point2i> points = plane->getPlaneIndicies();
	if (points.size() != 0) {
		hasPlane = true;
		for (int i = 0; i < points.size();i++) {
			int x = points[i].x;
			int y = points[i].y;
			cluster.at<cv::Vec3f>(y, x)[0] = 0;
			cluster.at<cv::Vec3f>(y, x)[1] = 0;
			cluster.at<cv::Vec3f>(y, x)[2] = 0;
		}

		cv::Point center = Util::findCentroid(cluster);

		cv::Mat hand_cluster = cv::Mat::zeros(cluster.rows, cluster.cols, cluster.type());
		Util::floodFill(center.x, center.y, cluster, hand_cluster, 0.02);

		if (checkForHand(hand_cluster, -0.99, 0.2)) {
			hand = Hand(hand_cluster, 30);
			double finger_length = Util::euclidianDistance3D(hand.fingers_xyz[0], hand.centroid_xyz);
			if (finger_length > 0.03 && finger_length < 0.2) {
				hasHand = true;
				return;
			}
		}
	}


	// Step 2.1.1 If there is plane, no hand, then the rest of points are shape
	shape = cluster;
	hasShape = true;
}

Hand Object3D::getHand() {
	return hand;
}

Plane Object3D::getPlane() {
	return *plane;
}

cv::Mat Object3D::getShape() {
	return shape;
}

double Object3D::centroidCircleSweep(cv::Mat cluster, double distance)
{
	cv::Mat channels[3];
	cv::split(cluster, channels);
	cv::Mat show_img = Visualizer::visualizeXYZMap(cluster);

	// Step 1: Find the center
	cv::Moments m = cv::moments(channels[2], false);
	cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
	cv::circle(show_img, center, 2, cv::Scalar(255, 0, 0),2);

	// Step 2: Find the radius (pixels) that correspond to distance (meters)
	double distancePerPixel = Util::euclideanDistancePerPixel(cluster, center, 5);
	int radius = distance / distancePerPixel;
	if (radius <= 0 || radius > cluster.cols / 4) {
		return -1;
	}

	// Step 3: Extract all pixels within distance
	cv::Mat binary_img;
	cv::normalize(channels[2], binary_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::Mat mask = cv::Mat::zeros(binary_img.size(), binary_img.type());
	cv::Mat dstImg = cv::Mat::zeros(binary_img.size(), binary_img.type());
	cv::circle(mask, center, radius, cv::Scalar(255, 255, 255));
	binary_img.copyTo(dstImg, mask);


	int covered = 0;
	for (int r = center.y; r > 0; r--) {
		for (int c = 0; c < dstImg.cols; c++) {
			if (dstImg.at<uchar>(r, c) != 0) {
				covered++;
			}
		}
	}

	double coverage = (double) covered / cv::countNonZero(mask);
	return coverage;
}

bool Object3D::checkForHand(cv::Mat cluster, double min_coverage, double max_coverage, double pointer_finger_distance)
{
	checkEdgeConnected(cluster);
	if ((rightEdgeConnected && !leftEdgeConnected) || (leftEdgeConnected && rightEdgeConnected)) {
		double coverage = centroidCircleSweep(cluster, pointer_finger_distance);
		if (coverage < max_coverage && coverage > min_coverage) {
			return true;
		}
	}

	return false;
}

void Object3D::checkEdgeConnected(cv::Mat cluster) {
	int cols = cluster.cols;
	int rows = cluster.rows;

	// Bottom Sweep
	int r = (int)(rows * 0.9);
	for (int c = 0; c < cols / 4 ; c++) {
		if (cluster.at<cv::Vec3f>(r, c)[2] != 0) {
			leftEdgeConnected = true;
			break;
		}
	}

	// Left Side Sweep
	int c = (int)(cols * 0.2);
	for (int r = 0; r < rows; r++) {
		if (cluster.at<cv::Vec3f>(r, c)[2] != 0) {
			leftEdgeConnected = true;
			break;
		}
	}

	// Bottom Sweep
	r = (int)(rows * 0.9);
	for (int c = cols / 4; c < cols; c++) {
		if (cluster.at<cv::Vec3f>(r, c)[2] != 0) {
			rightEdgeConnected = true;
			break;
		}
	}

	// Right Side Sweep
	c = (int)(cols * 0.8);
	for (int r = rows / 2; r < rows; r++) {
		if (cluster.at<cv::Vec3f>(r, c)[2] != 0) {
			rightEdgeConnected = true;
			break;
		}
	}

}


Object3D::~Object3D()
{

}