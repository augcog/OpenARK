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

	cv::namedWindow("hand", cv::WINDOW_AUTOSIZE);
	cv::imshow("hand", cluster);
	// Step 1: determine whether cluster is hand

	//if (checkForHand(cluster, 0.005, 0.25)) { //original
	if (checkForHand(cluster, 0.005, 0.4)) {
		hand = Hand(cluster, 50);

		hasHand = true;
		return;
	}

    // TEMPORARILY disabling plane detection
    return;


	// Step 2: determine whether there is a plane
	plane = new Plane(cluster);

	// Step 2.1 If there is plane, remove plane and look for hand
	auto points = plane->getPlaneIndicies();

	if (points.size() != 0)
	{
		hasPlane = true;

		for (auto i = 0; i < points.size(); i++)
		{
			auto x = points[i].x;
			auto y = points[i].y;
			cluster.at<cv::Vec3f>(y, x)[0] = 0;
			cluster.at<cv::Vec3f>(y, x)[1] = 0;
			cluster.at<cv::Vec3f>(y, x)[2] = 0;
		}

		auto center = Util::findCentroid(cluster);
		cv::Mat hand_cluster = cv::Mat::zeros(cluster.rows, cluster.cols, cluster.type());

		//determining the pixels that are similar to center and connected to it
		Util::floodFill(center.x, center.y, cluster, hand_cluster, 0.02);

		if (checkForHand(hand_cluster, -0.99, 0.2))
		{
			hand = Hand(hand_cluster, 30);
			auto finger_length = Util::euclidianDistance3D(hand.fingers_xyz[0], hand.centroid_xyz);
			if (finger_length > 0.03 && finger_length < 0.2)
			{
				hasHand = true;
				return;
			}
		}
	}

	// Step 2.1.1 If there is plane, no hand, then the rest of points are shape
	shape = cluster;
	hasShape = true;
}

Hand Object3D::getHand()
{
	return hand;
}

Plane Object3D::getPlane()
{
	return *plane;
}

cv::Mat Object3D::getShape()
{
	return shape;
}

//talk to Joe--some of the code that was there was not working!
double Object3D::centroidCircleSweep(cv::Mat cluster, double distance) const
{
	cv::Mat channels[3];
	cv::split(cluster, channels);

	// Step 1: Find the centroid
	auto m = cv::moments(channels[2], false);
	cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

	// Step 2: Find the radius (pixels) that correspond to distance (meters)
	auto distancePerPixel = Util::euclideanDistancePerPixel(cluster, center, 5);
	float radius = distance / distancePerPixel;

	if (radius <= 0 || radius > cluster.cols / 4)
	{
		return -1;
	}

	// Step 3: Extract all pixels within distance
	cv::Mat binary_img;
	cv::normalize(channels[2], binary_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::Mat mask = cv::Mat::zeros(binary_img.size(), binary_img.type());
	cv::Mat dstImg = cv::Mat::zeros(binary_img.size(), binary_img.type());
	cv::circle(mask, center, radius, cv::Scalar(255, 255, 255));
	binary_img.copyTo(dstImg, mask);
	auto covered = 0;
	//cv::namedWindow("mask", cv::WINDOW_AUTOSIZE);
	//cv::imshow("mask", mask);

	for (auto r = center.y; r > 0; r--) {
		for (auto c = 0; c < dstImg.cols; c++) {
			if (dstImg.at<uchar>(r, c) != 0) {
				covered++;
			}
		}
	}
	//returns 1 for covered and 1 for countNonZero(mask)
	auto coverage = static_cast<double>(covered) / cv::countNonZero(mask);
	return coverage;
}

bool Object3D::checkForHand(cv::Mat cluster, double min_coverage, double max_coverage, double pointer_finger_distance)
{
	checkEdgeConnected(cluster);
	//if ((rightEdgeConnected && !leftEdgeConnected) || (leftEdgeConnected && rightEdgeConnected)) { //original
		auto coverage = centroidCircleSweep(cluster, pointer_finger_distance);
		if (coverage < max_coverage && coverage > min_coverage)
		{
			return true;
		}

	return false;
}


void Object3D::checkEdgeConnected(cv::Mat cluster)
{
	auto cols = cluster.cols;
	auto rows = cluster.rows;

	// bottom Sweep
	auto r1 = static_cast<int>(rows * 0.9);
	for (auto c1 = 0; c1 < cols / 4; c1++)
	{
		if (cluster.at<cv::Vec3f>(r1, c1)[2] != 0)
		{
			leftEdgeConnected = true;
			break;
		}
	}

	// Left Side Sweep
	auto c2 = static_cast<int>(cols * 0.2);

	for (auto r2 = 0; r2 < rows; r2++)
	{
		if (cluster.at<cv::Vec3f>(r2, c2)[2] != 0)
		{
			leftEdgeConnected = true;
			break;
		}
	}

	// Bottom Sweep
	auto r3 = static_cast<int>(rows * 0.9);
	//for (auto c3 = cols / 4; c3 < cols; c3++) { //original
	for (auto c3 = cols - (cols / 4); c3 < cols; c3++)
	{ //fixed
		if (cluster.at<cv::Vec3f>(r3, c3)[2] != 0)
		{
			rightEdgeConnected = true;
			break;
		}
	}

	// Right Side Sweep
	auto c4 = static_cast<int>(cols * 0.8);
	for (auto r4 = rows / 2; r4 < rows; r4++)
	{
		if (cluster.at<cv::Vec3f>(r4, c4)[2] != 0)
		{
			rightEdgeConnected = true;
			break;
		}
	}

}



Object3D::~Object3D()
{

}

Hand Object3D::getHand() const
{
	return {};
}

Plane Object3D::getPlane() const
{
	return Plane();
}

cv::Mat Object3D::getShape() const
{
	return {};
}
