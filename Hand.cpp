#include "Hand.h"
#include "Util.h"
#include "Visualizer.h"

Hand::Hand()
{

}

Hand::Hand(cv::Mat xyzMap, float angle_threshhold, int cluster_thresh)
{
	CLUSTER_THRESHOLD = cluster_thresh;
	ANGLE_THRESHHOLD = angle_threshhold;
	analyzeHand(xyzMap);
}

Hand::~Hand()
{

}

void Hand::analyzeHand(cv::Mat xyzMap)
{
	
	cv::Mat normalizedDepthMap;
	cv::Mat channel[3];
	cv::split(xyzMap, channel);
	cv::normalize(channel[2], normalizedDepthMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	// Resize input
	cv::Mat input;
	cv::pyrUp(normalizedDepthMap, input, cv::Size(normalizedDepthMap.cols * 2, normalizedDepthMap.rows * 2));
	cv::pyrUp(input, input, cv::Size(input.cols * 2, input.rows * 2));

	cv::Mat threshold_output;
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// Find contours
	
	cv::threshold(input, threshold_output, 100, 255, cv::THRESH_BINARY);
	cv::findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// Find contour polygon
	
	std::vector< std::vector< cv::Point> > contours_poly(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
	}

	// Find largest contour
	std::vector<cv::Point> contour = Hand::findComplexContour(contours);

	// Find approximated convex hull
	
	std::vector<cv::Point> hull;
	std::vector<cv::Point> completeHull;
	std::vector<int> indexHull;
	if (contour.size() > 1) {
		cv::convexHull(contour, completeHull, 0, 1);
		cv::convexHull(contour, indexHull, 0, 0);
		hull = Hand::clusterConvexHull(completeHull, Hand::CLUSTER_THRESHOLD);
	}

	// Find convexityDefects
	
	std::vector<cv::Vec4i> defects;
	if (indexHull.size() > 3) {
		cv::convexityDefects(contour, indexHull, defects);
	}

	// Find max and min distances
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(channel[2], &minVal, &maxVal, &minLoc, &maxLoc);

	// Find center of contour
	
	cv::Point center = Hand::findCenter(contour);
	centroid_xyz = xyzMap.at<cv::Vec3f>(center.y / 4, center.x / 4); 
	centroid_ij = cv::Point2i(center.x, center.y); // SCALING

	// Generate visual
	cv::Mat img = cv::Mat::zeros(input.rows, input.cols, CV_8UC3);
	cv::Scalar color = cv::Scalar(0, 255, 0);

	// Draw contours
	cv::circle(img, center, 5, cv::Scalar(255, 0, 0), 2);
	
	for (int i = 0; i < contours.size(); i++) {
		cv::drawContours(img, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	}
	
	// Draw hull
	
	cv::Point index;
	cv::Point index_right;
	cv::Point index_left;
	double farthest = 0;

	if (hull.size() > 1) {
		for (int i = 0; i < hull.size(); i++) {
			cv::Point p1 = hull[i];
			cv::Point p2 = hull[(i + 1) % hull.size()];
			//cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 1);

			if (p1.y < centroid_ij.y && Util::euclideanDistance2D(p1, centroid_ij) > farthest) {
				farthest = Util::euclideanDistance2D(p1, centroid_ij);
				index = p1;
				index_right = hull[(i + 1) % hull.size()];
				index_left = hull[(i - 1) % hull.size()];
			}
		}
	}
	
	// Draw defects (filter)
	
	std::vector<cv::Point> endpoints;
	std::vector<cv::Point> fingerDefects;
	cv::Point lastStart;
	int found = -1;
	for (int i = 0; i < defects.size(); i++) {
		cv::Vec4i defect = defects[i];
		cv::Point start = contour[defect[0]];
		cv::Point end = contour[defect[1]];
		cv::Point farPt = contour[defect[2]];
		// Depth from edge of contour
		// std::cout << "Depth: " << depth << "\tThreshold: " << cv::norm(maxLoc - center) << "\t";
		// Defect conditions: depth is sufficient, inside contour, y value is above center
		int depth = defect[3]; 
		
		// maxLoc largest depth
		// first condition replace with meters distance from the edge
		// second test if inside the hull (no change)
		// above the center (no change)
		if (cv::norm(maxLoc - center) * 15 < depth && cv::pointPolygonTest(hull, farPt, false) > 0 && farPt.y < center.y) {
			cv::Vec3f pt1 = xyzMap.at<cv::Vec3f>(farPt.y / 4, farPt.x / 4);
			if (Util::euclidianDistance3D(pt1, centroid_xyz) > 0.05) {
				endpoints.push_back(start);
				endpoints.push_back(end);
				fingerDefects.push_back(farPt);
			}
		}
	}
	
	// Cluster fingertip locations
	
	endpoints = Hand::clusterConvexHull(endpoints, Hand::CLUSTER_THRESHOLD);
	for (int i = 0; i < endpoints.size(); i++) {
		cv::Point endpoint = endpoints[i];
		
		cv::Point closestDefect;
		int minDefectDistance = 1 << 29;
		for (int i = 0; i < fingerDefects.size(); i++) {
			if (cv::norm(endpoint - fingerDefects[i]) < minDefectDistance) {
				minDefectDistance = cv::norm(endpoint - fingerDefects[i]);
				closestDefect = fingerDefects[i];
			}
		}
		cv::Vec3f endPoint_xyz = Util::averageAroundPoint(xyzMap, cv::Point2i(endpoint.x / 4, endpoint.y / 4), 10);
		cv::Vec3f closestDefect_xyz = Util::averageAroundPoint(xyzMap, cv::Point2i(closestDefect.x / 4,  closestDefect.y / 4), 10);
		double finger_length = Util::euclidianDistance3D(endPoint_xyz, closestDefect_xyz);
		if (finger_length < 0.08 && finger_length > 0.025 && endpoint.y < closestDefect.y) {
			fingers_xyz.push_back(endPoint_xyz);
			fingers_ij.push_back(cv::Point2i(endpoint.x, endpoint.y)); // SCALING

			defects_xyz.push_back(Util::averageAroundPoint(xyzMap, cv::Point2i(closestDefect.x / 4, closestDefect.y / 4), 5));
			defects_ij.push_back(cv::Point2i(closestDefect.x, closestDefect.y)); // SCALING
		}
	}
	if ((float)cv::countNonZero(channel[2]) / (xyzMap.rows*xyzMap.cols) > 0.3) {
		return;
	}

	// If there is one or less visible fingers
	if (fingers_xyz.size() <= 1)
	{
		fingers_xyz.clear();
		fingers_ij.clear();

		cv::Vec3f indexFinger = Util::averageAroundPoint(xyzMap, cv::Point2i(index.x / 4, index.y / 4), 10);
		fingers_xyz.push_back(indexFinger);
		fingers_ij.push_back(cv::Point2i(index.x, index.y)); // SCALING
		
		double angle = Util::TriangleAngleCalculation(index_left.x, index_left.y, index.x, index.y, index_right.x, index_right.y);
		if (defects_ij.size() != 0) {
			for (int i = 0; i < fingers_xyz.size(); i++) {
				cv::circle(img, fingers_ij[i], 5, cv::Scalar(0, 0, 255), 3);
				cv::line(img, defects_ij[i], fingers_ij[i], cv::Scalar(255, 0, 255), 2);
				cv::circle(img, defects_ij[i], 5, cv::Scalar(0, 255, 255), 2);
				cv::line(img, defects_ij[i], centroid_ij, cv::Scalar(255, 0, 255), 2);
			}
		}
		else if (angle > ANGLE_THRESHHOLD) {
			cv::circle(img, fingers_ij[0], 5, cv::Scalar(0, 0, 255), 3);
			cv::line(img, fingers_ij[0], centroid_ij, cv::Scalar(255, 0, 255), 2);
		}
		
	}
	else {
		for (int i = 0; i < fingers_xyz.size(); i++) {
			cv::circle(img, fingers_ij[i], 5, cv::Scalar(0, 0, 255), 3);
			cv::line(img, defects_ij[i], fingers_ij[i], cv::Scalar(255, 0, 255), 2);
			cv::circle(img, defects_ij[i], 3, cv::Scalar(0, 255, 255), 2);
			cv::line(img, defects_ij[i], centroid_ij, cv::Scalar(255, 0, 255), 2);
		}
	}
	cv::imshow("Contours", img);
	
	
}

std::vector<cv::Point> Hand::findComplexContour(std::vector< std::vector<cv::Point> > contours) {
	std::vector<cv::Point> contour;
	int maxPoints = 0;
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > maxPoints) {
			maxPoints = contours[i].size();
			contour = contours[i];
		}
	}
	return contour;
}

std::vector<cv::Point> Hand::clusterConvexHull(std::vector<cv::Point> convexHull, int threshold) {
	std::vector<cv::Point> clusterHull;
	int i = 0;
	while (i < convexHull.size()) {
		// Select a point from cluster
		std::vector<cv::Point> cluster;
		cv::Point hullPoint = convexHull[i];
		cluster.push_back(hullPoint);
		i++;
		while (i < convexHull.size()) {
			cv::Point clusterPoint = convexHull[i];
			double distance = cv::norm(hullPoint - clusterPoint);
			if (distance < threshold) {
				cluster.push_back(clusterPoint);
				i++;
			}
			else {
				break;
			}
		}
		hullPoint = cluster[cluster.size() / 2];
		cv::Point center = findCenter(convexHull);
		int maxDist = cv::norm(hullPoint - center);
		for (int i = 0; i < cluster.size(); i++) {
			if (cv::norm(cluster[i] - center) > maxDist) {
				maxDist = cv::norm(cluster[i] - center);
				hullPoint = cluster[i];
			}
		}
		clusterHull.push_back(hullPoint);
	}
	return clusterHull;
}

cv::Point Hand::findCenter(std::vector<cv::Point> contour) {
	cv::Point center;
	cv::Moments M = cv::moments(contour, false);
	center = cv::Point((int)M.m10 / M.m00, (int)M.m01 / M.m00);
	return center;
}

bool Hand::touchObject(std::vector<double> &equation, const double threshold)
{
	if (equation.size() == 0) {
		return false;
	}

	for (int i = 0; i < fingers_xyz.size(); i++) {
		double x = fingers_xyz[i][0];
		double y = fingers_xyz[i][1];
		double z = fingers_xyz[i][2];
		if (z == 0) {
			return false;
		}

		double z_hat = equation[0] * x + equation[1] * y + equation[2];
		double r_squared = (z - z_hat) * (z - z_hat);

		if (r_squared < threshold) {
			return true;
		}
	}

	return false;
}