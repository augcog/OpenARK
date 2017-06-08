#include "Util.h"

cv::Vec3b Util::colorGenerator2()
{
	return cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
}

float Util::normalize(float a, float b)
{
	return sqrt(a*a + b*b);
}

bool Util::isMember(cv::Mat image, int x, int y)
{
	if (x < 0 || y < 0 || x >= image.cols || y >= image.rows) {
		return false;
	}

	if (image.at<uchar>(y, x) != 0) {
		return true;
	}

	return false;
}
int Util::getDistanceT(int x1, int y1, int x2, int y2)
{
	return abs(y1 - y2) + abs(x2 - x1);
}

double Util::euclidianDistance3D(cv::Vec3f pt1, cv::Vec3f pt2)
{
	double dx = pt1[0] - pt2[0];
	double dy = pt1[1] - pt2[1];
	double dz = pt1[2] - pt2[2];

	return sqrtf(dx*dx + dy*dy + dz*dz);
}

double Util::euclideanDistance2D(cv::Point pt1, cv::Point pt2)
{
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrtf(dx*dx + dy*dy);
}

double Util::euclideanDistancePerPixel(cv::Mat xyzMap, cv::Point pt, int radius)
{
	int x = pt.x;
	int y = pt.y;

	int r_lower = (y - radius < 0) ? 0 : y - radius;
	int c_lower = (x - radius < 0) ? 0 : x - radius;
	int r_upper = (y + radius > xyzMap.rows) ? xyzMap.rows : y + radius;
	int c_upper = (x + radius > xyzMap.cols) ? xyzMap.cols : x + radius;

	int count = 0;
	double average = 0;
	for (int r = r_lower; r < r_upper; r++) {
		for (int c = c_lower; c < c_upper; c++) {
			if (xyzMap.at<cv::Vec3f>(r, c)[2] != 0) {
				double distance = euclideanDistance2D(pt, cv::Point(c, r));
				if (distance != 0) {
					average += euclidianDistance3D(xyzMap.at<cv::Vec3f>(pt.y, pt.x), xyzMap.at<cv::Vec3f>(r, c)) / distance;
					count++;
				}
			}
		}
	}

	if (average == 0) {
		return average;
	}
	return average / count;
}

cv::Mat Util::removePoints(cv::Mat img, std::vector<cv::Point2i> points)
{
	cv::Mat result = img.clone();
	for (int i = 0; i < points.size(); i++) {
		int x = points[i].x;
		int y = points[i].y;
		result.at<cv::Vec3f>(y, x)[0] = 0;
		result.at<cv::Vec3f>(y, x)[1] = 0;
		result.at<cv::Vec3f>(y, x)[2] = 0;
	}
	return result;
}

cv::Vec3f Util::averageAroundPoint(cv::Mat xyzMap, cv::Point2i pt, int radius)
{
	int x = pt.x;
	int y = pt.y;

	int r_lower = (y - radius < 0) ? 0 : y - radius;
	int c_lower = (x - radius < 0) ? 0 : x - radius;
	int r_upper = (y + radius > xyzMap.rows) ? xyzMap.rows : y + radius;
	int c_upper = (x + radius > xyzMap.cols) ? xyzMap.cols : x + radius;

	int count = 0;
	cv::Vec3f average;
	for (int r = r_lower; r < r_upper; r++) {
		for (int c = c_lower; c < c_upper; c++) {
			if (xyzMap.at<cv::Vec3f>(r, c)[2] != 0) {
				average[0] += xyzMap.at<cv::Vec3f>(r, c)[0];
				average[1] += xyzMap.at<cv::Vec3f>(r, c)[1];
				average[2] += xyzMap.at<cv::Vec3f>(r, c)[2];
				count++;
			}
		}
	}

	if (count == 0) {
		return 0;
	}

	average[0] /= count;
	average[1] /= count;
	average[2] /= count;

	return average;
}

cv::Point Util::findCentroid(cv::Mat xyzMap)
{
	cv::Mat channels[3];
	cv::split(xyzMap, channels);
	cv::Moments m = cv::moments(channels[2], false);
	cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
	return center;
}

//Function to find Lenght of sides of triangle
double Util::DistanceTwoPoints(double x1, double y1, double x2, double y2)
{
	double x, y, distance;
	x = x2 - x1;
	y = y2 - y1;
	distance = pow(x, 2) + pow(y, 2);
	distance = sqrt(distance);
	return distance;
}

//Function to find angle with Sine rule
double Util::otherAngleFind(double biggerAngle, double largestDistance, double smallDistance)
{
	double otherAngle;
	otherAngle = smallDistance *sin(biggerAngle*3.14159265 / 180);
	otherAngle = otherAngle / largestDistance;
	otherAngle = asin(otherAngle)*180.0 / PI;
	return otherAngle;
}

//Function to find angle opposite to largest side of triangle
double Util::BiggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo)
{
	double biggerAngle;
	biggerAngle = pow(smallDistanceOne, 2) + pow(smallDistanceTwo, 2) - pow(largestDistance, 2);
	biggerAngle = fabs(biggerAngle / (2 * smallDistanceOne*smallDistanceTwo));
	biggerAngle = acos(biggerAngle)* 180.0 / PI;
	return biggerAngle;
}

//Calculate angle of triangle given three coordinates c++ code
double Util::TriangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3)
{
	double dist1, dist2, dist3;
	double angle1, angle2, angle3;
	double total;

	int largestLength = 0;
	dist1 = DistanceTwoPoints(x1, y1, x2, y2);
	dist2 = DistanceTwoPoints(x2, y2, x3, y3);
	dist3 = DistanceTwoPoints(x1, y1, x3, y3);

	if (dist1>dist2 && dist1 > dist3)
	{
		//cout<<"dist1 is greater";
		angle1 = BiggerAngleFind(dist1, dist2, dist3);
		angle2 = otherAngleFind(angle1, dist1, dist2);
		angle3 = otherAngleFind(angle1, dist1, dist3);

		//angle2 = OtherAngleFind(angle1, dist1, dist2);

		total = angle1 + angle2 + angle3;

		if (total <180)
		{
			angle1 = 180 - angle1;
		}
	}
	else if (dist2 > dist3 && dist2 > dist1)
	{
		//  cout<<"dist2 is greater";
		angle2 = BiggerAngleFind(dist2, dist1, dist3);
		angle1 = otherAngleFind(angle2, dist2, dist1);
		angle3 = otherAngleFind(angle2, dist2, dist3);

		total = angle1 + angle2 + angle3;

		if (total <180)
		{
			angle2 = 180 - angle2;
		}
	}
	else
	{
		//    cout<<"dist3 is greater";
		angle3 = BiggerAngleFind(dist3, dist1, dist2);
		angle1 = otherAngleFind(angle3, dist3, dist2);
		angle2 = otherAngleFind(angle3, dist3, dist2);

		total = angle1 + angle2 + angle3;

		if (total <180)
		{
			angle3 = 180 - angle3;
		}
	}

	//cout << endl << "Angle Between First Point and Second Point = " << angle3 << endl;
	//cout << "Angle Between First Point and Third Point = " << angle2 << endl;
	//cout << "Angle Between Second Point and Third Point = " << angle1 << endl;
	return angle2;
}

/***
Recursively performs floodfill on depthMap
***/
void Util::floodFill(int x, int y, cv::Mat& depthMap, cv::Mat& mask, double max_distance)
{
	if (x < 0 || x >= depthMap.cols || y < 0 || y >= depthMap.rows || depthMap.at<cv::Vec3f>(y, x)[2] == 0.0)
		return;
	if (closeEnough(x, y, depthMap, 4, max_distance)) {
		mask.at<cv::Vec3f>(y, x) = depthMap.at<cv::Vec3f>(y, x);
		depthMap.at<cv::Vec3f>(y, x)[0] = 0;
		depthMap.at<cv::Vec3f>(y, x)[1] = 0;
		depthMap.at<cv::Vec3f>(y, x)[2] = 0;
	}
	else {
		return;
	}

	floodFill(x + 1, y, depthMap, mask, max_distance);
	floodFill(x - 1, y, depthMap, mask, max_distance);
	floodFill(x, y + 1, depthMap, mask, max_distance);
	floodFill(x, y - 1, depthMap, mask, max_distance);
}

/***
Check whether candidate point is close enough to neighboring points
***/
bool Util::closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance)
{
	int num_close = 0;
	if (x - 1 < 0 || depthMap.at<cv::Vec3f>(y, x - 1)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x - 1)) < max_distance) {
		num_close++;
	}
	if (x + 1 >= depthMap.cols || depthMap.at<cv::Vec3f>(y, x + 1)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x + 1)) < max_distance) {
		num_close++;
	}
	if (y - 1 < 0 || depthMap.at<cv::Vec3f>(y - 1, x)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y - 1, x)) < max_distance) {
		num_close++;
	}
	if (y + 1 >= depthMap.rows || depthMap.at<cv::Vec3f>(y + 1, x)[2] == 0 || Util::euclidianDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y + 1, x)) < max_distance) {
		num_close++;
	}

	if (num_close >= num_neighbors) {
		return true;
	}

	return false;
}