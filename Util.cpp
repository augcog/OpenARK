#include "stdafx.h"
#include "Util.h"

std::vector<std::string> Util::split(char* string_in, char* delimeters){
    std::auto_ptr<char> buffer(new char[strlen(string_in) + 1]);
    strcpy(buffer.get(),string_in);
    char* token;
    std::vector<std::string> strings_out;
    token = strtok (buffer.get(),delimeters);
    while (token != NULL)
    {
        strings_out.push_back(std::string(token));
        token = strtok (NULL, delimeters);
    }
    return strings_out;
}

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
    if (x < 0 || y < 0 || x >= image.cols || y >= image.rows)
    {
        return false;
    }

    if (image.at<uchar>(y, x) != 0)
    {
        return true;
    }

    return false;
}

int Util::getDistanceT(int x1, int y1, int x2, int y2)
{
    return abs(y1 - y2) + abs(x2 - x1);
}

double Util::euclideanDistance3D(cv::Vec3f pt1, cv::Vec3f pt2)
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
    auto x = pt.x;
    auto y = pt.y;

    auto r_lower = (y - radius < 0) ? 0 : y - radius;
    auto c_lower = (x - radius < 0) ? 0 : x - radius;
    auto r_upper = (y + radius > xyzMap.rows) ? xyzMap.rows : y + radius;
    auto c_upper = (x + radius > xyzMap.cols) ? xyzMap.cols : x + radius;

    auto count = 0;
    double average = 0;

    for (auto r = r_lower; r < r_upper; r++)
    {

        for (auto c = c_lower; c < c_upper; c++)
        {

            if (xyzMap.at<cv::Vec3f>(r, c)[2] != 0)
            {

                auto distance = euclideanDistance2D(pt, cv::Point(c, r));

                if (distance != 0)
                {
                    average += euclideanDistance3D(xyzMap.at<cv::Vec3f>(pt.y, pt.x), xyzMap.at<cv::Vec3f>(r, c)) / distance;
                    count++;
                }
            }
        }
    }

    if (average == 0)
    {
        return average;
    }

    return average / count;
}

cv::Mat Util::removePoints(cv::Mat img, std::vector<cv::Point2i> points)
{
    auto result = img.clone();

    for (auto i = 0; i < points.size(); i++)
    {
        auto x = points[i].x;
        auto y = points[i].y;
        result.at<cv::Vec3f>(y, x)[0] = 0;
        result.at<cv::Vec3f>(y, x)[1] = 0;
        result.at<cv::Vec3f>(y, x)[2] = 0;
    }

    return result;
}

cv::Vec3f Util::averageAroundPoint(cv::Mat xyzMap, cv::Point2i pt, int radius)
{
    auto x = pt.x;
    auto y = pt.y;
    auto r_lower = (y - radius < 0) ? 0 : y - radius;
    auto c_lower = (x - radius < 0) ? 0 : x - radius;
    auto r_upper = (y + radius > xyzMap.rows) ? xyzMap.rows : y + radius;
    auto c_upper = (x + radius > xyzMap.cols) ? xyzMap.cols : x + radius;
    auto count = 0;
    cv::Vec3f average;

    for (auto r = r_lower; r < r_upper; r++)
    {

        for (auto c = c_lower; c < c_upper; c++)
        {

            if (xyzMap.at<cv::Vec3f>(r, c)[2] != 0)
            {
                average[0] += xyzMap.at<cv::Vec3f>(r, c)[0];
                average[1] += xyzMap.at<cv::Vec3f>(r, c)[1];
                average[2] += xyzMap.at<cv::Vec3f>(r, c)[2];
                count++;
            }
        }
    }

    if (count == 0)
    {
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
    //using image moments to find center of mass of the depth image
    auto m = cv::moments(channels[2], false);
    //Cx=M10/M00 and Cy=M01/M00
    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
    return center;
}

//Function to find Lenght of sides of triangle
double Util::distanceTwoPoints(double x1, double y1, double x2, double y2)
{
    auto x = x2 - x1;
    auto y = y2 - y1;
    auto distance = pow(x, 2) + pow(y, 2);
    distance = sqrt(distance);
    return distance;
}

//Function to find angle with Sine rule
double Util::otherAngleFind(double biggerAngle, double largestDistance, double smallDistance)
{
    auto otherAngle = smallDistance *sin(biggerAngle*3.14159265 / 180);
    otherAngle = otherAngle / largestDistance;
    otherAngle = asin(otherAngle)*180.0 / PI;
    return otherAngle;
}

//Function to find angle opposite to largest side of triangle
double Util::biggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo)
{
    auto biggerAngle = pow(smallDistanceOne, 2) + pow(smallDistanceTwo, 2) - pow(largestDistance, 2);
    biggerAngle = fabs(biggerAngle / (2 * smallDistanceOne*smallDistanceTwo));
    biggerAngle = acos(biggerAngle)* 180.0 / PI;
    return biggerAngle;
}

//Calculate angle of triangle given three coordinates
double Util::triangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double angle1, angle2, angle3;
    double total;
    auto largestLength = 0;
    auto dist1 = distanceTwoPoints(x1, y1, x2, y2);
    auto dist2 = distanceTwoPoints(x2, y2, x3, y3);
    auto dist3 = distanceTwoPoints(x1, y1, x3, y3);

    if (dist1>dist2 && dist1 > dist3)
    {
        angle1 = biggerAngleFind(dist1, dist2, dist3);
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
        angle2 = biggerAngleFind(dist2, dist1, dist3);
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
        angle3 = biggerAngleFind(dist3, dist1, dist2);
        angle1 = otherAngleFind(angle3, dist3, dist2);
        angle2 = otherAngleFind(angle3, dist3, dist2);

        total = angle1 + angle2 + angle3;

        if (total <180)
        {
            angle3 = 180 - angle3;
        }
    }
    return angle2;
}

/***
Recursively performs floodfill on depthMap for image segmentation
Determines pixels in an image that are similar to a seed pixel and connected to it
***/
void Util::floodFill(int x, int y, cv::Mat& depthMap, cv::Mat& mask, double max_distance)
{
    //check to see if the point (x,y) is within the depth image
    //check to see if the end of recursion by checking if depth map is all zero
    if (x < 0 || x >= depthMap.cols || y < 0 || y >= depthMap.rows || depthMap.at<cv::Vec3f>(y, x)[2] == 0.0)
        return;
    //using 4-connectivity to determine if a pixel is connected to another one
    if (closeEnough(x, y, depthMap, 4, max_distance))
    //if (closeEnough(x, y, depthMap, 8, max_distance)) //would using 8-connectivity give more accurate results?
    {
        //copy the depth map value at (x,y) to mask and zero it out in the depth map
        mask.at<cv::Vec3f>(y, x) = depthMap.at<cv::Vec3f>(y, x);
        depthMap.at<cv::Vec3f>(y, x)[0] = 0;
        depthMap.at<cv::Vec3f>(y, x)[1] = 0;
        depthMap.at<cv::Vec3f>(y, x)[2] = 0;
    }

    else
    {
        return;
    }
    //try the floodfill algorithm recursively for all the 4-connected neighbors
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
    auto num_close = 0;
    //check to see if the neighbor pixels Euclidean distance is within defined max distance
    if (x - 1 < 0 || depthMap.at<cv::Vec3f>(y, x - 1)[2] == 0 ||
        Util::euclideanDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x - 1)) < max_distance)
    {
        num_close++;
    }

    if (x + 1 >= depthMap.cols || depthMap.at<cv::Vec3f>(y, x + 1)[2] == 0 ||
        Util::euclideanDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y, x + 1)) < max_distance)
    {
        num_close++;
    }

    if (y - 1 < 0 || depthMap.at<cv::Vec3f>(y - 1, x)[2] == 0 ||
        Util::euclideanDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y - 1, x)) < max_distance)
    {
        num_close++;
    }

    if (y + 1 >= depthMap.rows || depthMap.at<cv::Vec3f>(y + 1, x)[2] == 0 ||
        Util::euclideanDistance3D(depthMap.at<cv::Vec3f>(y, x), depthMap.at<cv::Vec3f>(y + 1, x)) < max_distance)
    {
        num_close++;
    }

    if (num_close >= num_neighbors)
    {
        return true;
    }

    return false;
}

// convert an ij point to an angle, clockwise from the bottom (0 at 0 degrees, 2 * PI at 360)
double Util::pointToAngle(cv::Point pt) {
    double arctan = atan((double)abs(pt.x) / abs(pt.y));

    if (pt.x <= 0) {
        if (pt.y >= 0)
            return arctan;
        else // pt.y > 0
            return PI - arctan;
    }
    else { // pt.x <= 0
        if (pt.y <= 0)
            return PI + arctan;
        else // pt.y <= 0
            return 2 * PI - arctan;
    }
}

// get angle between two points through a central point
double Util::angleBetweenPoints(cv::Point a, cv::Point b, cv::Point center) {
    a -= center; b -= center;
    double angle = abs(pointToAngle(a) - pointToAngle(b));
    if (angle > PI) return 2 * PI - angle;
    return angle;
}

// convert an ij point to a slope, clockwise from the bottom (0 at 0 degrees, FLT_MAX at 360)
double Util::pointToSlope(cv::Point pt) {
    double ratio, step = FLT_MAX / 4;

    if (pt.y == 0) ratio = step;
    else ratio = std::min((double)abs(pt.x) / abs(pt.y), step);

    if (pt.x <= 0) {
        if (pt.y >= 0)
            return ratio;
        else // pt.y > 0
            return 2 * step - ratio;
    }
    else { // pt.x <= 0
        if (pt.y <= 0)
            return 2 * step + ratio;
        else // pt.y <= 0
            return 24 * step - ratio;
    }
}

double Util::magnitude(cv::Vec3f a) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// get angle between two 3D vectors through a central point
double Util::angleBetween3DVec(cv::Vec3f a, cv::Vec3f b, cv::Vec3f center) {
    a -= center; b -= center;
    cv::Mat A(a), B(b);
    double dot = A.dot(B), mA = magnitude(a), mB = magnitude(b);

    double res = abs(acos(dot / mA / mB));

    if (isnan(res)) return PI;
    else return res;
}

