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
    return magnitude(pt1 - pt2);
}

double Util::euclideanDistance2D(cv::Point pt1, cv::Point pt2)
{
    return magnitude(pt1 - pt2);
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
    int x = pt.x, y = pt.y;
    int r_lower = std::max(0, y - radius);
    int c_lower = std::max(0, x - radius);
    int r_upper = std::min(xyzMap.rows - 1, y + radius);
    int c_upper = std::min(xyzMap.cols - 1, x + radius);

    int count = 0;
    cv::Vec3f average(0, 0, 0);

    for (int r = r_lower; r <= r_upper; ++r)
    {
        cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);
        for (int c = c_lower; c <= c_upper; ++c)
        {
            if (ptr[c][2] > 0)
            {
                average[0] += ptr[c][0];
                average[1] += ptr[c][1];
                average[2] += ptr[c][2];
                ++count;
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

double Util::averageDepth(cv::Mat xyzMap){
    double total = 0.0;
    int numPts = 0;

    for (int r = 0; r < xyzMap.rows; ++r)
    {
        cv::Vec3f * ptr = xyzMap.ptr<cv::Vec3f>(r);
        for (int c = 0; c < xyzMap.cols; ++c)
        {
            if (ptr[c][2] != 0){
                total += ptr[c][2];
                ++numPts;
            }
        }
    }

    return total / numPts;
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

double Util::triangleArea(cv::Vec3f a, cv::Vec3f b, cv::Vec3f c)
{
    cv::Vec3f v0 = a - c, v1 = b - c;
    cv::Vec3f cross(v0[1] * v1[2] - v1[1] * v0[2],
                    v0[2] * v1[0] - v1[2] * v0[0],
                    v0[0] * v1[1] - v1[0] * v0[1]);
    return magnitude(cross) / 2.0;
}

double Util::quadrangleArea(cv::Vec3f pts[4])
{
    int valid = 0, bad = -1;
    for (int i = 0; i < 4; ++i) {
        if (pts[i][2] > 0.0) ++valid;
        else bad = i;
    }

    if (valid == 4) {
        // if all four points are nonzero, add both triangles
        double a1 = triangleArea(pts[1], pts[2], pts[0]),
               a2 = triangleArea(pts[1], pts[2], pts[3]);

        return a1 + a2;
    }
    else if (valid == 3) {
        cv::Vec3f t[] = { pts[0], pts[1], pts[2] };
        // swap to make sure the three good points are in the first three positions
        if (bad != 3) t[bad] = pts[3];

        // if three of four points are nonzero, add the triangle formed by these points
        return triangleArea(t[1], t[2], t[0]);
    }
    else {
        // if there are <= 2 points: ignore this set of four points
        return 0;
    }
}

double Util::surfaceArea(cv::Mat & depthMap)
{
    if (depthMap.rows == 0 || depthMap.cols == 0) return 0.0;

    double total = 0.0;

    cv::Vec3f * ptr, * nxPtr = depthMap.ptr<cv::Vec3f>(0);

    for (int r = 1; r < depthMap.rows; ++r) {
        ptr = nxPtr; // reuse previous pointer; upper row
        nxPtr = depthMap.ptr<cv::Vec3f>(r); // lower row

        //                { top left, top right, bottom left, bottom right}
        cv::Vec3f pts[] = { ptr[0],   ptr[0],    nxPtr[0],    nxPtr[0] };
        const int NUM_PTS = (sizeof pts ) / (sizeof pts[0]);

        for (int c = 1; c < depthMap.cols; ++c) {
            pts[0] = pts[1]; pts[2] = pts[3]; // reuse previous points
            pts[1] = ptr[c]; pts[3] = nxPtr[c];

            total += quadrangleArea(pts);
        }
    }

    return total;
}

 double Util::surfaceArea(const cv::Mat & depthMap, std::vector<cv::Point> & cluster, bool sorted, int clusterSize) {
    if (clusterSize < 0 || clusterSize > (int)cluster.size()) 
        clusterSize = (int)cluster.size(); // default cluster size = vector size

    if (clusterSize < 3) return 0;

    if (!sorted)
        radixSortPoints(cluster, depthMap.cols, depthMap.rows, clusterSize);
        //sort(cluster.begin(), cluster.begin() + clusterSize, Util::PointComparer<cv::Point>(false, true));

    std::vector<int> rows;
    std::vector<cv::Vec3f> xyz;
    xyz.reserve(clusterSize);

    for (unsigned i = 0; i < clusterSize; ++i) {
        
        cv::Vec3f xyzPt = depthMap.at<cv::Vec3f>(cluster[i]);
        xyz.push_back(xyzPt);

        if (!i || cluster[i].y > cluster[i - 1].y)
            rows.push_back(i);
    }

    rows.push_back(clusterSize);

    double total = 0.0;

    for (uint i = 0; i < rows.size() - 2; ++i) {
        int nx = rows[i + 1];

        for (uint j = rows[i]; j < rows[i + 1] - 1; ++j) {
            uint idx = j - rows[i];

            if (cluster[nx].x < cluster[j].x && nx < rows[i + 2]) {
                auto it1 = cluster.begin() + (nx + 1);
                auto it2 = cluster.begin() + (rows[i+2]);
                nx = std::lower_bound(it1, it2, cv::Point(cluster[j].x, cluster[nx].y), 
                    Util::PointComparer<cv::Point>(0, true)) 
                    - cluster.begin();
            }

            if (nx >= rows[i+2] || cluster[nx].x > cluster[j].x || cluster[nx].y - cluster[j].y > 1) continue;

            cv::Vec3f pts[4] = {xyz[j], xyz[j+1], xyz[nx], xyz[nx+1]};

            if (cluster[j+1].y != cluster[j].y ||
                cluster[j+1].x - cluster[j].x > 1) pts[1][2] = 0;

            if (cluster[nx + 1].y != cluster[nx].y ||
                cluster[nx + 1].x - cluster[nx].x > 1) pts[3][2] = 0;

            total += quadrangleArea(pts);
            ++nx;
        }
    }

    if (isnan(total)) return 0.0;

    return total;
}

// By Justin W Yang
double Util::surfaceAreaCircle(cv::Mat shape) {
    cv::Size size = shape.size();
    cv::Rect rect(0, 0, size.width, size.height);

    int dr[8] = {1, 0, -1, 0, 1, -1, 1, -1};
    int dc[8] = {0, 1, 0, -1, 1, 1, -1, -1};

    // printf("%d %d\n", size.width, size.height);

    double surfArea = 0;
    for (int r = 0; r < size.height; r++)
    {
        for (int c = 0; c < size.width; c++)
        {
            cv::Vec3f point = shape.at<cv::Vec3f>(r, c);
            if (point[2] == 0) {
                continue;
            }

            const int MAX = 2147483647;
            double radius = MAX;
            for (int idx = 0; idx < 8; idx++) {
                int nr = r + dr[idx];
                int nc = c + dc[idx];

                if (nr < 0 || nr >= size.height || nc < 0 || nc >= size.width) {
                    continue;
                }

                cv::Vec3f adjPoint = shape.at<cv::Vec3f>(nr, nc);
                // printf("%d %d %f %f %f\n", nr, nc, adjPoint[0], adjPoint[1], adjPoint[2]);
                if (adjPoint[2] == 0) {
                    continue;
                }

                double dist = euclideanDistance3D(point, adjPoint);
                radius = (dist < radius) ? dist : radius;
            }

            if (radius != MAX) {
                surfArea += M_PI * radius * radius;
            }
        }
    }

    return surfArea;
}

// By Justin W Yang
double Util::surfaceAreaTriangulate(cv::Mat shape) {
    cv::Size size = shape.size();
    cv::Rect rect(0, 0, size.width, size.height);

    int dr[4] = {0, 1, 0, -1};
    int dc[4] = {-1, 0, 1, 0};

    // printf("%d %d\n", size.width, size.height);

    double surfArea = 0;
    for (int r = 0; r < size.height; r++)
    {
        for (int c = 0; c < size.width; c++)
        {
            cv::Vec3f point = shape.at<cv::Vec3f>(r, c);
            if (point[2] == 0) {
                continue;
            }

            cv::Vec3f adj[4];
            bool validPoint[4] = {false};
            for (int idx = 0; idx < 4; idx++) {
                int nr = r + dr[idx];
                int nc = c + dc[idx];

                if (nr < 0 || nr >= size.height || nc < 0 || nc >= size.width) {
                    continue;
                }

                cv::Vec3f adjPoint = shape.at<cv::Vec3f>(nr, nc);
                // printf("%d %d %f %f %f\n", nr, nc, adjPoint[0], adjPoint[1], adjPoint[2]);
                if (adjPoint[2] == 0) {
                    continue;
                }

                adj[idx] = adjPoint;
                validPoint[idx] = true;
            }

            if (validPoint[0] && validPoint[1]) {
                double dist1 = euclideanDistance3D(point, adj[0]);
                double dist2 = euclideanDistance3D(point, adj[1]);
                double dist3 = euclideanDistance3D(adj[0], adj[1]);
                double s = (dist1 + dist2 + dist3) / 2;
                // printf("%f %f %f %f\n", s, dist1, dist2, dist3);
                surfArea += sqrt(abs(s * (s - dist1) * (s - dist2) * (s - dist3)));
            }

            if (validPoint[2] && validPoint[3]) {
                double dist1 = euclideanDistance3D(point, adj[2]);
                double dist2 = euclideanDistance3D(point, adj[3]);
                double dist3 = euclideanDistance3D(adj[2], adj[3]);
                double s = (dist1 + dist2 + dist3) / 2;
                // printf("%f %f %f %f\n", s, dist1, dist2, dist3);
                surfArea += sqrt(abs(s * (s - dist1) * (s - dist2) * (s - dist3)));
            }
        }
    }

    return surfArea;
}

////Function to find Lenght of sides of triangle
//double Util::distanceTwoPoints(double x1, double y1, double x2, double y2)
//{
//    auto x = x2 - x1;
//    auto y = y2 - y1;
//    auto distance = pow(x, 2) + pow(y, 2);
//    distance = sqrt(distance);
//    return distance;
//}
//
////Function to find angle with Sine rule
//double Util::otherAngleFind(double biggerAngle, double largestDistance, double smallDistance)
//{
//    auto otherAngle = smallDistance *sin(biggerAngle*3.14159265 / 180);
//    otherAngle = otherAngle / largestDistance;
//    otherAngle = asin(otherAngle)*180.0 / PI;
//    return otherAngle;
//}
//
////Function to find angle opposite to largest side of triangle
//double Util::biggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo)
//{
//    auto biggerAngle = pow(smallDistanceOne, 2) + pow(smallDistanceTwo, 2) - pow(largestDistance, 2);
//    biggerAngle = fabs(biggerAngle / (2 * smallDistanceOne*smallDistanceTwo));
//    biggerAngle = acos(biggerAngle)* 180.0 / PI;
//    return biggerAngle;
//}
//
////Calculate angle of triangle given three coordinates
//double Util::triangleAngleCalculation(double x1, double y1, double x2, double y2, double x3, double y3)
//{
//    double angle1, angle2, angle3;
//    double total;
//    auto largestLength = 0;
//    auto dist1 = distanceTwoPoints(x1, y1, x2, y2);
//    auto dist2 = distanceTwoPoints(x2, y2, x3, y3);
//    auto dist3 = distanceTwoPoints(x1, y1, x3, y3);
//
//    if (dist1>dist2 && dist1 > dist3)
//    {
//        angle1 = biggerAngleFind(dist1, dist2, dist3);
//        angle2 = otherAngleFind(angle1, dist1, dist2);
//        angle3 = otherAngleFind(angle1, dist1, dist3);
//
//        //angle2 = OtherAngleFind(angle1, dist1, dist2);
//
//        total = angle1 + angle2 + angle3;
//
//        if (total <180)
//        {
//            angle1 = 180 - angle1;
//        }
//    }
//
//    else if (dist2 > dist3 && dist2 > dist1)
//    {
//        angle2 = biggerAngleFind(dist2, dist1, dist3);
//        angle1 = otherAngleFind(angle2, dist2, dist1);
//        angle3 = otherAngleFind(angle2, dist2, dist3);
//        total = angle1 + angle2 + angle3;
//
//        if (total <180)
//        {
//            angle2 = 180 - angle2;
//        }
//    }
//
//    else
//    {
//        angle3 = biggerAngleFind(dist3, dist1, dist2);
//        angle1 = otherAngleFind(angle3, dist3, dist2);
//        angle2 = otherAngleFind(angle3, dist3, dist2);
//
//        total = angle1 + angle2 + angle3;
//
//        if (total <180)
//        {
//            angle3 = 180 - angle3;
//        }
//    }
//
//    return angle2;
//}
//
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

double Util::magnitude(cv::Point2f pt) {
    return sqrt(pt.x * pt.x + pt.y + pt.y);
}

double Util::magnitude(cv::Point pt) {
    return sqrt(pt.x * pt.x + pt.y + pt.y);
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

bool Util::pointInImage(const cv::Mat & img, const cv::Point pt, int scale) {
    return pt.x >= 0 && pt.x < img.cols * scale &&
           pt.y >= 0 && pt.y < img.rows * scale;
}

void Util::radixSortPoints(std::vector<cv::Point> & points, int wid, int hi, int num_pts) {
    if (num_pts < 0 || num_pts >(int)points.size())
        num_pts = (int)points.size();
   
    std::vector<std::vector<cv::Point> > buck(std::max(wid, hi));

    // order by x
    for (int i = 0; i < num_pts; ++i) buck[points[i].x].push_back(points[i]);

    int idx = -1;
    for (int i = 0; i < wid; ++i) {
        for (unsigned j = 0; j < buck[i].size(); ++j) {
            points[++idx] = buck[i][j];
        }
        buck[i].clear();
    }

    // order by y
    for (int i = 0; i < num_pts; ++i) buck[points[i].y].push_back(points[i]);

    idx = -1;
    for (int i = 0; i < hi; ++i) {
        for (unsigned j = 0; j < buck[i].size(); ++j) {
            points[++idx] = buck[i][j];
        }
        buck[i].clear();
    }
}

cv::Point Util::findPointOnCluster(const cv::Mat m, cv::Point pt, int max_tries) {
    int tries = 0, run = 1, len = 2, direction = 0;

    pt.x = std::min(std::max(0, pt.x), m.cols-1);
    pt.y = std::min(std::max(0, pt.y), m.rows-1);

    cv::Vec3f xyz = m.at<cv::Vec3f>(pt);
    cv::Point orig_pt = pt;

    // travel in a spiral and find a nearby nonzero point
    while (xyz[2] == 0  && tries < max_tries) {
        switch (direction) {
        case 0:
            ++pt.y;
            break;
        case 1:
            ++pt.x;
            break;
        case 2:
            --pt.y;
            break;
        default:
            --pt.x;
            break;
        }
        if ((--run) == 0) {
            run = (++len) / 2;
            direction = (direction + 1) % 4;
        }

        ++tries;
        if (!pointInImage(m, pt)) continue;

        xyz = m.at<cv::Vec3f>(pt);
    }

    if (tries >= max_tries) return orig_pt;

    return pt;
}

double Util::clusterShortestPath(const cv::Mat & m, cv::Point src, cv::Point sink) {
    if (!Util::pointInImage(m, src) || !Util::pointInImage(m, sink)) return FLT_MAX;

    int N = (m.rows + 1) * (m.cols + 1);
    std::set<std::pair<double, std::pair<int, int> > > st;

    st.insert(std::make_pair(0.0, std::make_pair(src.x, src.y)));
    
    cv::Vec3f sinkXyz = m.at<cv::Vec3f>(sink);

    double * dist = new double[N];

    bool * vis = new bool[N];
    memset(vis, 0, N * sizeof(bool));

    // A* Heuristic
    double * snkDist = new double[N];

    for (int r = 0; r<m.rows; ++r){
        const cv::Vec3f * ptr = m.ptr<cv::Vec3f>(r);
        for (int c = 0; c < m.cols; ++c) {
            dist[c + r * m.cols] = FLT_MAX;
            if (ptr[c][2] != 0) {
                snkDist[c + r * m.cols] = euclideanDistance3D(ptr[c], sinkXyz);
            }
        }
    }

    dist[src.x + src.y * m.cols] = 0;

    static const cv::Point nxtPoints[] = 
                 {
                   cv::Point(-1, 0), cv::Point(0, -1), cv::Point(0, 1), cv::Point(1, 0),
                   cv::Point(-5, 0), cv::Point(0, -5), cv::Point(0, 5), cv::Point(5, 0),
                   cv::Point(-1, -1), cv::Point(1, -1), cv::Point(1, 1), cv::Point(-1, 1),
                 };

    static const int nNxtPoints = (sizeof nxtPoints) / (sizeof nxtPoints[0]);

    while (!st.empty()) {
        auto tmpPair = *st.begin();
        cv::Point pt(tmpPair.second.first, tmpPair.second.second);
        if (pt == sink) break;

        vis[pt.x + pt.y * m.cols] = true;
        st.erase(st.begin());

        const cv::Vec3f & xyz = m.at<cv::Vec3f>(pt);

        double currDist = dist[pt.x + pt.y * m.cols];

        for (int i = 0; i < nNxtPoints; ++i) {
            // go to each adjacent point

            cv::Point adjPt = pt + nxtPoints[i];
            int adjIdx = adjPt.x + adjPt.y * m.cols;

            if (!Util::pointInImage(m, adjPt) || vis[adjIdx]) continue;

            const cv::Vec3f & adjXyz = m.at<cv::Vec3f>(adjPt);
            if (adjXyz[2] == 0) continue;

            double * adjDist = &dist[adjIdx];

            double between = euclideanDistance3D(adjXyz, xyz);

            if (currDist + between < *adjDist) {
                auto oldPair = std::make_pair(*adjDist + snkDist[adjIdx],
                                              std::make_pair(adjPt.x, adjPt.y));
                auto it = st.find(oldPair);
                if (it != st.end()) st.erase(it);

                *adjDist = currDist + between;
                auto newPair = std::make_pair(*adjDist + snkDist[adjIdx],
                                               std::make_pair(adjPt.x, adjPt.y));
                st.insert(newPair);
            }
        }
    }

    double result = dist[sink.x + sink.y * m.cols];
    delete[] dist;
    delete[] vis;
    delete[] snkDist;

    return result;
}

bool Util::PointComparer<cv::Point>::operator()(cv::Point a, cv::Point b) {
    if (compare_y_then_x) {
        if (a.y == b.y) return reverse ^ (a.x < b.x);
        return reverse ^ (a.y < b.y);
    }
    else{
        if (a.x == b.x) return reverse ^ (a.y < b.y);
        return reverse ^ (a.x < b.x);
    }
}

bool Util::PointComparer<cv::Point2f>::operator()(cv::Point2f a, cv::Point2f b) {
    if (compare_y_then_x) {
        if (a.y == b.y) return reverse ^ (a.x < b.x);
        return reverse ^ (a.y < b.y);
    }
    else{
        if (a.x == b.x) return reverse ^ (a.y < b.y);
        return reverse ^ (a.x < b.x);
    }
}

bool Util::PointComparer<cv::Vec3f>::operator()(cv::Vec3f a, cv::Vec3f b) {
    for (int i = (compare_y_then_x ? 2 : 0);
        (compare_y_then_x ? i >= 0 : i < 3);
        (compare_y_then_x ? --i : ++i)) {
        if (a[i] == b[i]) continue;

        return reverse ^ (a[i] < b[i]);
    }
    return false;
}

bool Util::PointComparer<cv::Vec3i>::operator()(cv::Vec3i a, cv::Vec3i b) {
    for (int i = (compare_y_then_x ? 2 : 0);
        (compare_y_then_x ? i >= 0 : i < 3);
        (compare_y_then_x ? --i : ++i)) {
        if (a[i] == b[i]) continue;

        return reverse ^ (a[i] < b[i]);
    }
    return false;
}
