#include "stdafx.h"
#include "version.h"
#include "Util.h"

namespace ark {

    namespace util {
        std::vector<std::string> split(char* string_in, char* delimeters) {
            std::auto_ptr<char> buffer(new char[strlen(string_in) + 1]);
            strcpy(buffer.get(), string_in);
            char* token;
            std::vector<std::string> strings_out;
            token = strtok(buffer.get(), delimeters);
            while (token != NULL)
            {
                strings_out.push_back(std::string(token));
                token = strtok(NULL, delimeters);
            }
            return strings_out;
        }

        Vec3b colorGenerator()
        {
            return Vec3b(rand() % 256, rand() % 256, rand() % 256);
        }

        float normalize(float a, float b)
        {
            return sqrt(a*a + b*b);
        }

        bool isMember(cv::Mat image, int x, int y)
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

        template<class T>
        float euclideanDistance(cv::Point_<T> pt1, cv::Point_<T> pt2)
        {
            return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
        }

        template float euclideanDistance<int>(cv::Point_<int> pt1, cv::Point_<int> pt2);
        template float euclideanDistance<float>(cv::Point_<float> pt1, cv::Point_<float> pt2);
        template float euclideanDistance<double>(cv::Point_<double> pt1, cv::Point_<double> pt2);

        template<class T>
        float euclideanDistance(cv::Vec<T, 3> pt1, cv::Vec<T, 3> pt2)
        {
            return sqrtf((pt1[0] - pt2[0]) * (pt1[0] - pt2[0]) +
                (pt1[1] - pt2[1]) * (pt1[1] - pt2[1]) +
                (pt1[2] - pt2[2]) * (pt1[2] - pt2[2]));
        }

        template float euclideanDistance<uchar>(cv::Vec<uchar, 3> pt1, cv::Vec<uchar, 3> pt2);
        template float euclideanDistance<int>(cv::Vec<int, 3> pt1, cv::Vec<int, 3> pt2);
        template float euclideanDistance<float>(cv::Vec<float, 3> pt1, cv::Vec<float, 3> pt2);
        template float euclideanDistance<double>(cv::Vec<double, 3> pt1, cv::Vec<double, 3> pt2);

        double euclideanDistancePerPixel(cv::Mat xyzMap, Point2i pt, int radius)
        {
            int r_lower = (pt.y - radius < 0) ? 0 : pt.y - radius,
                c_lower = (pt.x - radius < 0) ? 0 : pt.x - radius,
                r_upper = (pt.y + radius > xyzMap.rows) ? xyzMap.rows : pt.y + radius,
                c_upper = (pt.x + radius > xyzMap.cols) ? xyzMap.cols : pt.x + radius;

            int count = 0;
            double average = 0;

            for (int r = r_lower; r < r_upper; r++)
            {

                for (int c = c_lower; c < c_upper; c++)
                {

                    if (xyzMap.at<Vec3f>(r, c)[2] != 0)
                    {

                        double distance = euclideanDistance(pt, Point2i(c, r));

                        if (distance != 0)
                        {
                            average += euclideanDistance(xyzMap.at<Vec3f>(pt.y, pt.x), xyzMap.at<Vec3f>(r, c)) / distance;
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

        cv::Mat removePoints(cv::Mat img, std::vector<Point2i> points)
        {
            cv::Mat result = img.clone();

            for (int i = 0; i < points.size(); i++)
            {
                result.at<Vec3f>(points[i]) = 0;
            }

            return result;
        }

        Vec3f averageAroundPoint(cv::Mat xyzMap, Point2i pt, int radius)
        {
            int r_lower = std::max(0, pt.y - radius);
            int c_lower = std::max(0, pt.x - radius);
            int r_upper = std::min(xyzMap.rows - 1, pt.y + radius);
            int c_upper = std::min(xyzMap.cols - 1, pt.x + radius);

            int count = 0;
            Vec3f average(0, 0, 0);

            for (int r = r_lower; r <= r_upper; ++r)
            {
                Vec3f * ptr = xyzMap.ptr<Vec3f>(r);
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
                return 0;

            average[0] /= count;
            average[1] /= count;
            average[2] /= count;

            return average;
        }

        double averageDepth(cv::Mat xyzMap) {
            double total = 0.0;
            int numPts = 0;

            for (int r = 0; r < xyzMap.rows; ++r)
            {
                Vec3f * ptr = xyzMap.ptr<Vec3f>(r);
                for (int c = 0; c < xyzMap.cols; ++c)
                {
                    if (ptr[c][2] != 0) {
                        total += ptr[c][2];
                        ++numPts;
                    }
                }
            }

            return total / numPts;
        }

        Point2i findCentroid(cv::Mat xyzMap)
        {
            cv::Mat channels[3];
            cv::split(xyzMap, channels);
            //using image moments to find center of mass of the depth image
            auto m = cv::moments(channels[2], false);
            //Cx=M10/M00 and Cy=M01/M00
            Point2i center(m.m10 / m.m00, m.m01 / m.m00);
            return center;
        }

        static inline float getTriangleArea(Vec3f a, Vec3f b, Vec3f c)
        {
            Vec3f v0 = a - c, v1 = b - c;
            Vec3f cross(v0[1] * v1[2] - v1[1] * v0[2],
                v0[2] * v1[0] - v1[2] * v0[0],
                v0[0] * v1[1] - v1[0] * v0[1]);
            return magnitude(cross) / 2.0f;
        }

        float triangleArea(Vec3f a, Vec3f b, Vec3f c)
        {
            return getTriangleArea(a, b, c);
        }

        float quadrangleArea(Vec3f pts[4])
        {
            int valid = 0, bad = -1;
            for (int i = 0; i < 4; ++i) {
                if (pts[i][2] > 0.0f) ++valid;
                else bad = i;
            }

            if (valid == 4) {
                // if all four points are nonzero, add both triangles
                float a1 = triangleArea(pts[1], pts[2], pts[0]);
                float a2 = triangleArea(pts[1], pts[2], pts[3]);

                return a1 + a2;
            }
            else if (valid == 3) {
                // swap to make sure the three good points are in the first three positions
                if (bad != 3) pts[bad] = pts[3];

                // if three of four points are nonzero, add the triangle formed by these points
                return triangleArea(pts[0], pts[1], pts[2]);
            }
            else {
                // if there are <= 2 points: ignore this set of four points
                return 0.0f;
            }
        }

        double surfaceArea(cv::Mat & depthMap)
        {
            if (depthMap.rows == 0 || depthMap.cols == 0) return 0.0;

            double total = 0.0;

            Vec3f * ptr, *nxPtr = depthMap.ptr<Vec3f>(0);

            for (int r = 1; r < depthMap.rows; ++r) {
                ptr = nxPtr; // reuse previous pointer; upper row
                nxPtr = depthMap.ptr<Vec3f>(r); // lower row

                //                { top left, top right, bottom left, bottom right}
                Vec3f pts[] = { ptr[0],   ptr[0],    nxPtr[0],    nxPtr[0] };
                const int NUM_PTS = (sizeof pts) / (sizeof pts[0]);

                for (int c = 1; c < depthMap.cols; ++c) {
                    pts[0] = pts[1]; pts[2] = pts[3]; // reuse previous points
                    pts[1] = ptr[c]; pts[3] = nxPtr[c];

                    total += quadrangleArea(pts);
                }
            }

            return total;
        }

        double surfaceArea(const cv::Mat & depth_map,
            std::vector<Point2i> & points_ij,
            std::vector<Vec3f> & points_xyz,
            bool sorted, int cluster_size) {

            if (cluster_size < 0 || cluster_size >(int)points_ij.size())
                cluster_size = (int)points_ij.size(); // default cluster size = vector size

            if (cluster_size < 3) return 0;

            if (!sorted)
                radixSortPoints(points_ij, depth_map.cols, depth_map.rows, cluster_size, &points_xyz);

            std::vector<int> rowStart;

            for (unsigned i = 0; i < cluster_size; ++i) {
                if (!i || points_ij[i].y > points_ij[i - 1].y)
                    rowStart.push_back(i);
            }

            rowStart.push_back(cluster_size);

            double total = 0.0;

            for (uint i = 0; i < rowStart.size() - 2; ++i) {
                int nx = rowStart[i + 1];

                for (uint j = rowStart[i]; j < rowStart[i + 1] - 1; ++j) {
                    uint idx = j - rowStart[i];

                    if (points_ij[nx].x < points_ij[j].x && nx < rowStart[i + 2]) {
                        auto it1 = points_ij.begin() + (nx + 1);
                        auto it2 = points_ij.begin() + (rowStart[i + 2]);
                        nx = std::lower_bound(it1, it2, Point2i(points_ij[j].x, points_ij[nx].y),
                            PointComparer<Point2i>(0, true))
                            - points_ij.begin();
                    }

                    if (j + 1 >= points_xyz.size() || nx + 1 >= points_xyz.size() ||
                        nx >= rowStart[i + 2] || points_ij[nx].x > points_ij[j].x ||
                        points_ij[nx].y - points_ij[j].y > 1) continue;

                    Vec3f quad[4] =
                    { points_xyz[j], points_xyz[j + 1], points_xyz[nx], points_xyz[nx + 1] };

                    if (points_ij[j + 1].y != points_ij[j].y ||
                        points_ij[j + 1].x - points_ij[j].x > 1) quad[1][2] = 0;

                    if (points_ij[nx + 1].y != points_ij[nx].y ||
                        points_ij[nx + 1].x - points_ij[nx].x > 1) quad[3][2] = 0;

                    total += quadrangleArea(quad);
                    ++nx;
                }
            }

            if (isnan(total)) return 0.0;

            return total;
        }

        double surfaceAreaCircle(cv::Mat shape) {
            cv::Size size = shape.size();
            cv::Rect rect(0, 0, size.width, size.height);

            int dr[8] = { 1, 0, -1, 0, 1, -1, 1, -1 };
            int dc[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };

            double surfArea = 0;
            for (int r = 0; r < size.height; r++)
            {
                for (int c = 0; c < size.width; c++)
                {
                    Vec3f point = shape.at<Vec3f>(r, c);
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

                        Vec3f adjPoint = shape.at<Vec3f>(nr, nc);
                        if (adjPoint[2] == 0) {
                            continue;
                        }

                        double dist = euclideanDistance(point, adjPoint);
                        radius = (dist < radius) ? dist : radius;
                    }

                    if (radius != MAX) {
                        surfArea += M_PI * radius * radius;
                    }
                }
            }

            return surfArea;
        }

        double surfaceAreaTriangulate(cv::Mat shape) {
            cv::Size size = shape.size();
            cv::Rect rect(0, 0, size.width, size.height);

            int dr[4] = { 0, 1, 0, -1 };
            int dc[4] = { -1, 0, 1, 0 };

            double surfArea = 0;
            for (int r = 0; r < size.height; r++)
            {
                for (int c = 0; c < size.width; c++)
                {
                    Vec3f point = shape.at<Vec3f>(r, c);
                    if (point[2] == 0) {
                        continue;
                    }

                    Vec3f adj[4];
                    bool validPoint[4] = { false };
                    for (int idx = 0; idx < 4; idx++) {
                        int nr = r + dr[idx];
                        int nc = c + dc[idx];

                        if (nr < 0 || nr >= size.height || nc < 0 || nc >= size.width) {
                            continue;
                        }

                        Vec3f adjPoint = shape.at<Vec3f>(nr, nc);
                        if (adjPoint[2] == 0) {
                            continue;
                        }

                        adj[idx] = adjPoint;
                        validPoint[idx] = true;
                    }

                    if (validPoint[0] && validPoint[1]) {
                        double dist1 = euclideanDistance(point, adj[0]);
                        double dist2 = euclideanDistance(point, adj[1]);
                        double dist3 = euclideanDistance(adj[0], adj[1]);
                        double s = (dist1 + dist2 + dist3) / 2;
                        surfArea += sqrt(abs(s * (s - dist1) * (s - dist2) * (s - dist3)));
                    }

                    if (validPoint[2] && validPoint[3]) {
                        double dist1 = euclideanDistance(point, adj[2]);
                        double dist2 = euclideanDistance(point, adj[3]);
                        double dist3 = euclideanDistance(adj[2], adj[3]);
                        double s = (dist1 + dist2 + dist3) / 2;
                        surfArea += sqrt(abs(s * (s - dist1) * (s - dist2) * (s - dist3)));
                    }
                }
            }

            return surfArea;
        }

        /***
        Check whether candidate point is close enough to neighboring points
        ***/
        static bool closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance)
        {
            auto num_close = 0;
            //check to see if the neighbor pixels Euclidean distance is within defined max distance
            if (x - 1 < 0 || depthMap.at<Vec3f>(y, x - 1)[2] == 0 ||
                euclideanDistance(depthMap.at<Vec3f>(y, x), depthMap.at<Vec3f>(y, x - 1)) < max_distance)
            {
                num_close++;
            }

            if (x + 1 >= depthMap.cols || depthMap.at<Vec3f>(y, x + 1)[2] == 0 ||
                euclideanDistance(depthMap.at<Vec3f>(y, x), depthMap.at<Vec3f>(y, x + 1)) < max_distance)
            {
                num_close++;
            }

            if (y - 1 < 0 || depthMap.at<Vec3f>(y - 1, x)[2] == 0 ||
                euclideanDistance(depthMap.at<Vec3f>(y, x), depthMap.at<Vec3f>(y - 1, x)) < max_distance)
            {
                num_close++;
            }

            if (y + 1 >= depthMap.rows || depthMap.at<Vec3f>(y + 1, x)[2] == 0 ||
                euclideanDistance(depthMap.at<Vec3f>(y, x), depthMap.at<Vec3f>(y + 1, x)) < max_distance)
            {
                num_close++;
            }

            if (num_close >= num_neighbors)
            {
                return true;
            }

            return false;
        }

        /**
         * Performs floodfill on depthMap
         */
        int floodFill(int seed_x, int seed_y, cv::Mat& depthMap,
            std::vector <Point2i> * output_ij_points,
            std::vector <Vec3f> * output_xyz_points,
            double max_distance,
            cv::Mat * mask)
        {
            /*
            Listing of adjacent points to go to in each floodfill step ((6, 0) means to go right 6).
            Goes to 6,0, etc to fill in small gaps
            */
            static const Point2i nxtPoints[] =
            {
                Point2i(-6, 0),  
                Point2i(-1, 0),
                Point2i(0, -6), 
                Point2i(0, -1),
                Point2i(0, 1),
                Point2i(0, 6),
                Point2i(1, 0),
                Point2i(6, 0),   
            };

            static const int nNxtPoints = (sizeof nxtPoints) / (sizeof nxtPoints[0]);

            // stack for storing the 2d and 3d points
            static std::vector<std::pair<Point2i, Vec3f> > stk;

            // permanently allocate memory for our stack
            if (stk.size() <= depthMap.rows * depthMap.cols) {
                stk.resize(depthMap.rows * depthMap.cols + 1);
            }

            // add seed to stack
            Point2i seed = Point2i(seed_x, seed_y);
            stk[0] = std::make_pair(seed, depthMap.at<Vec3f>(seed));

            // pointer to top (first empty index) of stack
            int stkPtr = 1;
            // counts the total number of points visited
            int total = 0;

            // begin DFS
            while (stkPtr > 0) {
                // pop current point from stack
                Point2i pt = stk[--stkPtr].first;
                Vec3f & xyz = stk[stkPtr].second;

                if (!util::pointInImage(depthMap, pt)) continue;
                if (mask) mask->at<Vec3f>(pt) = Vec3f(xyz);
                depthMap.at<Vec3f>(pt)[2] = 0;

                // put point into the output vectors if provided
                if (output_ij_points) (*output_ij_points)[total] = pt;
                if (output_xyz_points) (*output_xyz_points)[total] = xyz;

                // increment the total # of points
                ++total;

                // go to each adjacent point
                for (int i = 0; i < nNxtPoints; ++i) {
                    Point2i adjPt = pt + nxtPoints[i];

                    // stop if outside bound of image
                    if (!util::pointInImage(depthMap, adjPt)) continue;

                    Vec3f & adjXyz = depthMap.at<Vec3f>(adjPt);

                    // stop if already visited
                    if (adjXyz[2] == 0) continue;

                    // compute 3D distance
                    double dist = util::euclideanDistance(xyz, adjXyz);
                    //scaled_dist_thresh = max_distance;

                    // scale distance for 'skip' points
                    if (abs(nxtPoints[i].x + nxtPoints[i].y) != 1)
                        dist /= 4;

                     // update & go to if point is close enough
                    if (dist < max_distance) {
                        stk[stkPtr++] = std::make_pair(adjPt, Vec3f(adjXyz));
                        depthMap.at<Vec3f>(adjPt)[2] = 0;
                    }
                }
            }

            return total;
        }

        // convert an ij point to an angle, clockwise from the bottom (0 at 0 degrees, 2 * PI at 360)
        double pointToAngle(Point2i pt) {
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
        double angleBetweenPoints(Point2i a, Point2i b, Point2i center) {
            a -= center; b -= center;
            double angle = abs(pointToAngle(a) - pointToAngle(b));
            if (angle > PI) return 2 * PI - angle;
            return angle;
        }

        // convert an ij point to a slope, clockwise from the bottom (0 at 0 degrees, FLT_MAX at 360)
        double pointToSlope(Point2i pt) {
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

        template <class T>
        double magnitude(cv::Point_<T> pt) {
            return sqrt(pt.x * pt.x + pt.y * pt.y);
        }

        template <class T>
        double magnitude(cv::Point3_<T> pt) {
            return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        }

        template <class T, int n>
        double magnitude(cv::Vec<T, n> pt) {
            double sm = 0;
            for (int i = 0; i < n; ++i) sm += pt[i] * pt[i];
            return sqrt(sm);
        }

        // instantialize
        template double magnitude<int>(cv::Point_<int> pt);
        template double magnitude<float>(cv::Point_<float> pt);
        template double magnitude<double>(cv::Point_<double> pt);
        template double magnitude<int>(cv::Point3_<int> pt);
        template double magnitude<float>(cv::Point3_<float> pt);
        template double magnitude<double>(cv::Point3_<double> pt);
        template double magnitude<ushort, 3>(cv::Vec<ushort, 3> pt);
        template double magnitude<int, 3>(cv::Vec<int, 3> pt);
        template double magnitude<float, 3>(cv::Vec<float, 3> pt);
        template double magnitude<double, 3>(cv::Vec<double, 3> pt);

        // get angle between two 3D vectors through a central point
        double angleBetween3DVec(Vec3f a, Vec3f b, Vec3f center) {
            a -= center; b -= center;
            cv::Mat A(a), B(b);
            double dot = A.dot(B), mA = magnitude(a), mB = magnitude(b);

            double res = abs(acos(dot / mA / mB));

            if (isnan(res)) return PI;
            else return res;
        }

        bool pointInImage(const cv::Mat & img, const Point2i pt) {
            return pt.x >= 0 && pt.x < img.cols &&
                pt.y >= 0 && pt.y < img.rows;
        }

        bool pointOnEdge(const cv::Size size, const Point2i pt, int margin_tb, int margin_lr) {
            return pt.x <= margin_lr || pt.x >= (size.width - margin_tb) ||
                pt.y <= margin_tb || pt.y >= (size.height - margin_tb);
        }

        bool pointOnEdge(const cv::Rect rect, const Point2i pt, int margin_tb, int margin_lr) {
            return pointOnEdge(rect.size(), pt - rect.tl(), margin_tb, margin_lr);
        }

        bool pointOnEdge(const cv::Mat & img, const Point2i pt, int margin_tb, int margin_lr) {
            return pointOnEdge(img.size(), pt, margin_tb, margin_lr);
        }

        void radixSortPoints(std::vector<Point2i> & points, int wid, int hi, int num_pts,
            std::vector<Vec3f> * points_xyz) {
            if (num_pts < 0 || num_pts >(int)points.size())
                num_pts = (int)points.size();

            static boost::mutex mtx;

            static int * buckets = nullptr, *bucketSize = nullptr;
            static int lastWid, lastHi;

            static Point2i * tmpPoints = nullptr;
            static Vec3f * tmpXyzPoints = nullptr;

            int maxDim = std::max(wid, hi);

            if (buckets == nullptr || lastWid != wid || lastHi != hi) {
                if (buckets != nullptr) delete[] buckets;
                if (bucketSize != nullptr) delete[] bucketSize;

                // permanently allocate memory for buckets, to improve efficiency
                buckets = new int[wid * hi];
                bucketSize = new int[maxDim];

                tmpPoints = new Point2i[wid * hi];
                tmpXyzPoints = new Vec3f[wid * hi];

                lastWid = wid;
                lastHi = hi;
            }

            // clear buckets
            memset(bucketSize, 0, wid * sizeof(int));

            // order by x
            for (int i = 0; i < num_pts; ++i) {
                int idx = points[i].x * hi + bucketSize[points[i].x];

                buckets[idx] = i;
                ++bucketSize[points[i].x];
            }

            int idx = -1;
            for (int i = 0; i < wid; ++i) {
                for (unsigned j = 0; j < bucketSize[i]; ++j) {
                    int k = buckets[i * hi + j];
                    tmpPoints[++idx] = points[k];
                    if (points_xyz)
                        tmpXyzPoints[idx] = (*points_xyz)[k];
                }
            }

            // clear buckets again
            memset(bucketSize, 0, hi * sizeof(int));

            // order by y
            for (int i = 0; i < num_pts; ++i) {
                int idx = tmpPoints[i].y * wid + bucketSize[tmpPoints[i].y];
                buckets[idx] = i;
                ++bucketSize[tmpPoints[i].y];
            }

            idx = -1;
            for (int i = 0; i < hi; ++i) {
                for (unsigned j = 0; j < bucketSize[i]; ++j) {
                    int k = buckets[i * wid + j];
                    points[++idx] = tmpPoints[k];
                    if (points_xyz)
                        (*points_xyz)[idx] = tmpXyzPoints[k];
                }
            }
        }

        Point2i nearestPointOnCluster(const cv::Mat m, Point2i pt, int max_tries) {
            int tries = 0, run = 1, len = 2, direction = 0;

            pt.x = std::min(std::max(0, pt.x), m.cols - 1);
            pt.y = std::min(std::max(0, pt.y), m.rows - 1);

            Vec3f xyz = m.at<Vec3f>(pt);
            Point2i orig_pt = pt;

            // travel in a spiral and find a nearby nonzero point
            while (xyz[2] == 0 && tries < max_tries) {
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

                xyz = m.at<Vec3f>(pt);
            }

            if (tries >= max_tries) return orig_pt;

            return pt;
        }

        Point2f largestInscribedCircle(const std::vector<Point2i> & contour,
            const std::vector<Point2i> & cluster,
            const std::vector<Vec3f> & cluster_xyz,
            int cluster_size, float top_dist_thresh,
            double * radius, int step) {

            if (cluster_size < 0 || cluster_size >(int)cluster.size())
                cluster_size = (int)cluster.size();

            double maxr = 0.0;
            int besti = 0;

            int top_row_pts = 0;
            Vec3f top_avg = Vec3f(0, 0, 0);

            for (unsigned i = 0; i < cluster.size(); ++i) {
                if (i && cluster[i].y != cluster[i - 1].y)
                    break;
                ++top_row_pts;
                top_avg += cluster_xyz[0];
            }

            top_avg /= top_row_pts;

            // find best center
            for (unsigned i = 0; i < cluster_size; i += step) {
                float topDist = euclideanDistance(cluster_xyz[i], top_avg);
                if (topDist > top_dist_thresh) continue;

                double edgeDist = cv::pointPolygonTest(contour, cluster[i], true);
                if (edgeDist > maxr) {
                    maxr = edgeDist;
                    besti = i;
                }
            }

            if (radius) *radius = maxr;
            return cluster[besti];
        }

        double contourCurvature(const std::vector<Point2i> & contour, int index,
            int start, int end) {
            int l = index, r = index;
            int sz = (int)contour.size();
            Point2i center = contour[index];

            for (int i = 0; i < start; ++i) {
                if (--l < 0) l = sz - 1;
                if (++r >= sz) r = 0;
            }

            double result = 0.0;
            for (int i = start; i <= end; ++i) {
                result += angleBetweenPoints(contour[l], contour[r], center);
                if (--l < 0) l = sz - 1;
                if (++r >= sz) r = 0;
            }

            return result / (end - start + 1);
        }

        bool PointComparer<Point2i>::operator()(Point2i a, Point2i b) {
            if (compare_y_then_x) {
                if (a.y == b.y) return reverse ^ (a.x < b.x);
                return reverse ^ (a.y < b.y);
            }
            else {
                if (a.x == b.x) return reverse ^ (a.y < b.y);
                return reverse ^ (a.x < b.x);
            }
        }

        bool PointComparer<Point2f>::operator()(Point2f a, Point2f b) {
            if (compare_y_then_x) {
                if (a.y == b.y) return reverse ^ (a.x < b.x);
                return reverse ^ (a.y < b.y);
            }
            else {
                if (a.x == b.x) return reverse ^ (a.y < b.y);
                return reverse ^ (a.x < b.x);
            }
        }

        bool PointComparer<Vec3f>::operator()(Vec3f a, Vec3f b) {
            for (int i = (compare_y_then_x ? 2 : 0);
                (compare_y_then_x ? i >= 0 : i < 3);
                (compare_y_then_x ? --i : ++i)) {
                if (a[i] == b[i]) continue;

                return reverse ^ (a[i] < b[i]);
            }
            return false;
        }

        bool PointComparer<cv::Vec3i>::operator()(cv::Vec3i a, cv::Vec3i b) {
            for (int i = (compare_y_then_x ? 2 : 0);
                (compare_y_then_x ? i >= 0 : i < 3);
                (compare_y_then_x ? --i : ++i)) {
                if (a[i] == b[i]) continue;

                return reverse ^ (a[i] < b[i]);
            }
            return false;
        }
    }
}
