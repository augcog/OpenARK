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

        Vec3b randomColor()
        {
            return Vec3b(rand() % 256, rand() % 256, rand() % 256);
        }

        Vec3b paletteColor(uchar color_index)
        {
            static const Vec3b palette[] = {
                Vec3b(0, 220, 255), Vec3b(177, 13, 201), Vec3b(94, 255, 34),
                Vec3b(54, 65, 255), Vec3b(64, 255, 255), Vec3b(217, 116, 0),
                Vec3b(27, 133, 255), Vec3b(190, 18, 240), Vec3b(63, 31, 0),
                Vec3b(75, 20, 133), Vec3b(255, 219, 127), Vec3b(204, 204, 57),
                Vec3b(112, 153, 61), Vec3b(64, 204, 46), Vec3b(112, 255, 1),
                Vec3b(170, 170, 170),
            };

            return palette[color_index % (sizeof palette / sizeof palette[0])];
        }

        template<class T>
        std::string pluralize(std::string str, T num)
        {
            if (num == 1) return str;
            return str + 's';
        }

        template std::string pluralize<int>(std::string str, int num);
        template std::string pluralize<unsigned int>(std::string str, unsigned int num);
        template std::string pluralize<long long>(std::string str, long long num);
        template std::string pluralize<size_t>(std::string str, size_t num);
        template std::string pluralize<float>(std::string str, float num);
        template std::string pluralize<double>(std::string str, double num);

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
        T euclideanDistance(cv::Point_<T> pt1, cv::Point_<T> pt2)
        {
            return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
        }

        template int euclideanDistance<int>(cv::Point_<int> pt1, cv::Point_<int> pt2);
        template float euclideanDistance<float>(cv::Point_<float> pt1, cv::Point_<float> pt2);
        template double euclideanDistance<double>(cv::Point_<double> pt1, cv::Point_<double> pt2);

        template<class T>
        T euclideanDistance(const cv::Vec<T, 3> & pt1, const cv::Vec<T, 3> & pt2)
        {
            return sqrt((pt1[0] - pt2[0]) * (pt1[0] - pt2[0]) +
                (pt1[1] - pt2[1]) * (pt1[1] - pt2[1]) +
                (pt1[2] - pt2[2]) * (pt1[2] - pt2[2]));
        }

        template uchar euclideanDistance<uchar>(const cv::Vec<uchar, 3> & pt1, const cv::Vec<uchar, 3> & pt2);
        template int euclideanDistance<int>(const cv::Vec<int, 3> & pt1, const cv::Vec<int, 3> & pt2);
        template float euclideanDistance<float>(const cv::Vec<float, 3> & pt1, const cv::Vec<float, 3> & pt2);
        template double euclideanDistance<double>(const cv::Vec<double, 3> & pt1, const cv::Vec<double, 3> & pt2);

        template<class T>
        T pointPlaneDistance(const cv::Vec<T, 3> & pt, T a, T b, T c)
        {
            return abs(a*pt[0] + b*pt[1] - pt[2] + c) / sqrt(a*a + b*b + 1.0);
        }

        template float pointPlaneDistance<float>(const cv::Vec<float, 3> & pt, float a, float b, float c);
        template double pointPlaneDistance<double>(const cv::Vec<double, 3> & pt, double a, double b, double c);

        template<class T>
        T pointPlaneDistance(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn)
        {
            return abs(eqn[0]*pt[0] + eqn[1]*pt[1] - pt[2] + eqn[2]) /
                   sqrt(eqn[0]*eqn[0] + eqn[1]*eqn[1] + 1.0);
        }

        template float pointPlaneDistance<float>(const cv::Vec<float, 3> & pt, const cv::Vec<float, 3> & eqn);
        template double pointPlaneDistance<double>(const cv::Vec<double, 3> & pt, const cv::Vec<double, 3> & eqn);

        template<class T>
        T pointPlaneNorm(const cv::Vec<T, 3> & pt, T a, T b, T c)
        {
            T alpha = (a*pt[0] + b*pt[1] - pt[2] + c);
            return alpha * alpha / (a*a + b*b + 1.0);
        }

        template float pointPlaneNorm<float>(const cv::Vec<float, 3> & pt, float a, float b, float c);
        template double pointPlaneNorm<double>(const cv::Vec<double, 3> & pt, double a, double b, double c);

        template<class T>
        T pointPlaneNorm(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn)
        {
            T alpha = eqn[0]*pt[0] + eqn[1]*pt[1] - pt[2] + eqn[2];
            return alpha * alpha / (eqn[0]*eqn[0] + eqn[1]*eqn[1] + 1.0);
        }

        template float pointPlaneNorm<float>(const cv::Vec<float, 3> & pt, const cv::Vec<float, 3> & eqn);
        template double pointPlaneNorm<double>(const cv::Vec<double, 3> & pt, const cv::Vec<double, 3> & eqn);

        template<class T, int N>
        cv::Vec<T, N> linearRegression(const std::vector<cv::Vec<T, N>> & points, int num_points)
        {
            if (num_points == -1) num_points = (int)points.size();

            typedef Eigen::Matrix<T, -1, -1> MatT;
            MatT A(points.size(), N), b(num_points, 1);

            for (int i = 0; i < num_points; ++i) {
                for (int j = 0; j < N - 1; ++j) {
                    A(i, j) = points[i][j];
                }
                A(i, N-1) = 1.0;
                b(i) = points[i][N - 1];
            }

            MatT LLSE = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

            cv::Vec<T, N> output;
            for (int i = 0; i < N; ++i) {
                output[i] = LLSE(i);
            }

            return output;
        }

        template cv::Vec<float, 3> linearRegression<float, 3>(const std::vector<cv::Vec<float, 3> > & points, int num_points);
        template cv::Vec<double, 3> linearRegression<double, 3>(const std::vector<cv::Vec<double, 3> > & points, int num_points);
        template cv::Vec<float, 2> linearRegression<float, 2>(const std::vector<cv::Vec<float, 2> > & points, int num_points);
        template cv::Vec<double, 2> linearRegression<double, 2>(const std::vector<cv::Vec<double, 2> > & points, int num_points);

        template<class T>
        cv::Vec<T, 3> ransacFindPlane(const std::vector<cv::Vec<T, 3>>& points,
                                       T thresh, int iterations, int num_points)
        {
            cv::Vec<T, 3> best; int bestInliers = 0;
            
            // number of points in each sample
            static const int SAMPLE_SIZE = 3;

            if (num_points == -1) num_points = (int) points.size();
            if (num_points < SAMPLE_SIZE) throw; // too few points

            // create RNG
            typedef boost::mt19937 rng_type;
            typedef boost::variate_generator<rng_type, boost::uniform_int<> > var_gen;

            rng_type rng;
            std::vector<boost::uniform_int<>> range;
            std::vector<var_gen> randidx;

            for (int i = 0; i < SAMPLE_SIZE; ++i) {
                range.emplace_back(0, num_points - i - 1);
                randidx.emplace_back(rng, range[i]);
            }

            // Begin iterating
            for (int iteration = 0; iteration < iterations; ++iteration) {
                // pick 3 random points
                int idx[SAMPLE_SIZE];
                for (int i = 0; i < SAMPLE_SIZE; ++i) {
                    idx[i] = randidx[i]();
                    for (int j = 0; j < i; ++j) {
                        if (idx[i] >= idx[j]) ++idx[i];
                    }
                }
                const cv::Vec<T, 3> & a = points[idx[0]], 
                    & b = points[idx[1]], & c = points[idx[2]];

                // compute equation of plane through these points
                cv::Vec<T, 3> normal = cv::normalize((b - a).cross(c - a));
                if (normal[2] > 0) normal = -normal;

                T k = (normal[0] * a[0] + normal[1] * a[1]) / normal[2] + a[2];
                cv::Vec<T, 3> eqn(-normal[0] / normal[2], -normal[1] / normal[2], k);

                // compute number of inliers
                int inliers = 0;
                for (int i = 0; i < num_points; ++i) {
                    T norm = util::pointPlaneNorm(points[i], eqn);
                    if (norm < thresh) ++inliers;
                }

                if (inliers > bestInliers) {
                    best = eqn;
                    bestInliers = inliers;
                }
            }

            return best;
        }

        template cv::Vec<float, 3> ransacFindPlane<float>(const std::vector<cv::Vec<float, 3>>& points,
                        float thresh, int iterations, int num_points);
        template cv::Vec<double, 3> ransacFindPlane<double>(const std::vector<cv::Vec<double, 3>>& points,
                        double thresh, int iterations, int num_points);

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

        void removePoints(cv::Mat & img, const std::vector<Point2i> & points)
        {
            for (int i = 0; i < points.size(); i++)
            {
                img.at<Vec3f>(points[i]) = 0;
            }
        }

        void removePlane(cv::Mat & xyz_map, const Vec3f & plane_equation,
            float threshold, cv::Mat * mask, uchar mask_color)
        {
            for (int row = 0; row < xyz_map.rows; ++row) {
                Vec3f * ptr = xyz_map.ptr<Vec3f>(row);
                uchar * maskPtr;
                if (mask != nullptr) maskPtr = mask->ptr<uchar>(row);

                for (int col = 0; col < xyz_map.cols; ++col) {
                    if (mask != nullptr && maskPtr[col] != mask_color) {
                        continue;
                    }

                    Vec3f & xyz = ptr[col];
                    if (pointPlaneNorm(xyz, plane_equation) < threshold) {
                        // found nearby plane, remove point (i.e. set to 0)
                        xyz = 0;
                    }
                }
            }
        }

        Vec3f averageAroundPoint(cv::Mat xyz_map, Point2i pt, int radius)
        {
            int count = 0;
            Vec3f average(0, 0, 0);

            for (int r = std::max(0, pt.y - radius); r <= pt.y + radius; ++r)
            {
                if (r >= xyz_map.rows) break;
               
                Vec3f * ptr = xyz_map.ptr<Vec3f>(r);

                for (int c = std::max(0, pt.x - radius); c <= pt.x + radius; ++c)
                {
                    if (c >= xyz_map.cols) break;

                    if (ptr[c][2] > 0)
                    {
                        // add to total
                        average += ptr[c];
                        ++count;
                    }
                }
            }

            if (count == 0) return 0;
            return average / count;
        }

        Vec3f normalAroundPoint(cv::Mat xyz_map, Point2i pt, int radius)
        {
            Vec3f center = xyz_map.at<Vec3f>(pt);
            if (center[2] == 0) return 0;

            bool empty = true;
            Vec3f average(0, 0, 0);

            for (int r = -radius; r <= radius; r += radius)
            {
                int rr = r + pt.y;
                if (rr < 0 || rr >= xyz_map.rows) continue;
               
                Vec3f * ptr = xyz_map.ptr<Vec3f>(rr);

                for (int c = -radius; c <= radius; c += radius)
                {
                    int cc = c + pt.x;
                    if ((!r && !c) || cc < 0 || cc >= xyz_map.cols) continue;

                    Vec3f a = ptr[cc], b;
                    if (a[2] <= 0) continue;

                    // pick second point ninety degrees CCW
                    int r2 = pt.y + c, c2 = pt.x - r;
                    if (r2 < 0 || r2 >= xyz_map.rows || c2 < 0 || c2 >= xyz_map.cols ||
                        (b = xyz_map.at<Vec3f>(r2, c2))[2] == 0) {
                        // if out of bounds, try ninety degrees CW

                        r2 = pt.y - c; c2 = pt.x + r;

                        // if still doesn't work, don't use this point
                        if (r2 < 0 || r2 >= xyz_map.rows || c2 < 0 || c2 >= xyz_map.cols || 
                            (b = xyz_map.at<Vec3f>(r2, c2))[2] == 0) 
                            continue;
                    }

                    // take cross product of the vectors
                    Vec3f cross = (a - center).cross(b - center);

                    // orient towards viewer
                    if (cross[2] > 0) cross = -cross;

                    // add to total
                    average += cross;
                    empty = false;
                }
            }

            if (empty) return 0;

            // divide to get average normal, then normalize
            average = cv::normalize(average);

            return average;
        }

        int removeOutliers(const std::vector<Vec3f> & data, std::vector<Vec3f>& output, 
                               double thresh, 
                               const std::vector<Point2i> * data_aux,
                               std::vector<Point2i> * output_aux, int num_data_pts)
        {
            if (num_data_pts == -1) num_data_pts = (int)data.size();
            const int NUM_OUTPUT_PTS = num_data_pts * (1.0 - thresh);

            Vec3f totalPos(0, 0, 0), avgPos;
            for (int i = 0; i < num_data_pts; ++i) {
                totalPos += data[i];
            }

            avgPos = totalPos / num_data_pts;

            // compute influence
            std::vector<float> influence(num_data_pts);
            for (int i = 0; i < num_data_pts; ++i) {
                influence[i] =
                    norm(avgPos - (totalPos - data[i]) / (num_data_pts - 1));
            }

            std::vector<float> influenceOrder = influence;

            // order by influence
            std::sort(influenceOrder.begin(), influenceOrder.end());

            output.resize(NUM_OUTPUT_PTS);
            if (output_aux) output_aux->resize(NUM_OUTPUT_PTS);

            // take elements with least influence
            for (int i = 0; i < num_data_pts; ++i) {
                int idx = std::lower_bound(influenceOrder.begin(), influenceOrder.end(),
                    influence[i]) - influenceOrder.begin();

                if (idx >= NUM_OUTPUT_PTS) continue;
                output[idx] = data[i]; 
                if (data_aux && output_aux) (*output_aux)[idx] = (*data_aux)[i];
            }

            return NUM_OUTPUT_PTS;
        }

        void computeNormalMap(const cv::Mat & xyz_map, cv::Mat & output_mat, 
            int normal_dist, int resolution, bool fill_in)
        {
            if (fill_in) {
                output_mat.create(xyz_map.size() / resolution, CV_32FC3);
            }
            else {
                output_mat.create(xyz_map.size(), CV_32FC3);
            }

            int step = fill_in ? 1 : resolution;

            Vec3f * curRow;
            for (int i = 0; i < output_mat.rows; i += step) {
                curRow = output_mat.ptr<Vec3f>(i);

                for (int j = 0; j < output_mat.cols; j += step) {
                    curRow[j] = util::normalAroundPoint(xyz_map,
                        Point2i(j, i) * (resolution / step), normal_dist);
                }
            }

            if (fill_in) {
                cv::resize(output_mat, output_mat, xyz_map.size());
            }
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

        double surfaceArea(const cv::Mat & depthMap)
        {
            if (depthMap.rows == 0 || depthMap.cols == 0) return 0.0;

            double total = 0.0;

            const Vec3f * ptr, *nxPtr = depthMap.ptr<Vec3f>(0);

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

        double surfaceArea(const cv::Size & frame_size,
            const std::vector<Point2i> & points_ij,
            const std::vector<Vec3f> & points_xyz,
            int cluster_size) {

            if (cluster_size < 0 || cluster_size >(int)points_ij.size())
                cluster_size = (int)points_ij.size(); // default cluster size = vector size

            if (cluster_size < 3) return 0;

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

                    if (i + 2 < rowStart.size() && nx >= rowStart[i + 2]) continue;

                    if (points_ij[nx].x < points_ij[j].x){
                        auto it1 = points_ij.begin() + (nx + 1);
                        auto it2 = points_ij.begin();

                        if (i + 2 < rowStart.size()) it2 += rowStart[i + 2];
                        else it2 = points_ij.end();

                        nx = std::lower_bound(it1, it2, Point2i(points_ij[j].x, points_ij[nx].y),
                            PointComparer<Point2i>(0, true))
                            - points_ij.begin();
                    }

                    if (j + 1 >= points_xyz.size() || nx + 1 >= points_xyz.size()) continue;

                    Vec3f quad[4] =
                    { points_xyz[j], points_xyz[j + 1], points_xyz[nx], points_xyz[nx + 1] };

                    if (points_ij[j + 1].y != points_ij[j].y) quad[1][2] = 0;

                    if (points_ij[nx + 1].y != points_ij[nx].y) quad[3][2] = 0;

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

        double diameter(const std::vector<cv::Point>& points, int & a, int & b)
        {
            double maxd = 0.0;
            a = b = 0;

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < (int)points.size(); ++j) {
                    double curd = ark::util::norm(points[j] - points[a]);
                    if (curd > maxd) {
                        b = j;
                        maxd = curd;
                    }
                }

                if (i == 0) std::swap(a, b);
            }

            return maxd;
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
         * Performs floodfill on ordered point cloud
         */
        int floodFill(const cv::Mat & xyz_map, const Point2i & seed,
            float max_distance, std::vector <Point2i> * output_ij_points,
            std::vector <Vec3f> * output_xyz_points, cv::Mat * output_mask,
            int interval, int interval2, const cv::Mat * access_mask, uchar access_mask_color, 
            cv::Mat * not_visited)
        {
            // if access mask disallows seed, stop
            if (access_mask && access_mask->at<uchar>(seed) != access_mask_color) return 0;

            /*
            Listing of adjacent points to go to in each floodfill step
            */
            std::vector<Point2i> nxtPoints
            {
                Point2i(-interval, 0), Point2i(0, -interval),
                Point2i(0, interval), Point2i(interval, 0),
            };

            if (interval2 > 0) {
                nxtPoints.push_back(Point2i(-interval2, 0));
                nxtPoints.push_back(Point2i(0, -interval2));
                nxtPoints.push_back(Point2i(0, interval2));
                nxtPoints.push_back(Point2i(interval2, 0));
            }
            
            // true if temporary 'visited' matrix allocated (we'll need to delete it after)
            bool tempVisMat = (not_visited == nullptr);

            // create 'visited' matrix
            if (tempVisMat) {
                not_visited = new cv::Mat(xyz_map.size(), CV_8U);
                *not_visited = cv::Scalar(1);
            }

            // stack for storing the 2d and 3d points
            static std::vector<std::pair<Point2i, Vec3f> > stk;

            const int R = xyz_map.rows, C = xyz_map.cols;

            // permanently allocate memory for our stack
            if (stk.size() < R * C) {
                stk.resize(R * C);
            }

            // use square of distance to save computations
            max_distance *= max_distance;


            // add seed to stack
            stk[0] = std::make_pair(seed, xyz_map.at<Vec3f>(seed));

            // pointer to top (first empty index) of stack
            int stkPtr = 1;
            // counts the total number of points visited
            int total = 0;

            // begin DFS
            while (stkPtr > 0) {
                // pop current point from stack
                Point2i pt = stk[--stkPtr].first;
                Vec3f & xyz = stk[stkPtr].second;

                if (!util::pointInImage(xyz_map, pt)) continue;
                if (output_mask) output_mask->at<Vec3f>(pt) = Vec3f(xyz);
                not_visited->at<Vec3f>(pt)[2] = 0;

                // put point into the output vectors if provided
                if (output_ij_points) {
                    if (output_ij_points->size() <= total) {
                        output_ij_points->push_back(pt);
                    }
                    else {
                        (*output_ij_points)[total] = pt;
                    }
                }
                if (output_xyz_points) {
                    (*output_xyz_points)[total] = Vec3f(xyz);
                }

                // increment the total # of points
                ++total;

                // go to each adjacent point
                for (uint i = 0; i < nxtPoints.size(); ++i) {
                    Point2i adjPt = pt + nxtPoints[i];

                    // stop if outside bound of image
                    if (!util::pointInImage(xyz_map, adjPt)) continue;

                    // stop if access mask tells us not to go there
                    if (access_mask && access_mask->at<uchar>(adjPt) != access_mask_color) continue;

                    // stop if already visited
                    bool is_visited;
                    if (not_visited->type() == CV_32FC3) {
                        is_visited = not_visited->at<Vec3f>(adjPt)[2] == 0;
                    }
                    else {
                        is_visited = not_visited->at<uchar>(adjPt) == 0;
                    }
                    if (is_visited) continue;

                    const Vec3f & adjXyz = xyz_map.at<Vec3f>(adjPt);

                    // compute 3D norm
                    double norm = util::norm(xyz - adjXyz);

                    // scale distance for 'skip' points
                    if (abs(nxtPoints[i].x + nxtPoints[i].y) > interval)
                        norm /= 25;

                    // update & go to if point is close enough
                    if (norm < max_distance) {
                        stk[stkPtr++] = std::make_pair(adjPt, Vec3f(adjXyz));

                        if (not_visited->type() == CV_32FC3) {
                            not_visited->at<Vec3f>(adjPt)[2] = 0;
                        }
                        else {
                            not_visited->at<uchar>(adjPt) = 0;
                        }
                    }
                }
            }

            if (tempVisMat) {
                delete not_visited;
                not_visited = nullptr;
            }

            return total;
        }

        // convert an ij point to an angle, clockwise from (0, 1) (0 at 0 degrees, 2 * PI at 360)
        double pointToAngle(const Point2f & pt) {
            return fmod(atan2(pt.x, -pt.y) + PI, 2 * PI);
        }

        // get angle between two points through a central point
        double angleBetweenPoints(const Point2f & a, const Point2f & b, const Point2f & center) {
            double angle = abs(pointToAngle(a - center) - pointToAngle(b - center));
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

        // instantialize for different types
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

        template <class T>
        double norm(cv::Point_<T> pt) {
            return pt.x * pt.x + pt.y * pt.y;
        }

        template <class T>
        double norm(cv::Point3_<T> pt) {
            return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        }

        template <class T, int n>
        double norm(cv::Vec<T, n> pt) {
            double sm = 0;
            for (int i = 0; i < n; ++i) sm += pt[i] * pt[i];
            return sm;
        }

        // instantialize
        template double norm<int>(cv::Point_<int> pt);
        template double norm<float>(cv::Point_<float> pt);
        template double norm<double>(cv::Point_<double> pt);
        template double norm<int>(cv::Point3_<int> pt);
        template double norm<float>(cv::Point3_<float> pt);
        template double norm<double>(cv::Point3_<double> pt);
        template double norm<ushort, 3>(cv::Vec<ushort, 3> pt);
        template double norm<int, 3>(cv::Vec<int, 3> pt);
        template double norm<float, 3>(cv::Vec<float, 3> pt);
        template double norm<double, 3>(cv::Vec<double, 3> pt);

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
            return pt.x >= 0 && pt.x < img.cols && pt.y >= 0 && pt.y < img.rows;
        }

        bool pointInRect(const cv::Rect & rect, const Point2i pt)
        {
            return pt.x >= rect.x && pt.x < rect.x + rect.width &&
                   pt.y >= rect.y && pt.y < rect.y + rect.height;
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
            const cv::Mat & xyz_map, const cv::Rect bounds, const Vec3f top_point, float top_dist_thresh,
            double * radius, int samples) { 

            // step 1: construct Voronoi diagram of (down-sampled) contour points
            typedef boost::polygon::point_data<int> xpoint;
            std::vector<xpoint> xpoints;

            int contour_step = std::max(1, (int)contour.size() / 100);
            xpoints.resize(contour.size() / contour_step);

            for (int i = 0; i < xpoints.size(); ++i) {
                xpoints[i] = xpoint(contour[i * contour_step].x, contour[i * contour_step].y);
            }

            boost::polygon::voronoi_diagram<double> vd;
            boost::polygon::construct_voronoi(xpoints.begin(), xpoints.end(), &vd);

            double maxr = 0.0;
            cv::Point bestpt = 0;

            // step 2: go through vertices in the diagram and find the center 
            // of the largest circle (must be one of the vertices)
            const std::vector<boost::polygon::voronoi_vertex<double>> & xverts = vd.vertices();
            for (unsigned i = 0; i < xverts.size(); ++i) {
                // bad vertex
                if (xverts[i].is_degenerate() || xverts[i].incident_edge()->is_infinite())
                    continue;

                Point2i vert (xverts[i].x(), xverts[i].y());
                // check in bounds
                if (!pointInRect(bounds, vert)) continue;

                // check distance to top
                Vec3f vec = xyz_map.at<Vec3f>(vert - bounds.tl());
                float dist = util::euclideanDistance(vec, top_point);
                if (dist > top_dist_thresh) continue;

                Point2i pt0 (xverts[i].incident_edge()->vertex0()->x(),
                    xverts[i].incident_edge()->vertex0()->y());
                Point2i pt1 (xverts[i].incident_edge()->vertex1()->x(),
                    xverts[i].incident_edge()->vertex1()->y());

                // check incidence line in bounds
                if (!pointInRect(bounds, pt0) || !pointInRect(bounds, pt1)) continue;

                // update best circle found so far
                double edgeDist = cv::pointPolygonTest(contour, vert, true);
                if (edgeDist > maxr) {
                    maxr = edgeDist;
                    bestpt = vert;
                }
            }

            // return result
            if (radius) *radius = maxr;
            return bestpt;
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
