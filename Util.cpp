#include "stdafx.h"
#include "Version.h"
#include "Util.h"

namespace ark {

    namespace util {
        std::vector<std::string> split(const std::string & string_in, char const * delimiters,
            bool ignore_empty, bool trim) {
            char * buffer = new char[string_in.size() + 1];
            strcpy(buffer, string_in.c_str());
            std::vector<std::string> output;
            for (char * token = strtok(buffer, delimiters);
                token != NULL; token = strtok(NULL, delimiters))
            {
                output.emplace_back(token);
                util::trim(*output.rbegin());
                if (ignore_empty && output.rbegin()->empty()) output.pop_back();
            }
            delete[] buffer;
            return output;
        }

        std::vector<std::string> split(const char * string_in, char const * delimiters,
            bool ignore_empty, bool trim) {
            return split(std::string(string_in), delimiters, ignore_empty, trim);
        }

        // trim from start (in place)
        void ltrim(std::string & s) {
            s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
                return !std::isspace(ch);
            }));
        }

        // trim from end (in place)
        void rtrim(std::string & s) {
            s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
                return !std::isspace(ch);
            }).base(), s.end());
        }

        // trim from both ends (in place)
        void trim(std::string & s) {
            ltrim(s);
            rtrim(s);
        }

        void upper(std::string & s)
        {
            for (size_t i = 0; i < s.size(); ++i)
                s[i] = std::toupper(s[i]);
        }

        void lower(std::string & s)
        {
            for (size_t i = 0; i < s.size(); ++i)
                s[i] = std::tolower(s[i]);
        }

        Vec3b randomColor()
        {
            return Vec3b(rand() % 256, rand() % 256, rand() % 256);
        }

        Vec3b paletteColor(int color_index, bool bgr)
        {
            static const Vec3b palette[] = {
                Vec3b(0, 220, 255), Vec3b(177, 13, 201), Vec3b(94, 255, 34),
                Vec3b(54, 65, 255), Vec3b(64, 255, 255), Vec3b(217, 116, 0),
                Vec3b(27, 133, 255), Vec3b(190, 18, 240), Vec3b(20, 31, 210),
                Vec3b(75, 20, 133), Vec3b(255, 219, 127), Vec3b(204, 204, 57),
                Vec3b(112, 153, 61), Vec3b(64, 204, 46), Vec3b(112, 255, 1),
                Vec3b(170, 170, 170), Vec3b(225, 30, 42)
            };

            Vec3b color = palette[color_index % (int)(sizeof palette / sizeof palette[0])];
            return bgr ? color : Vec3b(color[2], color[1], color[0]);
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
        float euclideanDistance(const cv::Point_<T> & pt1, const cv::Point_<T> & pt2)
        {
            return sqrtf((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
        }

        template float euclideanDistance<int>(const cv::Point_<int> & pt1, const cv::Point_<int> & pt2);
        template float euclideanDistance<float>(const cv::Point_<float> & pt1, const cv::Point_<float> & pt2);
        template float euclideanDistance<double>(const cv::Point_<double> & pt1, const cv::Point_<double> & pt2);

        template<class T>
        T euclideanDistance(const cv::Vec<T, 3> & pt1, const cv::Vec<T, 3> & pt2)
        {
            return sqrtf((pt1[0] - pt2[0]) * (pt1[0] - pt2[0]) +
                (pt1[1] - pt2[1]) * (pt1[1] - pt2[1]) +
                (pt1[2] - pt2[2]) * (pt1[2] - pt2[2]));
        }

        template uchar euclideanDistance<uchar>(const cv::Vec<uchar, 3> & pt1, const cv::Vec<uchar, 3> & pt2);
        template int euclideanDistance<int>(const cv::Vec<int, 3> & pt1, const cv::Vec<int, 3> & pt2);
        template float euclideanDistance<float>(const cv::Vec<float, 3> & pt1, const cv::Vec<float, 3> & pt2);
        template double euclideanDistance<double>(const cv::Vec<double, 3> & pt1, const cv::Vec<double, 3> & pt2);

        template<class Param_T>
        float pointLineNorm(const cv::Point_<Param_T> & p, const cv::Point_<Param_T> & a, const cv::Point_<Param_T> & b, int cv_norm_type)
        {
            cv::Point_<Param_T> ap = a - p, ab = b - a;
            return util::norm(ap - (ap.dot(ab) / ab.dot(ab)) * ab, cv_norm_type);
        }
        template float pointLineNorm<int>(const cv::Point_<int> & p, const cv::Point_<int> & a, const cv::Point_<int> & b, int cv_norm_type);
        template float pointLineNorm<float>(const cv::Point_<float> & p, const cv::Point_<float> & a, const cv::Point_<float> & b, int cv_norm_type);
        template float pointLineNorm<double>(const cv::Point_<double> & p, const cv::Point_<double> & a, const cv::Point_<double> & b, int cv_norm_type);

        template<class Param_T>
        Param_T pointLineNorm(const cv::Vec<Param_T, 3> & p, const cv::Vec<Param_T, 3> & a, const cv::Vec<Param_T, 3> & b, int cv_norm_type)
        {
            cv::Vec<Param_T, 3> ap = p - a, ab = b - a;
            return cv::norm(ap - (ap.dot(ab) / ab.dot(ab)) * ab, cv_norm_type);
        }
        template uchar pointLineNorm<uchar>(const cv::Vec<uchar, 3> & p, const cv::Vec<uchar, 3> & a, const cv::Vec<uchar, 3> & b, int cv_norm_type);
        template int pointLineNorm<int>(const cv::Vec<int, 3> & p, const cv::Vec<int, 3> & a, const cv::Vec<int, 3> & b, int cv_norm_type);
        template float pointLineNorm<float>(const cv::Vec<float, 3> & p, const cv::Vec<float, 3> & a, const cv::Vec<float, 3> & b, int cv_norm_type);
        template double pointLineNorm<double>(const cv::Vec<double, 3> & p, const cv::Vec<double, 3> & a, const cv::Vec<double, 3> & b, int cv_norm_type);

        template<class Param_T>
        float pointLineSegmentNorm(const cv::Point_<Param_T>& p, const cv::Point_<Param_T>& a, const cv::Point_<Param_T>& b, int cv_norm_type)
        {
            const cv::Point_<Param_T> ab = b - a, ap = p - a;
            const Param_T l2 = util::norm(ab, cv_norm_type);
            if (l2 == (Param_T)0) return util::norm(ap, cv_norm_type);
            float t = std::max<Param_T>(0, std::min<Param_T>(1, ap.dot(ab) / l2));
            return util::norm(ap - t * ab, cv_norm_type);
        }
        template float pointLineSegmentNorm<int>(const cv::Point_<int> & p, const cv::Point_<int> & a, const cv::Point_<int> & b, int cv_norm_type);
        template float pointLineSegmentNorm<float>(const cv::Point_<float> & p, const cv::Point_<float> & a, const cv::Point_<float> & b, int cv_norm_type);
        template float pointLineSegmentNorm<double>(const cv::Point_<double> & p, const cv::Point_<double> & a, const cv::Point_<double> & b, int cv_norm_type);

        template<class Param_T>
        Param_T pointLineSegmentNorm(const cv::Vec<Param_T, 3>& p, const cv::Vec<Param_T, 3>& a, const cv::Vec<Param_T, 3> & b, int cv_norm_type)
        {
            const cv::Vec<Param_T, 3> ab = b - a, ap = p - a;
            const Param_T l2 = cv::norm(ab, cv_norm_type);
            if (l2 == (Param_T)0) return cv::norm(ap, cv_norm_type);
            Param_T t = std::max<Param_T>(0, std::min<Param_T>(1, ap.dot(ab) / l2));
            return cv::norm(ap - t * ab, cv_norm_type);
        }
        template uchar pointLineSegmentNorm<uchar>(const cv::Vec<uchar, 3> & p, const cv::Vec<uchar, 3> & a, const cv::Vec<uchar, 3> & b, int cv_norm_type);
        template int pointLineSegmentNorm<int>(const cv::Vec<int, 3> & p, const cv::Vec<int, 3> & a, const cv::Vec<int, 3> & b, int cv_norm_type);
        template float pointLineSegmentNorm<float>(const cv::Vec<float, 3> & p, const cv::Vec<float, 3> & a, const cv::Vec<float, 3> & b, int cv_norm_type);
        template double pointLineSegmentNorm<double>(const cv::Vec<double, 3> & p, const cv::Vec<double, 3> & a, const cv::Vec<double, 3> & b, int cv_norm_type);

        template<class T>
        T pointPlaneDistance(const cv::Vec<T, 3> & pt, T a, T b, T c)
        {
            return fabs(a*pt[0] + b*pt[1] - pt[2] + c) / sqrt(a*a + b*b + 1.0);
        }

        template float pointPlaneDistance<float>(const cv::Vec<float, 3> & pt, float a, float b, float c);
        template double pointPlaneDistance<double>(const cv::Vec<double, 3> & pt, double a, double b, double c);

        template<class T>
        T pointPlaneDistance(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn)
        {
            return fabs(eqn[0] * pt[0] + eqn[1] * pt[1] - pt[2] + eqn[2]) /
                sqrt(eqn[0] * eqn[0] + eqn[1] * eqn[1] + 1.0);
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
            T alpha = eqn[0] * pt[0] + eqn[1] * pt[1] - pt[2] + eqn[2];
            return alpha * alpha / (eqn[0] * eqn[0] + eqn[1] * eqn[1] + 1.0);
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
                A(i, N - 1) = 1.0;
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

            if (num_points == -1) num_points = (int)points.size();
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
                    &b = points[idx[1]], &c = points[idx[2]];

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

        template <class T>
        void removePlane(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation,
            float threshold, cv::Mat * mask, uchar mask_color)
        {
            const Vec3f * refPtr;
            T * imgPtr;
            uchar * maskPtr;

            for (int row = 0; row < ref_cloud.rows; ++row) {
                refPtr = ref_cloud.ptr<Vec3f>(row);
                imgPtr = image.ptr<T>(row);
                if (mask) maskPtr = mask->ptr<uchar>(row);

                for (int col = 0; col < ref_cloud.cols; ++col) {
                    if (mask && maskPtr[col] != mask_color) {
                        continue;
                    }

                    //std::cout << (refPtr[col], plane_equation) << endl;
                    if (pointPlaneNorm(refPtr[col], plane_equation) < threshold) {
                        // found nearby plane, remove point (i.e. set to 0)
                        imgPtr[col] = 0;
                    }
                }
            }
        }

        template void removePlane<uchar>(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation, float threshold, cv::Mat * mask, uchar mask_color);
        template void removePlane<ushort>(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation, float threshold, cv::Mat * mask, uchar mask_color);
        template void removePlane<uint>(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation, float threshold, cv::Mat * mask, uchar mask_color);
        template void removePlane<int>(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation, float threshold, cv::Mat * mask, uchar mask_color);
        template void removePlane<float>(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation, float threshold, cv::Mat * mask, uchar mask_color);
        template void removePlane<Vec3f>(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation, float threshold, cv::Mat * mask, uchar mask_color);

        Vec3f averageAroundPoint(const cv::Mat & xyz_map, const Point2i & pt, int radius)
        {
            const int T = std::max(0, pt.y - radius), B = std::min(xyz_map.rows - 1, pt.y + radius);
            if (T >= B) return 0;

            const int L = std::max(0, pt.x - radius), R = std::min(xyz_map.cols - 1, pt.x + radius);
            if (L >= R) return 0;

            int total = 0;
            Vec3f avg(0.0f, 0.0f, 0.0f);

            const Vec3f * ptr;
            for (int r = T; r < B; ++r) {
                ptr = xyz_map.ptr<Vec3f>(r);
                for (int c = L; c < R; ++c) {
                    if (ptr[c][2] > 0) {
                        ++total;
                        avg += ptr[c];
                    }
                }
            }

            return avg / total;
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
            std::vector<int> influenceOrder(num_data_pts);

            for (int i = 0; i < num_data_pts; ++i) {
                influence[i] = norm(avgPos - (totalPos - data[i]) / (num_data_pts - 1));
                influenceOrder[i] = i;
            }

            // order by influence
            std::sort(influenceOrder.begin(), influenceOrder.end(),
                [& influence](int a, int b) {
                    return influence[a] < influence[b];
                }
            );

            output.resize(NUM_OUTPUT_PTS);
            if (output_aux) output_aux->resize(NUM_OUTPUT_PTS);

            // take elements with least influence
            for (int i = 0; i < NUM_OUTPUT_PTS; ++i) {
                int idx = influenceOrder[i];
                output[i] = data[idx];
                if (data_aux && output_aux) (*output_aux)[i] = (*data_aux)[idx];
            }

            return NUM_OUTPUT_PTS;
        }

        Vec3f normalAtPoint(const cv::Mat & xyz_map, const Point2i & pt, int radius)
        {
            const Vec3f & center = xyz_map.at<Vec3f>(pt);
            if (center[2] == 0) return 0;

            int xr = (pt.x < radius) ? radius : -radius;
            int yr = (pt.y < radius) ? radius : -radius;

            return normalize((xyz_map.at<Vec3f>(pt.y + yr, pt.x) - center).cross(
                              xyz_map.at<Vec3f>(pt.y, pt.x + xr) - center));
        }

        void computeNormalMap(const cv::Mat & xyz_map, cv::Mat & output_mat,
            int normal_dist, int resolution, bool fill_in)
        {
            cv::Size stripes = xyz_map.size() / resolution;

            if (fill_in) {
                output_mat.create(stripes, CV_32FC3);
            }
            else {
                output_mat.create(xyz_map.size(), CV_32FC3);
            }

            const int R = stripes.height, C = stripes.width;
            int step = fill_in ? 1 : resolution;
            int multiplier = fill_in ? 1 : resolution;

            cv::parallel_for_(cv::Range(0, R*C), [&](const cv::Range & range) {
                for (int r = range.start; r < range.end; ++r) {
                    int i = r / C * multiplier, j = r % C * multiplier;
                    output_mat.ptr<Vec3f>(i)[j] =
                        util::normalAtPoint(xyz_map,
                            Point2i(j, i) * (resolution / step), normal_dist);
                }
            });

            if (fill_in) {
                cv::resize(output_mat, output_mat, xyz_map.size());
            }
        }

        double averageDepth(cv::Mat xyzMap) {
            cv::Mat depth; cv::extractChannel(xyzMap, depth, 2);
            return cv::mean(depth, depth > 0.0f)[0];
        }

        Point2i findCentroid(cv::Mat xyzMap)
        {
            cv::Mat depth;
            cv::extractChannel(xyzMap, depth, 2);
            //using image moments to find center of mass of the depth image
            auto m = cv::moments(depth, false);
            //Cx=M10/M00 and Cy=M01/M00
            return Point2i(m.m10 / m.m00, m.m01 / m.m00);
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

                    if (points_ij[nx].x < points_ij[j].x) {
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

            if (std::isnan(total)) return 0.0;

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
                        surfArea += sqrtf(fabs(s * (s - dist1) * (s - dist2) * (s - dist3)));
                    }

                    if (validPoint[2] && validPoint[3]) {
                        double dist1 = euclideanDistance(point, adj[2]);
                        double dist2 = euclideanDistance(point, adj[3]);
                        double dist3 = euclideanDistance(adj[2], adj[3]);
                        double s = (dist1 + dist2 + dist3) / 2;
                        surfArea += sqrtf(fabs(s * (s - dist1) * (s - dist2) * (s - dist3)));
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
            float thresh, std::vector <Point2i> * output_ij_points,
            std::vector <Vec3f> * output_xyz_points, cv::Mat * output_mask,
            int inv1, int inv2, float inv2_thresh, cv::Mat * color)
        {
            // true if temporary 'visited' matrix allocated (we'll need to delete it after)
            bool tempVisMat = !color;

            // create 'visited' matrix
            if (tempVisMat) {
                color = new cv::Mat(xyz_map.size(), CV_8U);
                *color = cv::Scalar(255);
            }

            color->at<uchar>(seed) = 1;

            // stack for storing the 2d points
            static std::vector<Point2i> stk;
            const int R = xyz_map.rows, C = xyz_map.cols;

            // permanently allocate memory for our stack
            if (stk.size() < R * C) {
                stk.resize(R * C);
            }

            thresh *= thresh; // use square of distance to save computations
            float max_distance2 = inv2_thresh * inv2_thresh; // for interval2

            // add seed to stack
            stk[0] = seed;

            int stkSize = 1, total = 0, nNext;

            // stores next points
            std::array<Point2i, 4> nextPts;

            Point2i pt;
            const Vec3f * xyzPtr;
            Vec3f * oPtr;
            uchar * visPtr;
            bool sw;

            if (output_ij_points) {
                output_ij_points->clear();
                output_ij_points->reserve(R * C);
            }
            if (output_xyz_points) {
                output_xyz_points->clear();
                output_xyz_points->reserve(R * C);
            }

            int origX;

            // begin DFS / scanline hybrid flood fill
            while (stkSize > 0) {
                // pop current point from stack
                pt = stk[--stkSize];

                // create pointers to current row for faster access
                xyzPtr = xyz_map.ptr<Vec3f>(pt.y);
                visPtr = color->ptr<uchar>(pt.y);
                if (output_mask) oPtr = output_mask->ptr<Vec3f>(pt.y);;

                origX = pt.x;
                sw = true;

                const Vec3f * xyz;
                while (visPtr[pt.x] > 0) {
                    // if not visited, visit; otherwise ignore this point
                    xyz = &xyzPtr[pt.x];

                    // mark as visited
                    visPtr[pt.x] = 0;

                    // output this point to mask, etc.
                    if (output_mask) oPtr[pt.x] = *xyz;
                    if (output_ij_points) output_ij_points->push_back(pt);
                    if (output_xyz_points) output_xyz_points->push_back(*xyz);

                    // increment the total number of points
                    ++total;

                    // make a list of adjacent points
                    nNext = -1;
                    if (pt.y >= inv1) nextPts[++nNext] = std::move(Point2i(pt.x, pt.y - inv1));
                    if (pt.y < R - inv1) nextPts[++nNext] = std::move(Point2i(pt.x, pt.y + inv1));

                    if (inv2 > 0) {
                        if (pt.y >= inv2) nextPts[++nNext] = std::move(Point2i(pt.x, pt.y - inv2));
                        if (pt.y < R - inv2) nextPts[++nNext] = std::move(Point2i(pt.x, pt.y + inv2));
                    }

                    // go to each adjacent point
                    for (uint i = 0; i <= nNext; ++i) {
                        Point2i & adjPt = nextPts[i];
                        uchar & adjVis = color->at<uchar>(adjPt);

                        // skip if already visited
                        if (adjVis <= 1) continue;

                        // update & push to stack if point is close enough
                        if (util::norm(*xyz - xyz_map.at<Vec3f>(adjPt)) <
                            (i < 2 ? thresh : max_distance2)) {
                            stk[stkSize++] = adjPt;
                            adjVis = 1; // mark 'visiting'
                        }
                    }

                    // scanline
                    if (sw) {
                        // go right
                        pt.x += inv1;
                        if (pt.x >= C || visPtr[pt.x] == 0 ||
                            util::norm(*xyz - xyzPtr[pt.x]) >= thresh) {
                            sw = false;

                            // reset to middle
                            pt.x = origX - inv1;
                            xyz = &xyzPtr[origX];
                            if (pt.x < 0 || util::norm(*xyz - xyzPtr[pt.x]) >= thresh) {
                                break;
                            }
                        }
                    }
                    else {
                        // go left
                        pt.x -= inv1;
                        if (pt.x < 0 || util::norm(*xyz - xyzPtr[pt.x]) >= thresh) {
                            break;
                        }
                    }
                }
            }

            if (tempVisMat) {
                delete color;
                color = nullptr;
            }

            return total;
        }

        // convert an ij point to an angle, clockwise from (0, 1) (0 at 0 degrees, 2 * PI at 360)
        double pointToAngle(const Point2f & pt) {
            return fmod(atan2(pt.x, -pt.y) + PI, 2 * PI);
        }

        // convert angle to ij point with unit magnitude
        Point2f angleToPoint(double angle)
        {
            angle += 2.5 * PI;
            return cv::normalize(cv::Vec2f(cos(angle), sin(angle)));
        }

        // get angle between two points through a central point
        double angleBetweenPoints(const Point2f & a, const Point2f & b, const Point2f & center) {
            double angle = fabs(pointToAngle(a - center) - pointToAngle(b - center));
            if (angle > PI) return 2 * PI - angle;
            return angle;
        }

        Point2f normalize(const Point2f & pt)
        {
            if (pt.x == pt.y && pt.x == 0.0f) return pt;
            return pt / sqrtf(pt.x * pt.x + pt.y * pt.y);
        }

        Vec3f normalize(const Vec3f & vec)
        {
            Vec3f result = cv::normalize(vec);
            if (result[2] > 0) return -result;
            return result;
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

        // now basically just a wrapper around cv norm, probably will remove eventually
        template <class Param_T>
        double norm(const cv::Point_<Param_T> & pt, int cv_norm_type) {
            if (cv_norm_type == cv::NORM_L2SQR) {
                return pt.x * pt.x + pt.y * pt.y;
            }
            else if (cv_norm_type == cv::NORM_L2) {
                return magnitude(pt);
            }
            // warning: inefficient
            return cv::norm(cv::Mat(pt), cv_norm_type);
        }

        template <class Param_T>
        double norm(const cv::Point3_<Param_T> & pt, int cv_norm_type) {
            if (cv_norm_type == cv::NORM_L2SQR) {
                return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            }
            else if (cv_norm_type == cv::NORM_L2) {
                return magnitude(pt);
            }
            // warning: inefficient
            return cv::norm(cv::Mat(pt), cv_norm_type);
        }

        template <class Param_T, int n>
        double norm(const cv::Vec<Param_T, n> & pt, int cv_norm_type) {
            return cv::norm(pt, cv_norm_type);
        }

        // instantialize
        template double norm<int>(const cv::Point_<int> & pt, int cv_norm_type);
        template double norm<float>(const cv::Point_<float> & pt, int cv_norm_type);
        template double norm<double>(const cv::Point_<double> & pt, int cv_norm_type);
        template double norm<int>(const cv::Point3_<int> & pt, int cv_norm_type);
        template double norm<float>(const cv::Point3_<float> & pt, int cv_norm_type);
        template double norm<double>(const cv::Point3_<double> & pt, int cv_norm_type);
        template double norm<ushort, 3>(const cv::Vec<ushort, 3> & pt, int cv_norm_type);
        template double norm<int, 3>(const cv::Vec<int, 3> & pt, int cv_norm_type);
        template double norm<float, 3>(const cv::Vec<float, 3> & pt, int cv_norm_type);
        template double norm<double, 3>(const cv::Vec<double, 3> & pt, int cv_norm_type);

        // get angle between two 3D vectors through a central point
        double angleBetween3DVec(Vec3f a, Vec3f b, Vec3f center) {
            a -= center; b -= center;
            cv::Mat A(a), B(b);
            double dot = A.dot(B), mA = magnitude(a), mB = magnitude(b);

            double res = fabs(acos(dot / mA / mB));

            if (std::isnan(res)) return PI;
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
            cv::Point bestpt(0, 0);

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

        float contourCurvature(const std::vector<Point2i>& contour, int index,
            float radius, int max_tries)
        {
            const int N = (int) contour.size();

            Point2i center = contour[index];
            int idx[2]; Point2f points[2];

            // find the point on the contour at 'radius' distance from the center point
            for (int i = 0; i < 2; ++i) {
                int delta = i * 2 - 1; // {0: -1; 1: +1}
                idx[i] = index;

                int tries = 0;

                float dist = 0.0f;
                do {
                    idx[i] = (idx[i] + delta + N) % N;
                    dist = euclideanDistance(contour[idx[i]], center);
                    ++tries;
                } while (dist <= radius && idx[i] != index &&
                    (tries <= max_tries || max_tries < 0));

                points[i] = contour[idx[i]]; 

                const Point2i & prevPt = contour[(idx[i] - delta + N) % N];
                float pdist = euclideanDistance(prevPt, center);

                // scale linearly on the edge of the contour between
                // the previous and current points to approximate desired distance
                if (dist > radius && radius >= pdist) {
                    float fact = (radius - pdist) / (dist - pdist);
                    points[i] = (1.0 - fact) * Point2f(prevPt) + fact * points[i];
                }
            }

            float r2 = (idx[1] - idx[0]) / 2.0f; r2 *= r2;
            float dx = (points[1].x - points[0].x) / (idx[1] - idx[0]);
            float dy = (points[1].y - points[0].y) / (idx[1] - idx[0]);
            float d2x = (points[1].x + points[0].x - 2.0f * center.x) / r2;
            float d2y = (points[1].y + points[0].y - 2.0f * center.y) / r2;
            float norm = dx * dx + dy * dy;

            if (norm == 0.0f) return 0.0f;
            return fabs(dx * d2y - dy * d2x) / powf(norm, 1.5f);
        }

        float contourLocalAngle(const std::vector<Point2i> & contour, int index,
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

        float radiusInDirection(const cv::Mat & xyz_map, const Point2i & center,
            double angle, double angle_offset)
        {
            Point2f cen(center), dir = angleToPoint(angle + angle_offset);

            float curr = 
                ceilf(sqrtf(0.25f * xyz_map.rows * xyz_map.rows + 0.25f * xyz_map.cols * xyz_map.cols));

            // find outermost farthest set pixel by linear scan

            Point2i checkPt;
            for (; curr >= 0.0f; --curr) {
                cv::Point2f pos = cen + curr * dir;
                float xm = fmodf(pos.x, 1.0), ym = fmodf(pos.y, 1.0);

                checkPt = Point2i(floorf(pos.x), floorf(pos.y));

                if (pointInImage(xyz_map, checkPt) && xyz_map.at<Vec3f>(checkPt)[2] > 0) {
                    return curr;
                }
            }

            return 0;
        }

        pcl::PointXYZRGBA toPCLPoint(const Eigen::Vector3d & v, int r, int g, int b, int a) {
            pcl::PointXYZRGBA pt;
            pt.x = v.x(); pt.y = v.y(); pt.z = v.z();
            pt.r = r; pt.g = g; pt.b = b; pt.a = a;
            return pt;
        }

        pcl::PointXYZRGBA toPCLPoint(const Vec3f & v, int r, int g, int b, int a) {
            pcl::PointXYZRGBA pt;
            pt.x = v[0];
            pt.y = v[1];
            pt.z = v[2];
            pt.r = r;
            pt.b = b;
            pt.g = g;
            pt.a = a;
            return pt;
        }

        template<> bool PointComparer<Point2i>::operator()(Point2i a, Point2i b) {
            if (compare_y_then_x) {
                if (a.y == b.y) return reverse ^ (a.x < b.x);
                return reverse ^ (a.y < b.y);
            }
            else {
                if (a.x == b.x) return reverse ^ (a.y < b.y);
                return reverse ^ (a.x < b.x);
            }
        }

        template<> bool PointComparer<Point2f>::operator()(Point2f a, Point2f b) {
            if (compare_y_then_x) {
                if (a.y == b.y) return reverse ^ (a.x < b.x);
                return reverse ^ (a.y < b.y);
            }
            else {
                if (a.x == b.x) return reverse ^ (a.y < b.y);
                return reverse ^ (a.x < b.x);
            }
        }

        template<> bool PointComparer<Vec3f>::operator()(Vec3f a, Vec3f b) {
            for (int i = (compare_y_then_x ? 2 : 0);
                (compare_y_then_x ? i >= 0 : i < 3);
                (compare_y_then_x ? --i : ++i)) {
                if (a[i] == b[i]) continue;

                return reverse ^ (a[i] < b[i]);
            }
            return false;
        }

        template<> bool PointComparer<cv::Vec3i>::operator()(cv::Vec3i a, cv::Vec3i b) {
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
