#include "stdafx.h"
#include "version.h"

#include "HandFeatureExtractor.h"
#include "Object3D.h"
#include "Hand.h"
#include "Util.h"

using namespace boost::filesystem;

namespace ark {
    namespace classifier {
        namespace features {
            // default image extension
            static const std::string IMG_EXT = ".tsi";

            template <class T>
            static inline void readBinary(std::ifstream & ifs, T * val) {
                ifs.read(reinterpret_cast<char *>(val), sizeof(*val));
            }

            static void readTSI(cv::Mat & m, std::string path) {
                std::ifstream ifs(path, std::ios::binary | std::ios::in);

                ushort wid, hi;
                readBinary(ifs, &hi);
                readBinary(ifs, &wid);

                m = cv::Mat::zeros(hi, wid, CV_32FC3);

                int zr = 0;

                for (int i = 0; i < hi; ++i) {
                    Vec3f * ptr = m.ptr<Vec3f>(i);
                    for (int j = 0; j < wid; ++j) {
                        if (zr) --zr;
                        else {
                            if (!ifs) break;
                            float x; readBinary(ifs, &x);
                            if (x <= 1) {
                                ptr[j][0] = x;
                                readBinary(ifs, &ptr[j][1]);
                                readBinary(ifs, &ptr[j][2]);
                            }
                            else {
                                zr = (int)x - 2;
                            }
                        }
                    }
                }
            }

            static inline double contourDiameter(const std::vector<Point2i> & cont) {
                double maxd;
                Point2i startp, bestp;
                startp = bestp = cont[0];

                for (int _ = 0; _ < 2; ++_) {
                    maxd = 0;
                    for (auto p : cont) {
                        double curd = util::magnitude(p - startp);
                        if (curd > maxd) {
                            bestp = p;
                            maxd = curd;
                        }
                    }
                    startp = bestp;
                }

                return maxd;
            }

            void computeMeanAndVariance(const cv::Mat& clean, Vec3f center,
                double& avg_dist, double& var_dist, double& avg_depth, double& var_depth) {

                avg_dist = avg_depth = 0;
                int totalpts = 0;

                for (int r = 0; r < clean.rows; ++r) {
                    const Vec3f * ptr = clean.ptr<Vec3f>(r);
                    for (int c = 0; c < clean.cols; ++c) {
                        Vec3f pt = ptr[c];
                        if (pt[2] != 0) {
                            Point2f xypt(pt[0], pt[1]);
                            avg_dist +=
                                sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) + (pt[1] - center[1]) * (pt[1] - center[1]));
                            avg_depth += pt[2];
                            ++totalpts;
                        }
                    }
                }

                if (totalpts == 0) {
                    avg_dist = avg_depth = 1.0;
                    var_dist = var_depth = 0.0;
                    return;
                }

                avg_dist /= totalpts;
                avg_depth /= totalpts;

                var_dist = var_depth = 0;

                for (int r = 0; r < clean.rows; ++r) {
                    const Vec3f * ptr = clean.ptr<Vec3f>(r);

                    for (int c = 0; c < clean.cols; ++c) {
                        Vec3f pt = ptr[c];
                        if (pt[2] != 0) {
                            Point2f xypt(pt[0], pt[1]);
                            double dist =
                                sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) + (pt[1] - center[1]) * (pt[1] - center[1]));
                            var_dist += (dist - avg_dist) * (dist - avg_dist);
                            var_depth += (pt[2] - avg_depth) * (pt[2] - avg_depth);
                        }
                    }
                }

                var_dist /= totalpts;
                var_depth /= totalpts;
            }

            std::vector<double> extractHandInfo(const cv::Mat & depth_original) {

                cv::Mat depth;
                depth_original.convertTo(depth, CV_32FC3, 1.0 / 256);

                Object3D obj(depth);

                std::vector<double> result;

                if (obj.hasHand) {
                    Hand hand = obj.getHand();
                    result.push_back(hand.numFingers());

                    for (int j = 0; j < 3; ++j) result.push_back(hand.center_xyz[j]);

                    for (int i = 0; i < hand.numFingers(); ++i) {
                        for (int j = 0; j < 3; ++j) result.push_back(hand.fingers_xyz[i][j]);
                        for (int j = 0; j < 3; ++j) result.push_back(hand.defects_xyz[i][j]);
                    }
                }

                return result;
            }

            std::vector<double> extractHandFeatures(Object3D & obj, const cv::Mat & depth_map,
                Point2i top_left, double img_scale, int full_wid) {
                if (full_wid < 0) full_wid = depth_map.cols;

                std::vector<double> result;

                if (obj.hasHand) {
                    Hand hand = obj.getHand();

                    int nFingers = hand.numFingers();

                    Vec3f center = hand.center_xyz;
                    Point2i centerij = hand.center_ij;

                    double avgdist, vardist, avgdepth, vardepth;
                    computeMeanAndVariance(depth_map, center, avgdist, vardist, avgdepth, vardepth);

                    result.push_back(nFingers);

                    if (nFingers > 1) result.reserve(nFingers * 13 + 10);
                    else result.reserve(13);

                    double area = obj.getSurfArea();

                    result.push_back(avgdist * 20.0);
                    result.push_back(sqrt(vardist) * 25.0);
                    result.push_back(area * 10.00);
                    result.push_back(sqrt(vardepth) * 25.0);

                    double contArea = cv::contourArea(obj.getContour());
                    result.push_back(contArea / cv::contourArea(obj.getConvexHull()));

                    float radius; Point2f circCenter;
                    cv::minEnclosingCircle(obj.getContour(), circCenter, radius);
                    result.push_back(contArea / (PI * radius * radius));

                    result.push_back(cv::arcLength(obj.getContour(), true) /
                        cv::arcLength(obj.getConvexHull(), true) / 2.0);

                    result.push_back(contourDiameter(obj.getContour()) / (double)full_wid);

                    std::vector<std::pair<double, int>> fingerOrder;

                    double avgLen = 0;
                    for (int i = 0; i < nFingers; ++i) {
                        Vec3f finger = hand.fingers_xyz[i], defect = hand.defects_xyz[i];

                        double finger_len = util::euclideanDistance(finger, defect);
                        avgLen += finger_len;

                        fingerOrder.push_back(std::make_pair(finger_len, i));
                    }

                    std::sort(fingerOrder.begin(), fingerOrder.end(), std::greater<std::pair<int, int> >());

                    result.push_back(avgLen / nFingers * 5.0);

                    for (int k = 0; k < nFingers; ++k) {
                        int j = fingerOrder[k].second;

                        Vec3f finger = hand.fingers_xyz[j], defect = hand.defects_xyz[j];
                        Point2i fingerij = hand.fingers_ij[j], defectij = hand.defects_ij[j];

                        if (defect != center) {
                            result.push_back(util::euclideanDistance(finger, defect) * 5.0);
                            result.push_back(util::euclideanDistance(defect, center) * 5.0);
                            result.push_back(util::euclideanDistance(finger, center) * 5.0);
                        }

                        if (defectij == centerij) {
                            Point2i index, index_right, index_left;
                            double farthest = 0;
                            int hullIdx = 0;

                            std::vector<Point2i> hull = obj.getConvexHull();

                            if (hull.size() > 1)
                            {
                                for (int i = 0; i < hull.size(); i++)
                                {
                                    Point2i p1 = hull[i], p2 = hull[(i + 1) % hull.size()];
                                    Vec3f xyzP1 =
                                        util::averageAroundPoint(depth_map, p1 / img_scale - top_left, 22);

                                    double dist = util::euclideanDistance(xyzP1, center);

                                    if (p1.y < centerij.y + 5 &&
                                        dist > farthest)
                                    {
                                        farthest = dist;
                                        hullIdx = i;
                                        index = p1;
                                        index_right = hull[(i + 1) % hull.size()];
                                        index_left = hull[(i - 1) % hull.size()];
                                    }
                                }
                            }

                            double angle = util::angleBetweenPoints(index_left, index_right, index) / (2 * PI);

                            result.push_back(angle);

                            if (hull.size() > 3)
                            {
                                index_right = hull[(hullIdx + 2) % hull.size()];
                                index_left = hull[(hullIdx - 2) % hull.size()];
                                angle = util::angleBetweenPoints(index_left, index_right, index) / (2 * PI);
                                result.push_back(angle);
                            }
                        }
                        else {
                            result.push_back(util::angleBetween3DVec(finger, defect, center) / PI);
                            result.push_back(util::angleBetweenPoints(fingerij, centerij, defectij) / PI);
                        }

                        if (defectij != centerij) {
                            result.push_back(util::pointToAngle(fingerij - centerij));
                            result.push_back(util::pointToAngle(defectij - centerij));
                        }

                        double minDistDefect = depth_map.cols, minDistFinger = depth_map.cols;
                        double maxDistDefect = 0, maxDistFinger = 0;

                        //double minSp = depth_map.cols, maxSp = 0;

                        if (nFingers > 1) {
                            for (int jj = 0; jj < nFingers; ++jj) {
                                if (j == jj) continue;

                                double distDefect = util::euclideanDistance(defect, hand.defects_xyz[jj]),
                                    distFinger = util::euclideanDistance(finger, hand.fingers_xyz[jj]);

                                //double spFinger = Util::clusterShortestPath(depth_map, 
                                //                               fingerij / img_scale - top_left,
                                //                               hand.fingers_ij[jj] / img_scale - top_left);

                                if (distDefect < minDistDefect) minDistDefect = distDefect;
                                if (distDefect > maxDistDefect) maxDistDefect = distDefect;
                                if (distFinger < minDistFinger) minDistFinger = distFinger;
                                if (distFinger > maxDistFinger) maxDistFinger = distFinger;
                                //if (spFinger > minSp) minSp = spFinger;
                                //if (spFinger > maxSp) maxSp = spFinger;
                            }

                            result.push_back(minDistFinger * 5.0);
                            result.push_back(maxDistFinger * 5.0);
                            result.push_back(minDistDefect * 5.0);
                            result.push_back(maxDistDefect * 5.0);
                            //result.push_back(minSp * 5.0);
                            //result.push_back(maxSp * 5.0);
                        }
                    }

                    for (unsigned i = 0; i < result.size(); ++i) {
                        if (isnan(result[i])) {
                            result[i] = 1.0;
                        }
                        else if (result[i] >= FLT_MAX) {
                            result[i] = 100.0;
                        }
                    }
                }

                return result;
            }

            std::vector<double> extractHandFeatures(const cv::Mat & depth_original) {
                cv::Mat depth;
                if (depth_original.type() != 21) {
                    depth_original.convertTo(depth, CV_32FC3, 1.0 / 256);
                }
                else {
                    depth = depth_original;
                }

                Object3D obj(depth);
                return extractHandFeatures(obj, depth);
            }

            /**
            * Helper for reading in a test case file & passing the information retrieved to a feature extractor function
            */
            static std::vector<double> extractFileHelper(std::string testCaseName, std::string dataDir,
                std::string depthPath, std::vector<double>(*extractFunc)
                (const cv::Mat & depth_map)) {

                if (dataDir[dataDir.size() - 1] != '/' && dataDir[dataDir.size() - 1] != '\\') {
                    dataDir += path::preferred_separator;
                }

                // read depth map
                cv::Mat depth;
                if (IMG_EXT == ".tsi")
                    readTSI(depth, dataDir + depthPath + testCaseName + IMG_EXT);
                else
                    depth = cv::imread(dataDir + depthPath + testCaseName + IMG_EXT);
                return extractFunc(depth);
            }

            std::vector<double> extractHandInfo(std::string testCaseName, std::string dataDir, std::string depthPath) {

                return extractFileHelper(testCaseName, dataDir, depthPath, extractHandInfo);
            }

            std::vector<double> extractHandFeatures(std::string testCaseName, std::string dataDir, std::string depthPath) {

                return extractFileHelper(testCaseName, dataDir, depthPath, extractHandFeatures);
            }
        }
    }
}
