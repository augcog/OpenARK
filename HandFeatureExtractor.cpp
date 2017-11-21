#include "stdafx.h"

#include "HandFeatureExtractor.h"
#include "Object3D.h"
#include "Hand.h"
#include "Util.h"

using namespace boost::filesystem;

namespace classifier {
    namespace features {
        // default image extension
        static const std::string IMG_EXT = ".png";

        void computeMeanAndVariance(const cv::Mat& clean, cv::Vec3f center,
            double& avgdist, double& vardist, double& avgdepth, double& vardepth) {

            avgdist = avgdepth = 0;
            int totalpts = 0;

            for (int r = 0; r < clean.rows; ++r) {
                const cv::Vec3f * ptr = clean.ptr<cv::Vec3f>(r);
                for (int c = 0; c < clean.cols; ++c) {
                    cv::Vec3f pt = ptr[c];
                    if (pt[2] != 0) {
                        cv::Point2f xypt(pt[0], pt[1]);
                        avgdist += 
                            sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) + (pt[1] - center[1]) * (pt[1] - center[1]));
                        avgdepth += pt[2];
                        ++totalpts;
                    }
                }
            }

            if (totalpts == 0) {
                avgdist = avgdepth = 1.0;
                vardist = vardepth = 0.0;
                return;
            }

            avgdist /= totalpts;
            avgdepth /= totalpts;

            vardist = vardepth = 0;

            for (int r = 0; r < clean.rows; ++r) {
                const cv::Vec3f * ptr = clean.ptr<cv::Vec3f>(r);
                for (int c = 0; c < clean.cols; ++c) {
                    cv::Vec3f pt = ptr[c];
                    if (pt[2] != 0) {
                        cv::Point2f xypt(pt[0], pt[1]);
                        double dist = 
                            sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) + (pt[1] - center[1]) * (pt[1] - center[1]));
                        vardist += (dist - avgdist) * (dist - avgdist);
                        vardepth += (pt[2] - avgdepth) * (pt[2] - avgdepth);
                    }
                }
            }

            vardist /= totalpts;
            vardepth /= totalpts;
        }


        std::vector<double> extractHandInfo(const cv::Mat & depth_original) {

            cv::Mat depth;
            depth_original.convertTo(depth, CV_32FC3, 1.0 / 256);

            Object3D obj(depth);

            std::vector<double> result;

            if (obj.hasHand) {
                Hand hand = obj.getHand();
                result.push_back(hand.getNumFingers());

                for (int j = 0; j<3; ++j) result.push_back(hand.centroid_xyz[j]);

                for (int i = 0; i < hand.getNumFingers(); ++i) {
                    for (int j = 0; j<3; ++j) result.push_back(hand.fingers_xyz[i][j]);
                    for (int j = 0; j<3; ++j) result.push_back(hand.defects_xyz[i][j]);
                }
            }

            return result;
        }

        std::vector<double> extractHandFeatures(const Object3D & obj,  cv::Mat & depthMap) {
            std::vector<double> result;

            if (obj.hasHand) {
                Hand hand = obj.getHand();

                int nFingers = hand.getNumFingers();

                cv::Vec3f center = hand.centroid_xyz;
                cv::Point centerij = hand.centroid_ij;

                if (nFingers > 1) result.reserve(nFingers * 11 + 6);
                else result.reserve(13);

                result.push_back(nFingers);

                double avgdist, vardist, avgdepth, vardepth;
                computeMeanAndVariance(depthMap, center, avgdist, vardist, avgdepth, vardepth);

                double area = Util::surfaceArea(depthMap);

                result.push_back(cv::contourArea(obj.getComplexContour()) / cv::contourArea(obj.getConvexHull()));
                result.push_back(avgdist * 20.0);
                result.push_back(sqrt(vardist) * 25.0);
                //result.push_back(avgdepth * 1.50);
                result.push_back(area * 5.00);
                result.push_back(sqrt(vardepth) * 25.0);
                result.push_back(cv::contourArea(obj.getComplexContour()) / cv::contourArea(obj.getConvexHull()));

                for (int i = 0; i < nFingers; ++i) {
                    cv::Vec3f finger = hand.fingers_xyz[i], defect = hand.defects_xyz[i];
                    cv::Point fingerij = hand.fingers_ij[i], defectij = hand.defects_ij[i];

                    result.push_back(Util::euclideanDistance3D(finger, defect) * 5.0);
                    if (defect != center) {
                        result.push_back(Util::euclideanDistance3D(defect, center) * 5.0);
                    }
                    result.push_back(Util::euclideanDistance3D(finger, center) * 5.0);

                    if (defectij == centerij) {
                        cv::Point index, index_right, index_left;
                        double farthest = 0;

                        std::vector<cv::Point> hull = obj.getConvexHull();

                        if (hull.size() > 1)
                        {
                            for (int i = 0; i < hull.size(); i++)
                            {
                                cv::Point p1 = hull[i], p2 = hull[(i + 1) % hull.size()];
                                if (p1.y < centerij.y && Util::euclideanDistance2D(p1, centerij) > farthest)
                                {
                                    farthest = Util::euclideanDistance2D(p1, centerij);
                                    index = p1;
                                    index_right = hull[(i + 1) % hull.size()];
                                    index_left = hull[(i - 1) % hull.size()];
                                }
                            }
                        }

                        double angle = Util::triangleAngleCalculation(index_left.x, index_left.y,
                            index.x, index.y, index_right.x, index_right.y) / 360.0;

                        result.push_back(angle);
                    }
                    else {
                        result.push_back(Util::angleBetween3DVec(finger, defect, center) / PI);
                        result.push_back(Util::angleBetweenPoints(fingerij, centerij, defectij) / PI);
                    }

                    result.push_back(Util::pointToAngle(fingerij - centerij));
                    if (defectij != centerij) {
                        result.push_back(Util::pointToAngle(defectij - centerij));
                    }

                    double minDistDefect = depthMap.cols, minDistFinger = depthMap.cols;
                    double maxDistDefect = 0, maxDistFinger = 0;

                    if (nFingers > 1) {
                        for (int j = 0; j < nFingers; ++j) {
                            if (i == j) continue;

                            double distDefect = Util::euclideanDistance3D(defect, hand.defects_xyz[j]),
                                distFinger = Util::euclideanDistance3D(finger, hand.fingers_xyz[j]);

                            if (distDefect < minDistDefect) {
                                minDistDefect = distDefect;
                            }
                            if (distDefect > maxDistDefect) {
                                maxDistDefect = distDefect;
                            }
                            if (distFinger < minDistFinger) {
                                minDistFinger = distFinger;
                            }
                            if (distFinger > maxDistFinger) {
                                maxDistFinger = distFinger;
                            }
                        }

                        result.push_back(minDistFinger * 5.0);
                        result.push_back(maxDistFinger * 5.0);
                        result.push_back(minDistDefect * 5.0);
                        result.push_back(maxDistDefect * 5.0);
                    }
                }

                for (unsigned i = 0; i < result.size(); ++i) {
                    if (isnan(result[i])) {
                        result[i] = 1.0;
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
            (const cv::Mat & depthMap)) {

            if (dataDir[dataDir.size() - 1] != '/' && dataDir[dataDir.size() - 1] != '\\') {
                dataDir += path::preferred_separator;
            }

            // read depth map
            cv::Mat depth = cv::imread(dataDir + depthPath + testCaseName + IMG_EXT);
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