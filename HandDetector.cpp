#include "stdafx.h"
#include "HandDetector.h"

namespace ark {
    HandDetector::HandDetector(bool elim_planes, DetectionParams::Ptr params)
        : Detector(params), externalPlaneDetector(false) {
        if (elim_planes) {
            planeDetector = std::make_shared<PlaneDetector>();
        }
        else {
            planeDetector = nullptr;
        }
    }

    HandDetector::HandDetector(PlaneDetector::Ptr plane_detector, DetectionParams::Ptr params)
            : Detector(params), planeDetector(plane_detector), externalPlaneDetector(true) {
    }

    const std::vector<Hand::Ptr> & HandDetector::getHands() const {
        return hands;
    }

    void HandDetector::detect(cv::Mat & image)
    {
        hands.clear();

        // 1. initialize
        const int R = image.rows, C = image.cols;
        cv::Mat floodFillMap(R, C, CV_8U);

        const Vec3f * ptr;
        uchar * visPtr;

        for (int r = 0; r < R; ++r)
        {
            visPtr = floodFillMap.ptr<uchar>(r);
            ptr = image.ptr<Vec3f>(r);
            for (int c = 0; c < C; ++c)
            {
                visPtr[c] = ptr[c][2] > 0 ? 255 : 0;
            }
        }

        // 2. eliminate large planes

        if (planeDetector) {
            if (!externalPlaneDetector) planeDetector->update(image);
            const std::vector<FramePlane::Ptr> & planes = planeDetector->getPlanes();
            if (planes.size()) {
                for (FramePlane::Ptr plane : planes) {
                    util::removePlane<uchar>(image, floodFillMap, plane->equation,
                        params->handPlaneMinNorm);
                }
            }
        }

        // 3. flood fill on point cloud 
        std::shared_ptr<Hand> bestHandObject;
        float closestHandDist = FLT_MAX;

        std::vector<Point2i> allIJPoints;
        std::vector<Vec3f> allXYZPoints;

        allIJPoints.reserve(R * C);
        allXYZPoints.reserve(R * C);

#ifdef DEBUG
        cv::Mat floodFillVis = cv::Mat::zeros(R, C, CV_8UC3);
        int compID = 0;
#endif

        // compute the minimum number of points in a cluster according to params
        const int CLUSTER_MIN_POINTS = (int)(params->handClusterMinPoints * R * C);

        for (int r = 0; r < R; r += params->handClusterInterval)
        {
            ptr = image.ptr<Vec3f>(r);
            visPtr = floodFillMap.ptr<uchar>(r);

            for (int c = 0; c < C; c += params->handClusterInterval)
            {
                if (visPtr[c] > 0 && ptr[c][2] > 0)
                {
                    int points_in_comp = util::floodFill(image, Point2i(c, r),
                        params->handClusterMaxDistance,
                        &allIJPoints, &allXYZPoints, nullptr, 1, 6,
                        params->handClusterMaxDistance * 8, &floodFillMap);

                    if (points_in_comp >= CLUSTER_MIN_POINTS)
                    {
                        VecP2iPtr ijPoints = std::make_shared<std::vector<Point2i> >(allIJPoints);
                        VecV3fPtr xyzPoints = std::make_shared<std::vector<Vec3f> >(allXYZPoints);

                        // 4. for each cluster, test if hand

                        // if matching required conditions, construct 3D object
                        Hand::Ptr handPtr = std::make_shared<Hand>(ijPoints, xyzPoints, image,
                            params, false, points_in_comp);

                        if (ijPoints->size() < CLUSTER_MIN_POINTS) continue;

#ifdef DEBUG
                        cv::Vec3b color = util::paletteColor(compID++);
                        for (uint i = 0; i < points_in_comp; ++i) {
                            floodFillVis.at<Vec3b>(allIJPoints[i]) = color;
                        }

                        if (handPtr->getWristIJ().size() >= 2) {
                            cv::circle(floodFillVis, handPtr->getWristIJ()[0], 5, cv::Scalar(100, 255, 255));
                            cv::circle(floodFillVis, handPtr->getWristIJ()[1], 5, cv::Scalar(100, 255, 255));
                        }
#endif

                        if (handPtr->isValidHand()) {
                            float distance = handPtr->getDepth();

                            if (distance < closestHandDist) {
                                bestHandObject = handPtr;
                                closestHandDist = distance;
                            }

#ifdef DEBUG
                            cv::polylines(floodFillVis, handPtr->getContour(), true, cv::Scalar(255, 255, 255));
#endif
                            if (handPtr->getSVMConfidence() >
                                params->handSVMHighConfidenceThresh ||
                                !params->handUseSVM) {
                                // avoid duplicate hand
                                if (bestHandObject == handPtr) bestHandObject = nullptr;
                                hands.push_back(handPtr);
                            }
                        }
                    }
                }
            }
        }

        if (bestHandObject != nullptr) {
            // if no hands surpass 'high confidence threshold', at least add one hand
            hands.push_back(bestHandObject);
        }

        if (params->handUseSVM) {
            std::sort(hands.begin(), hands.end(), [](Hand::Ptr a, Hand::Ptr b) {
                return a->getSVMConfidence() > b->getSVMConfidence();
            });
        }
        else {
            std::sort(hands.begin(), hands.end(), [](Hand::Ptr a, Hand::Ptr b) {
                return a->getDepth() < b->getDepth();
            });
        }
#ifdef DEBUG
        cv::imshow("[Hand Flood Fill Debug]", floodFillVis);
#endif
    }
}
