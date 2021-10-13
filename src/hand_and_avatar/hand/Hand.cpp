#include "stdafx.h"
#include "Version.h"

#include "Util.h"
#include "Hand.h"
#include "Visualizer.h"
#include "HandClassifier.h"

// limited to file scope
namespace {
    /**
    * Comparator for sorting defects in order of angle (only available in Hand.cpp)
    */
    class DefectComparer {
    public:
        /**
        * Create a new defect comparator
        * @param contour the contour which the defects were computed from
        * @param defects list of defects
        * @param center center point from which slopes should be computed from
        */
        DefectComparer(const std::vector<ark::Point2i> & contour,
            const std::vector<cv::Vec4i> & defects, const ark::Point2i & center) {
            angle.resize(contour.size());

            for (unsigned i = 0; i < defects.size(); ++i) {
                ark::Point2i pt = contour[defects[i][ATTR_USED]] - center;
                angle[defects[i][ATTR_USED]] = ark::util::pointToAngle(pt);
            }
        }

        /**
        * Compare two defects (least counterclockwise from bottom is less)
        */
        bool operator()(cv::Vec4i a, cv::Vec4i b) const {
            int idxA = a[ATTR_USED], idxB = b[ATTR_USED];
            return angle[idxA] > angle[idxB];
        }

    private:
        /**
        * index of the Vec4i used for comparison
        */
        const int ATTR_USED = 2;

        /**
        * stores the slopes of all the points on the contour
        */
        std::vector<double> angle;

        // default constructor disabled
        DefectComparer() {};
    };
}

namespace ark {

    static const std::string SVM_PATH = "config/hand-svm";
    // Initialize the SVM hand validator
    static const classifier::SVMHandValidator & handValidator = classifier::SVMHandValidator(SVM_PATH);

    Hand::Hand() : FrameObject() { }

    Hand::Hand(const cv::Mat & cluster_depth_map, DetectionParams::Ptr params)
        : FrameObject(cluster_depth_map, params)
    {
        // Determine whether cluster is a hand
        isHand = checkForHand();
    }

    Hand::Hand(VecP2iPtr points_ij, VecV3fPtr points_xyz, const cv::Mat & depth_map, DetectionParams::Ptr params, bool sorted, int points_to_use)
        : FrameObject(points_ij, points_xyz, depth_map, params, sorted, points_to_use)
    {
        // Determine whether cluster is a hand
        isHand = checkForHand();
    }

    Hand::~Hand() { }

    int Hand::getNumFingers() const {
        return (int)fingersXYZ.size();
    }

    bool Hand::checkForHand()
    {
        checkEdgeConnected();

        // if not connected, stop
        if (params->handRequireEdgeConnected && !leftEdgeConnected && !rightEdgeConnected) {
            return false;
        }

#ifdef DEBUG
        cv::Mat visual = cv::Mat::zeros(fullMapSize.height, fullMapSize.width, CV_8UC3);
        cv::Mat defectVisual = cv::Mat::zeros(fullMapSize.height, fullMapSize.width, CV_8UC3);
#endif
        if (points->size() == 0 || num_points == 0) {
            return false;
        }

        computeContour(xyzMap, points.get(), points_xyz.get(), topLeftPt, num_points);

        // ** Find center of contour **
        Point2i centroid = findCenter(contour) - topLeftPt;

        // Make sure center is on cluster
        centroid = util::nearestPointOnCluster(xyzMap, centroid);

        // Find radius and center point of largest inscribed circle above center
        Vec3f topPt = util::averageAroundPoint(xyzMap, (*points)[0] - topLeftPt,
            params->xyzAverageSize);

        // radius of largest inscribed circle
        double cirrad;
        // center of largest inscribed circle (coordinates from top left of FULL depth map)
        Point2i circen =
            util::largestInscribedCircle(contour, xyzMap, getBoundingBox(), topPt,
                params->centerMaxDistFromTop, &cirrad);

        Point2i center = circen - topLeftPt;
        this->palmCenterIJ = circen;
        this->palmCenterXYZ = util::averageAroundPoint(xyzMap, center, params->xyzAverageSize);
        this->circleRadius = cirrad;

        // ** Find wrist positions **
        int wristL = -1, wristR = -1;
        Point2i wristL_ij, wristR_ij;
        Vec3f wristL_xyz, wristR_xyz;

        // 1. get seed points for wrist detection
        //    - if connected to edge, gets leftmost & right most connected contour points
        //    - else, gets lowest contour point
        int contactL = -1, contactR = -1, direction = 1;
        Point2i contactL_ij, contactR_ij;

        const int lMargin = params->contactSideEdgeThresh,
            rMargin = fullMapSize.width - params->contactSideEdgeThresh;

        for (unsigned i = 0; i < contour.size(); ++i) {
            Point2i pt = contour[i];

            if (touchingEdge()) {
                if (pt.y > fullMapSize.height * params->handEdgeConnectMaxY &&
                    util::pointOnEdge(fullMapSize, pt, params->contactBotEdgeThresh,
                        params->contactSideEdgeThresh)) {

                    if (contactL == -1) {
                        contactL = contactR = i;
                        continue;
                    }

                    const Point2i & ccl = contour[contactL], &ccr = contour[contactR];

                    if (pt.x <= lMargin) {
                        if (ccl.x > lMargin || ccl.y > pt.y) contactL = i;
                        if (ccr.x <= lMargin && ccr.y < pt.y) contactR = i;
                    }
                    else if (pt.x >= rMargin) {
                        if (ccr.x < rMargin || ccr.y > pt.y) contactR = i;
                        if (ccl.x >= rMargin && ccl.y < pt.y) contactL = i;
                    }
                    else {
                        if (ccl.x > pt.x) contactL = i;
                        if (ccr.x < pt.x) contactR = i;
                    }
                }
            }

            else {
                if (contactL == -1 || pt.y > contour[contactL].y) {
                    contactL = contactR = i;
                }
            }
        }

        if (contactL >= 0 && contactR >= 0) {
            contactL_ij = contour[contactL];
            contactR_ij = contour[contactR];

            // step 2: detect direction to move from lci and rci
            // direction: 1 = lci +, rci -; -1 = lci -, rci +
            if ((contactR > contactL && contactR - contactL < contour.size() / 2) ||
                (contactR <= contactL && contactL - contactR >= contour.size() / 2)) {
                direction = -1;
            }

            // step 3: move in direction until close enough to center
            int i = contactL;
            do {
                Point2i pt = contour[i];
                Vec3f xyz = util::averageAroundPoint(xyzMap,
                    pt - topLeftPt, params->xyzAverageSize);

                float dist = util::euclideanDistance(xyz, this->palmCenterXYZ);

                if (dist <= params->wristCenterDistThresh) {
                    wristL = i;
                    break;
                }

                i = (int)(i + direction + contour.size()) % contour.size();
            } while (i != contactR);

            i = contactR;
            do {
                Point2i pt = contour[i];
                Vec3f xyz = util::averageAroundPoint(xyzMap,
                    pt - topLeftPt, params->xyzAverageSize);

                float dist = util::euclideanDistance(xyz, this->palmCenterXYZ);

                if (dist <= params->wristCenterDistThresh) {
                    wristR = i;
                    break;
                }

                i = (int)(i - direction + contour.size()) % contour.size();
            } while (i != contactL);
        }

        if (wristL < 0 || wristR < 0) {
#ifdef DEBUG
            std::cerr << "[Hand Debug]: WRIST NOT FOUND\n";
#endif
            return false;
        }

        if (contour[wristL].x > contour[wristR].x) {
            std::swap(wristL, wristR);
        }

        wristL_ij = contour[wristL];
        wristR_ij = contour[wristR];
        wristL_xyz = util::averageAroundPoint(xyzMap,
            wristL_ij - topLeftPt, params->xyzAverageSize);
        wristR_xyz = util::averageAroundPoint(xyzMap,
            wristR_ij - topLeftPt, params->xyzAverageSize);

        float wristWidth = util::euclideanDistance(wristL_xyz, wristR_xyz);

        // output wrist points
        this->wristXYZ.push_back(wristL_xyz);
        this->wristXYZ.push_back(wristR_xyz);
        this->wristIJ.push_back(wristL_ij);
        this->wristIJ.push_back(wristR_ij);

        // Eliminate by wrist width
        if (wristWidth < params->wristWidthMin || wristWidth > params->wristWidthMax) {
#ifdef DEBUG
            std::cerr << "[Hand Debug]: OBJECT ELIMINATED BY WRIST WIDTH (" << wristWidth << "m)\n";
#endif
            return false;
        }

        // (finished detecting wrist)

        // ** Remove everything below wrist **

        std::vector<Point2i> aboveWristPointsIJ;
        std::vector<Vec3f> aboveWristPointsXYZ;

        if (wristR_ij.x != wristL_ij.x) {
            double slope = (double)(wristR_ij.y - wristL_ij.y) / (wristR_ij.x - wristL_ij.x);

            for (int i = 0; i < num_points; ++i) {
                const Point2i & pt = (*points)[i];
                double y_hat = wristL_ij.y + (pt.x - wristL_ij.x) * slope;

                Vec3f & vec = xyzMap.at<Vec3f>(pt - topLeftPt);

                if (pt.y > y_hat) {
                   vec = 0;
                }
                else {
                    aboveWristPointsIJ.push_back(pt);
                    aboveWristPointsXYZ.push_back(vec);
                }
            }
        }

        num_points = (int)aboveWristPointsIJ.size();
        points->swap(aboveWristPointsIJ);
        points_xyz->swap(aboveWristPointsXYZ);

        // recompute contour
        computeContour(xyzMap, points.get(), points_xyz.get(), topLeftPt, num_points);

        // ** Find dominant direction **
        float contourFar = -1.0; uint contourFarIdx = 0;
        for (uint i = 0; i < contour.size(); ++i) {
            float norm = util::norm(util::averageAroundPoint(xyzMap, contour[i] - topLeftPt, params->xyzAverageSize) -
                                    this->palmCenterXYZ);
            if (norm > contourFar) {
                contourFar = norm;
                contourFarIdx = i;
            }
        }

        this->dominantDir = util::normalize(contour[contourFarIdx] - this->palmCenterIJ);

        // ** SVM check **
        if (params->handUseSVM && handValidator.isTrained()) {
            this->svmConfidence = handValidator.classify(*this, xyzMap,
                topLeftPt, fullMapSize.width);
            if (this->svmConfidence < params->handSVMConfidenceThresh) {
                // SVM confidence value below threshold, reverse decision & destroy the hand instance
                return false;
            }
        }

        // if too small/large, stop
        surfaceArea = util::surfaceArea(fullMapSize, *points, *points_xyz, num_points);
        if (surfaceArea < params->handMinArea || surfaceArea > params->handMaxArea) {
#ifdef DEBUG
            std::cerr << "[Hand Debug]: OBJECT ELIMINATED BY SURFACE AREA (" << surfaceArea << "m^2)\n";
#endif
            return false;
        }

        // ** Detect fingers **

        // recompute convex hull based on new contour
        convexHull.clear(); getConvexHull();

        // compute defects
        std::vector<cv::Vec4i> defects;

        if (indexHull.size() > 3)
        {
            std::vector<int> tmpHull;

            cv::convexityDefects(contour, indexHull, defects);
        }

        // sort all defects found by angle
        DefectComparer comparer(contour, defects, this->palmCenterIJ);
        std::sort(defects.begin(), defects.end(), comparer);

        // stores fingertip and defect candidates
        std::vector<int> fingerTipCands, fingerDefectCands, goodDefects;

        /* stores end point of previous defect
           (for considering if current start point is far enough to be a separate finger) */
        Vec3f lastEnd;

        /* true if current defect is the first defect meeting
           the angle & distance criteria for defects*/
        bool first = true;

        // process defects
        for (int i = 0; i < defects.size(); ++i)
        {
            // contains info about the defect
            cv::Vec4i defect = defects[i];

#ifdef DEBUG
            cv::line(defectVisual, contour[defects[i][0]], contour[defects[i][2]], cv::Scalar(0, 255, 0));
            cv::line(defectVisual, contour[defects[i][1]], contour[defects[i][2]], cv::Scalar(0, 0, 255));
            cv::circle(defectVisual, contour[defects[i][2]], 5, cv::Scalar(255,255,255), 2);
#endif

            // point on convex hull where defect begins. fingertip candidate
            Point2i start = contour[defect[0]] - topLeftPt;

            // point on convex hull where defect ends. fingertip candidate
            Point2i end = contour[defect[1]] - topLeftPt;

            // farthest point in the defect from the convex hull
            Point2i farPt = contour[defect[2]] - topLeftPt;

            // snap to nearest point actually on the cluster (should already be, just in case)
            start = util::nearestPointOnCluster(xyzMap, start);
            end = util::nearestPointOnCluster(xyzMap, end);
            farPt = util::nearestPointOnCluster(xyzMap, farPt);

            // if any of the points is somehow out of the image, skip
            if (!util::pointInImage(xyzMap, farPt) ||
                !util::pointInImage(xyzMap, start) ||
                !util::pointInImage(xyzMap, end)) continue;

            // obtain xyz positions of points
            Vec3f far_xyz = util::averageAroundPoint(xyzMap, farPt, params->xyzAverageSize);
            Vec3f start_xyz = util::averageAroundPoint(xyzMap, start, params->xyzAverageSize);
            Vec3f end_xyz = util::averageAroundPoint(xyzMap, end, params->xyzAverageSize);

            // compute some distances used in heuristics
            double farCenterDist = util::euclideanDistance(far_xyz, this->palmCenterXYZ);
            double startEndDist = util::euclideanDistance(start_xyz, end_xyz);

            if (farCenterDist > params->defectFarCenterMinDist &&
                farCenterDist < params->defectFarCenterMaxDist &&
                startEndDist > params->defectStartEndMinDist)
            {
                goodDefects.push_back(i);

                // the angle from start through farPt to end
                double angle = util::angleBetweenPoints(start, end, farPt);
                // if angle too large skip
                if (angle > params->defectMaxAngle) continue;

                if (!util::pointOnEdge(fullMapSize, start + topLeftPt, params->bottomEdgeThresh, params->sideEdgeThresh) &&
                    (first || util::euclideanDistance(lastEnd, start_xyz) > params->defectMinDist)) {
                    // add start as candidate
                    fingerTipCands.push_back(defect[0]);
                    fingerDefectCands.push_back(defect[2]);
                    first = false;
                }

                if (!util::pointOnEdge(fullMapSize, end + topLeftPt, params->bottomEdgeThresh, params->sideEdgeThresh)) {
                    // add end as candidate
                    fingerTipCands.push_back(defect[1]);
                    fingerDefectCands.push_back(defect[2]);
                }

                // set last end point
                lastEnd = end_xyz;
            }
        }

#ifdef DEBUG
        cv::polylines(visual, contour, true, cv::Scalar(0, 200, 0));

        for (int i = 0; i < goodDefects.size(); ++i)
        {
            cv::Vec4i defect = defects[goodDefects[i]];
            Point2i start = contour[defect[0]] - topLeftPt,
                end = contour[defect[1]] - topLeftPt,
                farPt = contour[defect[2]] - topLeftPt;

            cv::circle(visual, farPt + topLeftPt, 10, cv::Scalar(255, 255, 0), 2);

            if (start.y + topLeftPt.y > fullMapSize.height - 20 ||
                end.y + topLeftPt.y > fullMapSize.height - 20) {
            }

            cv::line(visual, start + topLeftPt, farPt + topLeftPt, cv::Scalar(255, 100, 0), 2);
            cv::line(visual, end + topLeftPt, farPt + topLeftPt, cv::Scalar(0, 0, 255), 2);
        }

        cv::circle(visual, circen, cirrad, cv::Scalar(255, 0, 255), 2);

        for (int i = 0; i < fingerDefectCands.size(); ++i) {
            cv::circle(visual, contour[fingerDefectCands[i]], 8, cv::Scalar(255, 255, 255), 2);
        }

        cv::rectangle(visual, contactL_ij - Point2i(10, 10), contactL_ij + Point2i(10, 10),
            cv::Scalar(0, 0, 255), 2);
        cv::rectangle(visual, contactR_ij - Point2i(10, 10), contactR_ij + Point2i(10, 10),
            cv::Scalar(0, 0, 255), 2);

        cv::rectangle(visual, wristR_ij - Point2i(10, 10), wristR_ij + Point2i(10, 10),
            cv::Scalar(0, 255, 255), 2);
        cv::rectangle(visual, wristL_ij - Point2i(10, 10), wristL_ij + Point2i(10, 10),
            cv::Scalar(0, 255, 255), 2);

        cv::imshow("[Hand Defects Debug]", defectVisual);
#endif

        // select fingers from candidates
        std::vector<Point2i> fingerTipsIj, fingerDefectsIj;
        std::vector<Vec3f> fingerTipsXyz;
        std::vector<int> fingerTipsIdx, fingerDefectsIdx;

        for (unsigned i = 0; i < fingerTipCands.size(); ++i)
        {
            Point2i finger_ij = contour[fingerTipCands[i]] - topLeftPt;
            Point2i defect_ij = contour[fingerDefectCands[i]] - topLeftPt;

            if (defect_ij.y < center.y + params->defectMaxYFromCenter &&
                defect_ij.y + topLeftPt.y < fullMapSize.height - params->bottomEdgeThresh) {

                Vec3f finger_xyz = util::averageAroundPoint(xyzMap, finger_ij, params->xyzAverageSize);
                Vec3f defect_xyz = util::averageAroundPoint(xyzMap, defect_ij, params->xyzAverageSize);

                // compute a number of features used to eliminate finger candidates
                float finger_length = util::euclideanDistance(finger_xyz, defect_xyz);
                float centroid_defect_dist = util::euclideanDistance(this->palmCenterXYZ, defect_xyz);
                float finger_defect_slope = (float)(defect_ij.y - finger_ij.y) / abs(defect_ij.x - finger_ij.x);
                float finger_center_slope = (float)(center.y - finger_ij.y) / abs(center.x - finger_ij.x);
                double centroid_defect_finger_angle = util::angleBetweenPoints(finger_ij, center, defect_ij);

                float finger_length_ij = util::euclideanDistance(finger_ij, defect_ij);
                float curve_near = util::contourCurvature(contour, fingerTipCands[i], finger_length_ij * 0.15f);
                float curve_far = util::contourCurvature(contour, fingerTipCands[i], finger_length_ij * 0.45f);
#ifdef DEBUG
                cv::Scalar txtColorNear, txtColorFar;
                txtColorFar = txtColorNear = cv::Scalar(150, 150, 150);
                if (curve_near < params->fingerCurveNearMin) {
                    txtColorNear = cv::Scalar(0, 0, 255);
                }
                if (curve_far < params->fingerCurveFarMin) {
                    txtColorFar = cv::Scalar(0, 0, 255);
                }

                cv::putText(visual,
                    std::to_string(curve_near), finger_ij + topLeftPt + Point2i(0, 10),
                    0, 0.3, txtColorNear, 1);
                cv::putText(visual,
                    std::to_string(curve_far) + "F", finger_ij + topLeftPt + Point2i(0, -10),
                    0, 0.3, txtColorFar, 1);
#endif

                if (finger_length < params->fingerLenMax && finger_length > params->fingerLenMin &&
                    finger_defect_slope > params->fingerDefectSlopeMin &&
                    finger_center_slope > params->fingerCenterSlopeMin &&
                    centroid_defect_finger_angle > params->centroidDefectFingerAngleMin &&
                    finger_xyz[2] != 0 && curve_near >= params->fingerCurveNearMin &&
                    curve_far >= params->fingerCurveFarMin)
                {

                    fingerTipsXyz.push_back(finger_xyz);
                    fingerTipsIj.push_back(finger_ij + topLeftPt);
                    fingerDefectsIj.push_back(defect_ij + topLeftPt);
                    fingerTipsIdx.push_back(fingerTipCands[i]);
                    fingerDefectsIdx.push_back(fingerDefectCands[i]);
                }
            }
        }

        std::vector<int> fingerTipsIdxFiltered, defects_idx_filtered;

        // threshold out close fingers & fingers near the very edge
        for (int i = 0; i < fingerTipsXyz.size(); ++i) {
            double mindist = DBL_MAX;

            for (int j = 0; j < fingerTipsXyz.size(); ++j) {
                if (fingerTipsXyz[i][1] > fingerTipsXyz[j][1] ||
                    (fingerTipsXyz[i][1] == fingerTipsXyz[j][1] && i >= j)) continue;

                double dist = util::euclideanDistance(fingerTipsXyz[i], fingerTipsXyz[j]);
                if (dist < mindist) {
                    mindist = dist;
                    if (mindist < params->fingerDistMin) break;
                }
            }

            // remove this finger
            if (mindist < params->fingerDistMin) continue;

            // push to output
            this->fingersIJ.push_back(fingerTipsIj[i]);
            this->fingersXYZ.push_back(fingerTipsXyz[i]);
            fingerTipsIdxFiltered.push_back(fingerTipsIdx[i]);

            this->defectsIJ.push_back(fingerDefectsIj[i]);
            Vec3f defXyz = util::averageAroundPoint(xyzMap, fingerDefectsIj[i] - topLeftPt,
                params->xyzAverageSize);
            this->defectsXYZ.push_back(defXyz);
            defects_idx_filtered.push_back(fingerDefectsIdx[i]);
        }

        // special case for one or less visible fingers
        if (this->fingersXYZ.size() <= 1)
        {
            this->fingersXYZ.clear();
            this->fingersIJ.clear();
            fingerTipsIdxFiltered.clear();

            // find candidate for index finger
            Point2i indexFinger_ij, indexFinger_right, indexFinger_left;
            int indexFinger_idx;
            double farthest = 0;

            // find farthest point on the convex hull & record the points to the left and right of it
            if (convexHull.size() > 1)
            {
                for (int i = 0; i < convexHull.size(); ++i)
                {
                    Point2i convexPt = convexHull[i];

                    if (util::pointOnEdge(fullMapSize, convexPt, params->bottomEdgeThresh,
                        params->sideEdgeThresh)) continue;

                    Vec3f convexPt_xyz = util::averageAroundPoint(xyzMap, convexPt - topLeftPt, 10);

                    double dist = util::euclideanDistance(convexPt_xyz, this->palmCenterXYZ);
                    double slope = (double)(this->palmCenterIJ.y - convexPt.y) / abs(convexPt.x - this->palmCenterIJ.x);

                    if (slope > -0.1 &&
                        convexPt.y < fullMapSize.height - 10 && // cut off bottom points
                        dist > farthest)
                    {
                        farthest = dist;
                        indexFinger_ij = convexPt;
                        indexFinger_idx = indexHull[i];
                        indexFinger_right = convexHull[(i + 1) % convexHull.size()];
                        indexFinger_left = convexHull[(i - 1 + convexHull.size()) % convexHull.size()];
                    }
                }
            }

            indexFinger_ij = util::nearestPointOnCluster(xyzMap, indexFinger_ij - topLeftPt, 10000) + topLeftPt;

            Vec3f indexFinger_xyz =
                util::averageAroundPoint(xyzMap, indexFinger_ij - topLeftPt, 10);

            double angle = util::angleBetweenPoints(indexFinger_left, indexFinger_right, indexFinger_ij);

            this->defectsIJ.clear(); this->defectsXYZ.clear(); defects_idx_filtered.clear();

            if (angle <= params->singleFingerAngleThresh ||
                util::pointOnEdge(fullMapSize, indexFinger_ij, params->bottomEdgeThresh,
                    params->sideEdgeThresh) || goodDefects.size() == 0) {
                // angle too small or point on edge
#ifdef DEBUG
                std::cerr << "[Hand Debug]: Single finger angle too small or point on edge\n";
#endif
            }
            else {
                this->fingersXYZ.push_back(indexFinger_xyz);
                this->fingersIJ.push_back(indexFinger_ij);
                fingerTipsIdxFiltered.push_back(indexFinger_idx);

                double best = DBL_MAX;
                Point2i bestDef;
                Vec3f bestXyz;
                int bestIdx = -1;

                for (int j = 0; j < goodDefects.size(); ++j) {
                    cv::Vec4i defect = defects[goodDefects[j]];
                    Point2i farPt = contour[defect[2]] - topLeftPt;
                    Vec3f far_xyz =
                        util::averageAroundPoint(xyzMap, farPt, params->xyzAverageSize);

                    farPt = util::nearestPointOnCluster(xyzMap, farPt);

                    double dist = util::euclideanDistance(far_xyz, indexFinger_xyz);

                    if (dist > params->singleFingerLenMin && dist < best) {
                        best = dist;
                        bestDef = farPt;
                        bestXyz = far_xyz;
                        bestIdx = defect[2];
                    }
                }

                if (best == DBL_MAX) {
                    this->defectsIJ.push_back(this->palmCenterIJ);
                    this->defectsXYZ.push_back(this->palmCenterXYZ);
                    defects_idx_filtered.push_back(-1);
                }
                else {
                    this->defectsIJ.push_back(bestDef + topLeftPt);
                    this->defectsXYZ.push_back(bestXyz);
                    defects_idx_filtered.push_back(bestIdx);
                }

                // filter by curvature
                float finger_length_ij =
                    util::euclideanDistance(indexFinger_ij, bestDef + topLeftPt);
                float curve_near = util::contourCurvature(contour, indexFinger_idx, finger_length_ij * 0.15f);
                float curve_far = util::contourCurvature(contour, indexFinger_idx, finger_length_ij * 0.45f);

#ifdef DEBUG
                cv::Scalar txtColorNear = cv::Scalar(0, 255, 255);
                cv::Scalar txtColorFar = cv::Scalar(0, 255, 255);
                if (curve_near < params->fingerCurveNearMin) {
                    txtColorNear = cv::Scalar(0, 0, 190);
                }
                if (curve_far < params->fingerCurveFarMin) {
                    txtColorFar = cv::Scalar(0, 0, 190);
                }

                cv::rectangle(visual, cv::Rect(bestDef.x + topLeftPt.x - 5,
                    bestDef.y + topLeftPt.y - 5, 10, 10),
                    cv::Scalar(255, 0, 0), 2);

                cv::putText(visual,
                    std::to_string(curve_far) + "F", indexFinger_ij + Point2i(0, 10),
                    0, 0.5, txtColorFar, 1);

                cv::putText(visual,
                    std::to_string(curve_near), indexFinger_ij + Point2i(0, -10),
                    0, 0.5, txtColorNear, 1);
#endif

                if (curve_near < params->fingerCurveNearMin ||
                    curve_far < params->fingerCurveFarMin) {
                    this->fingersIJ.clear(); this->fingersXYZ.clear();
                    this->defectsIJ.clear(); this->defectsXYZ.clear();
                }
                else {
                    double fingerLen = util::euclideanDistance(indexFinger_xyz, this->defectsXYZ[0]);
                    // too long or too short
                    if (fingerLen > params->singleFingerLenMax || fingerLen < params->singleFingerLenMin) {
                        this->fingersXYZ.clear(); this->fingersIJ.clear();
                        this->defectsIJ.clear(); this->defectsXYZ.clear();
                        fingerTipsIdxFiltered.clear(); defects_idx_filtered.clear();
                    }
                }
            }
        }

#ifdef DEBUG
        cv::imshow("[Hand Debug]", visual);
#endif

        int nFin = (int) this->fingersIJ.size();

        // report not hand if there are too few/many fingers
        if (nFin > 6 || nFin < 1) {
#ifdef DEBUG
            //std::cerr << "[Hand Debug]: OBJECT ELIMINATED BECAUSE NOT ENOUGH FINGERS (" << nFin << ")\n";
#endif
            return false;
        }

        return true;
    }

    void Hand::checkEdgeConnected()
    {
        int cols = fullMapSize.width, rows = fullMapSize.height;

        // Bottom Sweep
        int row = rows - params->bottomEdgeThresh - topLeftPt.y, col;

        if (row >= 0 && row < xyzMap.rows) {
            for (col = 0; col < std::min(cols / 2 - topLeftPt.x, xyzMap.cols); ++col)
            {
                if (xyzMap.at<Vec3f>(row, col)[2] != 0)
                {
                    leftEdgeConnected = true;
                    break;
                }
            }
        }

        if (!leftEdgeConnected) {
            // Left Side Sweep
            col = params->sideEdgeThresh - topLeftPt.x;
            if (col >= 0 && col < xyzMap.cols) {
                for (row = std::min(rows - 1 - topLeftPt.y, xyzMap.rows - 1);
                    row >= std::max(rows * params->handEdgeConnectMaxY - topLeftPt.y, 0.0); --row)
                {
                    if (xyzMap.at<Vec3f>(row, col)[2] != 0)
                    {
                        leftEdgeConnected = true;
                        break;
                    }
                }
            }
        }

        // Bottom Sweep
        row = rows - params->bottomEdgeThresh - topLeftPt.y;
        if (row >= 0 && row < xyzMap.rows) {
            for (col = cols / 2 - topLeftPt.x; col < cols - topLeftPt.x; ++col)
            {
                if (col < 0 || col >= xyzMap.cols) continue;
                if (xyzMap.at<Vec3f>(row, col)[2] != 0)
                {
                    rightEdgeConnected = true;
                    break;
                }
            }
        }

        if (!rightEdgeConnected) {
            // Right Side Sweep
            col = cols - params->sideEdgeThresh - topLeftPt.x;
            if (col >= 0 && col < xyzMap.cols) {
                for (row = std::min(rows - 1 - topLeftPt.y, xyzMap.rows - 1);
                    row >= std::max(rows * params->handEdgeConnectMaxY - topLeftPt.y, 0.0); --row)
                {
                    if (row < 0 || row >= xyzMap.rows) continue;
                    if (xyzMap.at<Vec3f>(row, col)[2] != 0)
                    {
                        rightEdgeConnected = true;
                        break;
                    }
                }
            }
        }
    }

    int Hand::touchingPlane(const FramePlane & plane, std::vector<int> & output,
        double threshold, bool extrapolate) const
    {
        output.clear();

        for (int i = 0; i < getNumFingers(); i++)
        {
            bool touching = plane.touching(fingersXYZ[i], fingersIJ[i], threshold, !extrapolate);
            if (touching) {
                output.push_back(i);
            }
        }

        return (int)output.size();
    }

    int Hand::touchingPlanes(const std::vector<std::shared_ptr<FramePlane>>& planes,
        std::vector<std::pair<int, std::vector<int> >>& output, double threshold, bool extrapolate) const
    {
        output.clear();

        for (int i = 0; i < getNumFingers(); i++)
        {
            bool first = true;

            for (int j = 0; j < planes.size(); ++j) {
                const FramePlane & plane = *planes[j];

                bool touching = plane.touching(fingersXYZ[i], fingersIJ[i],
                    threshold, !extrapolate);

                if (touching) {
                    if (first) {
                        output.push_back({ i, std::vector<int>() });
                        first = false;
                    }

                    output.rbegin()->second.push_back(j);
                }
            }
        }

        return (int)output.size();
    }

    const Vec3f & Hand::getPalmCenter() const
    {
        return palmCenterXYZ;
    }

    const Point2i & Hand::getPalmCenterIJ() const
    {
        return palmCenterIJ;
    }

    const std::vector<Vec3f> & Hand::getFingers() const
    {
        return fingersXYZ;
    }

    const std::vector<Point2i> & Hand::getFingersIJ() const
    {
        return fingersIJ;
    }

    const std::vector<Vec3f> & Hand::getDefects() const
    {
        return defectsXYZ;
    }

    const std::vector<Point2i> & Hand::getDefectsIJ() const
    {
        return defectsIJ;
    }

    const std::vector<Vec3f> & Hand::getWrist() const
    {
        return wristXYZ;
    }

    const std::vector<Point2i> & Hand::getWristIJ() const
    {
        return wristIJ;
    }

    double Hand::getCircleRadius() const
    {
        return circleRadius;
    }

    Point2f Hand::getDominantDirection() const
    {
        return dominantDir;
    }

    float Hand::getSVMConfidence() const
    {
        return svmConfidence;
    }

    bool Hand::isValidHand() const
    {
        return isHand;
    }

    bool Hand::touchingEdge() const
    {
        return leftEdgeConnected || rightEdgeConnected;
    }

    bool Hand::touchingLeftEdge() const
    {
        return leftEdgeConnected;
    }

    bool Hand::touchingRightEdge() const
    {
        return rightEdgeConnected;
    }
}
