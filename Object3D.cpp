#include "stdafx.h"
#include "version.h"

#include "Object3D.h"
#include "Hand.h"
#include "Visualizer.h"
#include "Util.h"

#include "HandFeatureExtractor.h"

// limited to file scope
namespace {
    /**
    * Comparator for sorting defects in order of slope (only available in Object3D)
    */
    class DefectComparer {
    public:
        /**
        * Create a new comparator
        * @param contour the contour which the defects were computed from
        * @param defects list of defects
        * @param center center point from which slopes should be computed from
        */
        DefectComparer(std::vector<ark::Point2i> contour,
            std::vector<cv::Vec4i> defects, ark::Point2i center) {
            slope.resize(contour.size());

            for (unsigned i = 0; i < defects.size(); ++i) {
                ark::Point2i pt = contour[defects[i][ATTR_USED]] - center;
                slope[defects[i][ATTR_USED]] = ark::util::pointToSlope(pt);
            }
        }

        /**
        * Compare two defects (least counterclockwise from bottom is less)
        */
        bool operator()(cv::Vec4i a, cv::Vec4i b) const {
            int idxA = a[ATTR_USED], idxB = b[ATTR_USED];
            return slope[idxA] > slope[idxB];
        }

    private:
        /**
        * index of the Vec4i used for comparison
        */
        const int ATTR_USED = 2;

        /**
        * stores the slopes of all the points on the contour
        */
        std::vector<double> slope;

        // default constructor disabled
        DefectComparer() {};
    };
}

namespace ark {

    static const char * SVM_PATH = "svm/";

    // The SVM hand classifier
    classifier::HandClassifier & Object3D::handClassifier = classifier::SVMHandClassifier(SVM_PATH);

    // Amount image is scaled by for contour finding
    static const int IMG_SCALE = 2;

    Object3D::Object3D() { }

    Object3D::Object3D(cv::Mat depthMap, const ObjectParams * params) {
        if (params == nullptr) params = &ObjectParams();

        fullXyzMap = depthMap;
        fullMapSize = depthMap.size();
        points = new std::vector<Point2i>;
        points_xyz = new std::vector<Vec3f>;

        this->params = params;

        int minc = INT_MAX, minr = INT_MAX, maxc = 0, maxr = 0;
        for (int r = 0; r < depthMap.rows; ++r) {
            Vec3f * ptr = depthMap.ptr<Vec3f>(r);

            for (int c = 0; c < depthMap.cols; ++c) {
                if (ptr[c][2] > 0) {
                    points->push_back(Point2i(c, r));
                    points_xyz->push_back(Vec3f(*ptr));

                    minc = std::min(minc, c);
                    maxc = std::max(maxc, c);
                    minr = std::min(minr, r);
                    maxr = std::max(maxr, r);
                }
            }
        }

        topLeftPt = Point2i(minc, minr);
        fullXyzMap(cv::Rect(minc, minr, maxc - minc + 1, maxr - minr + 1)).copyTo(xyzMap);

        // compute surface area
        surfaceArea = util::surfaceArea(depthMap, *points, *points_xyz, true);

        // initialize object, detect hand, etc.
        initializeObject();
    }

    Object3D::Object3D(std::vector<Point2i> & points_ij,
        std::vector<Vec3f> & points_xyz,
        cv::Mat & depth_map,
        const ObjectParams * params,
        bool sorted, int points_to_use) {

        if (params == nullptr) params = &ObjectParams();

        if (points_to_use < 0 || points_to_use >(int)points_ij.size())
            num_points = (int)points_ij.size();
        else
            num_points = points_to_use;


        this->points = &points_ij;
        this->points_xyz = &points_xyz;

        if (!sorted) {
            util::radixSortPoints(points_ij, depth_map.cols, depth_map.rows, num_points, &points_xyz);
        }

        cv::Rect bounding(depth_map.cols, points_ij[0].y, -1, points_ij[num_points - 1].y);

        for (int i = 0; i < num_points; ++i) {
            Point2i pt = points_ij[i];
            //Vec3f v = depthMap.at<Vec3f>(pt);

            bounding.x = std::min(pt.x, bounding.x);
            bounding.width = std::max(pt.x, bounding.width);
        }

        bounding.width -= bounding.x - 1;
        bounding.height -= bounding.y - 1;

        xyzMap = cv::Mat::zeros(bounding.size(), depth_map.type());
        topLeftPt = Point2i(bounding.x, bounding.y);
        fullMapSize = depth_map.size();

        for (int i = 0; i < num_points; ++i) {
            Point2i pt = points_ij[i] - topLeftPt;
            xyzMap.at<Vec3f>(pt) = points_xyz[i];
        }

        this->params = params;

        surfaceArea = util::surfaceArea(depth_map, points_ij, points_xyz, true, num_points);

        // initialize object, detect hand, etc.
        initializeObject();
    }

    inline Point2i Object3D::findCenter(std::vector<Point2i> contour)
    {
        //using image moments to find center of mass of the object
        //Cx=M10/M00 and Cy=M01/M00
        cv::Moments M = cv::moments(contour, false);
        Point2i center = Point2i(static_cast<int>(M.m10) / M.m00, static_cast<int>(M.m01) / M.m00);
        return center;
    }

    std::vector<Point2i> Object3D::clusterConvexHull(std::vector<Point2i> convex_hull, int threshold)
    {
        std::vector<Point2i> clusterHull;
        int i = 0;

        while (i < convex_hull.size())
        {
            // Select a point from cluster
            std::vector<Point2i> cluster;
            Point2i hullPoint = convex_hull[i];
            cluster.push_back(hullPoint);
            i++;

            while (i < convex_hull.size())
            {
                Point2i clusterPoint = convex_hull[i];
                double distance = cv::norm(hullPoint - clusterPoint);

                if (distance < threshold)
                {
                    cluster.push_back(clusterPoint);
                    i++;
                }

                else
                {
                    break;
                }
            }

            hullPoint = cluster[cluster.size() / 2];
            Point2i center = findCenter(convex_hull);
            int maxDist = cv::norm(hullPoint - center);

            for (int j = 0; j < cluster.size(); j++)
            {

                if (cv::norm(cluster[j] - center) > maxDist)
                {
                    maxDist = cv::norm(cluster[j] - center);
                    hullPoint = cluster[j];
                }
            }
            clusterHull.push_back(hullPoint);
        }

        return clusterHull;
    }

    // compute contour, convex hull of this 3D object, and finds hands (saves duplication in constructors)
    void Object3D::initializeObject()
    {
        // step 1: initialize and compute properties
        checkEdgeConnected();

        // surface area criterion. need to move this if enabling plane detection
        if (surfaceArea >= params->handMinArea && surfaceArea <= params->handMaxArea) {
            // if not connected skip
            if (!params->handRequireEdgeConnected || leftEdgeConnected || rightEdgeConnected) {
               
                // Step 2: determine whether cluster is a hand
                this->hand = checkForHand();
                if (this->hand) {
                    hasHand = true;
                    return;
                }
            }
        }

#ifdef PLANE_ENABLED
        plane = boost::make_shared<Plane>(points, points_xyz, num_points);

        std::vector<cv::Point> plane_points = plane->getPlaneIndicies();

        // Step 3.1 If there is plane, remove plane and look for hand
        if (plane_points.size() != 0)
        {
            hasPlane = true;

            for (int i = 0; i < plane_points.size(); i++)
            {
                xyzMap.at<cv::Vec3f>(plane_points[i] - topLeftPt) = 0;
            }

            cv::Mat hand_cluster = cv::Mat::zeros(xyzMap.rows, xyzMap.cols, xyzMap.type());

            std::vector<cv::Point> hand_points (num_points);
            std::vector<cv::Vec3f> hand_points_xyz (num_points);

            centerIj.x = INT_MAX;
            getCenterIj();
            if (centerIj.x < 0 || centerIj.y < 0) return;

            //determining the pixels that are similar to center and connected to it
            int num_hand_pts = util::floodFill(centerIj.x - topLeftPt.x, centerIj.y - topLeftPt.y,
                                                xyzMap, &hand_points, &hand_points_xyz, 
                                                params->clusterMaxDistance, &hand_cluster);

            for (int i = 0; i < num_hand_pts; ++i) {
                hand_points[i] += topLeftPt;
            }

            hand = checkForHand(&hand_cluster, &hand_points, &hand_points_xyz, topLeftPt, num_hand_pts);

            if (hand != nullptr)
            {
                hasHand = true;
                return;
            }

        }
#endif

        // Step 3.1.1 If there is plane, no hand, then the rest of points are shape
        shape = xyzMap;
        hasShape = true;
    }

    boost::shared_ptr<Hand> Object3D::checkForHand(const cv::Mat * cluster,
        const std::vector<Point2i> * points,
        const std::vector<Vec3f> * points_xyz,
        Point topLeftPt,
        int num_points,
        const ObjectParams * params) 
    {
    
        // Begin visualization

        if (num_points < 0) num_points = this->num_points;

#if defined(DEBUG) && defined(DEMO)
        cv::Mat visual = cv::Mat::zeros(fullMapSize.height, fullMapSize.width, CV_8UC3);
#endif
        // End visualization

        // Default values
        if (cluster == nullptr) cluster = &this->xyzMap;
        if (points == nullptr) points = this->points;
        if (points_xyz == nullptr) points_xyz = this->points_xyz;
        if (params == nullptr) params = this->params;
        if (topLeftPt.x == INT_MAX) topLeftPt = this->topLeftPt;

        computeContour(*cluster, points, points_xyz, topLeftPt, num_points);

        // recompute convex hull based on new contour
        convexHull.clear(); getConvexHull();

        // Create empty hand instance
        boost::shared_ptr<Hand> hand = boost::make_shared<Hand>();

        // Begin by computing defects
        std::vector<cv::Vec4i> defects;

        if (indexHull.size() > 3)
        {
            std::vector<int> tmpHull;

            cv::convexityDefects(contour, indexHull, defects);
        }

        // ** Find center of contour **
        Point2i centroid = findCenter(contour) - topLeftPt;

        // Make sure center is on cluster
        centroid = util::nearestPointOnCluster(xyzMap, centroid);

        // Find radius and center point of largest inscribed circle above center

        // radius of largest inscribed circle
        double cirrad;
        // center of largest inscribed circle (coordinates from top left of FULL depth map)
        Point2i circen =
            util::largestInscribedCircle(contour, *points, *points_xyz, num_points,
                params->centerMaxDistFromTop,
                &cirrad);

        Point2i center = circen - topLeftPt;
        hand->center_ij = circen;
        hand->center_xyz = util::averageAroundPoint(*cluster, center, params->xyzAverageSize);
        hand->circle_radius = cirrad;

        // ** Find wrist positions **
        int wristL = -1, wristR = -1;
        Point2i wristL_ij, wristR_ij;
        Vec3f wristL_xyz, wristR_xyz;

        // 1. get leftmost & rightmost points on contour connected to the edge
        int contactL = -1, contactR = -1, direction = 1;
        Point2i contactL_ij, contactR_ij;
        Vec3f contactL_xyz, contactR_xyz;

        const int lMargin = params->contactSideEdgeThresh,
            rMargin = fullMapSize.width - params->contactSideEdgeThresh;

        for (unsigned i = 0; i < contour.size(); ++i) {
            Point2i pt = contour[i];

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

        if (contactL > 0 && contactR > 0) {
            contactL_ij = contour[contactL];
            contactR_ij = contour[contactR];
            contactL_xyz = util::averageAroundPoint(*cluster,
                contactL_ij - topLeftPt, params->xyzAverageSize);
            contactR_xyz = util::averageAroundPoint(*cluster,
                contactR_ij - topLeftPt, params->xyzAverageSize);

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
                Vec3f xyz = util::averageAroundPoint(*cluster,
                    pt - topLeftPt, params->xyzAverageSize);

                float dist = util::euclideanDistance(xyz, hand->center_xyz);

                if (dist <= params->wristCenterDistThresh) {
                    wristL = i;
                    break;
                }

                i = (int)(i + direction + contour.size()) % contour.size();
            } while (i != contactR);

            i = contactR;
            do {
                Point2i pt = contour[i];
                Vec3f xyz = util::averageAroundPoint(*cluster,
                    pt - topLeftPt, params->xyzAverageSize);

                float dist = util::euclideanDistance(xyz, hand->center_xyz);

                if (dist <= params->wristCenterDistThresh) {
                    wristR = i;
                    break;
                }

                i = (int)(i - direction + contour.size()) % contour.size();
            } while (i != contactL);
        }

        if (wristL < 0 || wristR < 0) {
            //
            // Begin visualization
#if defined(DEBUG) && defined(DEMO)
            //cv::putText(visual, "WRIST NOT FOUND", Point2i(10, 30), 0, 0.5, 255);
            //cv::imshow("[Debug]", visual);
#endif
            // End visualization //*/
            return nullptr;
        }

        wristL_ij = contour[wristL];
        wristR_ij = contour[wristR];
        wristL_xyz = util::averageAroundPoint(*cluster,
            wristL_ij - topLeftPt, params->xyzAverageSize);
        wristR_xyz = util::averageAroundPoint(*cluster,
            wristR_ij - topLeftPt, params->xyzAverageSize);

        float wristWidth = util::euclideanDistance(wristL_xyz, wristR_xyz);
        float contactWidth = util::euclideanDistance(contactL_xyz, contactR_xyz);

        Point2i wristM_ij = (contour[wristL] + contour[wristR]) / 2;
        Vec3f wristM_xyz = util::averageAroundPoint(*cluster,
            wristM_ij - topLeftPt, params->xyzAverageSize);

        double wristAngle = util::angleBetween3DVec(wristL_xyz, wristR_xyz, wristM_xyz);

        // output wrist points to hand
        hand->wrist_ij[0] = wristL_ij;
        hand->wrist_ij[1] = wristR_ij;
        hand->wrist_xyz[0] = wristL_xyz;
        hand->wrist_xyz[1] = wristR_xyz;

        // Eliminate by wrist & contact width
        if (wristWidth < params->wristWidthMin || wristWidth > params->wristWidthMax ||
            contactWidth < params->contactWidthMin ||
            contactWidth > params->contactWidthMax ||
            wristAngle > params->wristAngleMax ||
            wristAngle < params->wristAngleMin) {
            //
            // Begin visualization
#if defined(DEBUG) && defined(DEMO)
            //cv::putText(visual, "ELIMINATED BY WRIST/CONTACT WIDTH", Point2i(10, 30), 0, 0.5, 255);
            //cv::putText(visual, "WW:" + std::to_string(wristWidth), Point2i(10, 55), 0, 0.5, 255);
            //cv::putText(visual, "CW: " + std::to_string(contactWidth), Point2i(10, 80), 0, 0.5, 255);
            //cv::putText(visual, "WA: " + std::to_string(wristAngle), Point2i(10, 105), 0, 0.5, 255);
            //cv::imshow("[Debug]", visual);
#endif
            // End visualization //*/
            return nullptr;
        }

        // ** Detect fingers **
        // sort all defects found by angle
        DefectComparer comparer(contour, defects, hand->center_ij);
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

            // skip if defect is under wrist
            if (direction == -1) {
                if (wristL <= wristR) {
                    if (defect[2] >= wristL && defect[2] <= wristR) continue;
                }
                else if (defect[2] <= wristR || defect[2] >= wristL) continue;
            }
            else {
                if (wristL <= wristR) {
                    if (defect[2] <= wristL || defect[2] >= wristR) continue;
                }
                else if (defect[2] >= wristR && defect[2] <= wristL) continue;
            }

            // point on convex hull where defect begins. fingertip candidate
            Point2i start = contour[defect[0]] - topLeftPt;

            // point on convex hull where defect ends. fingertip candidate
            Point2i end = contour[defect[1]] - topLeftPt;

            // farthest point in the defect from the convex hull
            Point2i farPt = contour[defect[2]] - topLeftPt;

            // snap to nearest point actually on the cluster (should already be, just in case)
            start = util::nearestPointOnCluster(*cluster, start);
            end = util::nearestPointOnCluster(*cluster, end);
            farPt = util::nearestPointOnCluster(*cluster, farPt);

            // if any of the points is somehow out of the image, skip
            if (!util::pointInImage(*cluster, farPt) ||
                !util::pointInImage(*cluster, start) ||
                !util::pointInImage(*cluster, end)) continue;

            // obtain xyz positions of points
            Vec3f far_xyz = util::averageAroundPoint(*cluster, farPt, params->xyzAverageSize);
            Vec3f start_xyz = util::averageAroundPoint(*cluster, start, params->xyzAverageSize);
            Vec3f end_xyz = util::averageAroundPoint(*cluster, end, params->xyzAverageSize);

            // compute some distances used in heuristics
            double farCenterDist = util::euclideanDistance(far_xyz, hand->center_xyz);
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

        // <- Add a * between the two slashes to comment out whole visualization block
        // Begin visualization
#if defined(DEBUG) && defined(DEMO)
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

        cv::putText(visual, std::to_string(wristWidth),
            wristL_ij + Point2i(10, 0), 0, 0.5, cv::Scalar(255, 255, 255));
        cv::putText(visual, std::to_string(contactWidth),
            contactL_ij + Point2i(10, 0), 0, 0.5, cv::Scalar(255, 255, 255));
#endif
        // End visualization //*/

        // Eliminate by wrist & contact width
        if (wristWidth < params->wristWidthMin || wristWidth > params->wristWidthMax ||
            contactWidth < params->contactWidthMin ||
            contactWidth > params->contactWidthMax) {
            //
            // Begin visualization
#if defined(DEBUG) && defined(DEMO)
            //cv::putText(visual, "ELIMINATED BY WRIST/CONTACT WIDTH", Point2i(10, 30), 0, 0.5, 255);
            //cv::putText(visual, "WW:" + std::to_string(wristWidth), Point2i(10, 55), 0, 0.5, 255);
            //cv::putText(visual, "CW: " + std::to_string(contactWidth), Point2i(10, 80), 0, 0.5, 255);
            //cv::putText(visual, "WA: " + std::to_string(wristAngle), Point2i(10, 105), 0, 0.5, 255);
            //cv::imshow("[Debug]", visual);
#endif
            // End visualization //*/
            return nullptr;
        }

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

                Vec3f finger_xyz = util::averageAroundPoint(*cluster, finger_ij, params->xyzAverageSize);
                Vec3f defect_xyz = util::averageAroundPoint(*cluster, defect_ij, params->xyzAverageSize);

                // compute a number of features used to eliminate finger candidates
                double finger_length = util::euclideanDistance(finger_xyz, defect_xyz);
                double centroid_defect_dist = util::euclideanDistance(hand->center_xyz, defect_xyz);
                double finger_defect_slope = (double)(defect_ij.y - finger_ij.y) / abs(defect_ij.x - finger_ij.x);
                double finger_center_slope = (double)(center.y - finger_ij.y) / abs(center.x - finger_ij.x);
                double centroid_defect_finger_angle =
                    util::angleBetweenPoints(finger_ij, center, defect_ij);

                // number of points to the defect
                int points_to_defect = std::min(abs(fingerDefectCands[i] - fingerTipCands[i]),
                    std::min(fingerDefectCands[i], fingerTipCands[i]) +
                    (int)contour.size() -
                    std::max(fingerDefectCands[i], fingerTipCands[i]));

                if (points_to_defect < 10)
                    continue;

                int curve_near_lo = std::max(2, points_to_defect / 20);
                int curve_near_hi = curve_near_lo + 4;
                int curve_mid_lo = std::max(2, points_to_defect / 5);
                int curve_mid_hi = curve_mid_lo + 5;
                int curve_far_lo = std::max(2, points_to_defect * 9 / 10);
                int curve_far_hi = curve_far_lo + 5;

                double curve_near = util::contourCurvature(contour, fingerTipCands[i],
                    curve_near_lo, curve_near_hi);
                double curve_mid = util::contourCurvature(contour, indexHull[i],
                    curve_mid_lo, curve_mid_hi);
                double curve_far = util::contourCurvature(contour, fingerTipCands[i],
                    curve_far_lo, curve_far_hi);

                curve_far = std::min(curve_mid, curve_far);
                //
                // Begin visualization
#if defined(DEBUG) && defined(DEMO)
                cv::Scalar txtColor = cv::Scalar(255, 255, 255);
                if (!(curve_near >= params->fingerCurveNearMin &&
                    curve_near <= params->fingerCurveNearMax &&
                    curve_far >= params->fingerCurveFarMin &&
                    curve_far <= params->fingerCurveFarMax)) {
                    txtColor = cv::Scalar(0, 0, 255);
                }

                cv::putText(visual,
                    std::to_string(curve_far), finger_ij + topLeftPt + Point2i(0, 10),
                    0, 0.5, txtColor, 1);

                cv::putText(visual,
                    std::to_string(curve_near), finger_ij + topLeftPt + Point2i(0, -10),
                    0, 0.5, txtColor, 1);
#endif
                // End visualization //*/

                if (finger_length < params->fingerLenMax && finger_length > params->fingerLenMin &&
                    finger_defect_slope > params->fingerDefectSlopeMin &&
                    finger_center_slope > params->fingerCenterSlopeMin &&
                    centroid_defect_finger_angle > params->centroidDefectFingerAngleMin &&
                    finger_xyz[2] != 0 &&
                    curve_near >= params->fingerCurveNearMin &&
                    curve_near <= params->fingerCurveNearMax &&
                    curve_far >= params->fingerCurveFarMin &&
                    curve_far <= params->fingerCurveFarMax)
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
                if (fingerTipsXyz[i][1] >= fingerTipsXyz[j][1]) continue;

                double dist = util::euclideanDistance(fingerTipsXyz[i], fingerTipsXyz[j]);
                if (dist < mindist) {
                    mindist = dist;
                    if (mindist < params->fingerDistMin) break;
                }
            }

            // remove this finger 
            if (mindist < params->fingerDistMin) continue;

            hand->fingers_ij.push_back(fingerTipsIj[i]);
            hand->fingers_xyz.push_back(fingerTipsXyz[i]);
            fingerTipsIdxFiltered.push_back(fingerTipsIdx[i]);

            hand->defects_ij.push_back(fingerDefectsIj[i]);
            Vec3f defXyz = util::averageAroundPoint(*cluster, hand->defects_ij[i] - topLeftPt,
                params->xyzAverageSize);
            hand->defects_xyz.push_back(defXyz);
            defects_idx_filtered.push_back(fingerDefectsIdx[i]);
        }

        // special case for one or less visible fingers
        if (hand->fingers_xyz.size() <= 1)
        {
            hand->fingers_xyz.clear();
            hand->fingers_ij.clear();
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

                    Vec3f convexPt_xyz = util::averageAroundPoint(*cluster, convexPt - topLeftPt, 22);

                    double dist = util::euclideanDistance(convexPt_xyz, hand->center_xyz);
                    double slope = (double)(hand->center_ij.y - convexPt.y) / abs(convexPt.x - hand->center_ij.x);

                    if (slope > -0.1 &&
                        convexPt.y < fullMapSize.height - 10 && // cut off bottom points
                        dist > farthest)
                    {
                        farthest = dist;
                        indexFinger_ij = convexPt;
                        indexFinger_idx = indexHull[i];
                        indexFinger_right = convexHull[(i + 1) % convexHull.size()];
                        indexFinger_left = convexHull[(i - 1) % convexHull.size()];
                    }
                }
            }

            indexFinger_ij = util::nearestPointOnCluster(*cluster, indexFinger_ij - topLeftPt, 10000) + topLeftPt;

            Vec3f indexFinger_xyz =
                util::averageAroundPoint(*cluster, indexFinger_ij - topLeftPt, 10);

            double angle = util::angleBetweenPoints(indexFinger_left, indexFinger_right, indexFinger_ij);

            hand->defects_ij.clear(); hand->defects_xyz.clear(); defects_idx_filtered.clear();

            if (angle <= params->singleFingerAngleThresh ||
                util::pointOnEdge(fullMapSize, indexFinger_ij, params->bottomEdgeThresh,
                    params->sideEdgeThresh) || goodDefects.size() == 0) {
                // angle too large or point on edge
            }
            else {
                hand->fingers_xyz.push_back(indexFinger_xyz);
                hand->fingers_ij.push_back(indexFinger_ij);
                fingerTipsIdxFiltered.push_back(indexFinger_idx);

                double best = DBL_MAX;
                Point2i bestDef;
                Vec3f bestXyz;
                int bestIdx = -1;

                for (int j = 0; j < goodDefects.size(); ++j) {
                    cv::Vec4i defect = defects[goodDefects[j]];
                    Point2i farPt = contour[defect[2]] - topLeftPt;
                    Vec3f far_xyz =
                        util::averageAroundPoint(*cluster, farPt, params->xyzAverageSize);

                    farPt = util::nearestPointOnCluster(*cluster, farPt);

                    double dist = util::euclideanDistance(far_xyz, indexFinger_xyz);

                    if (dist > params->singleFingerLenMin && dist < best) {
                        best = dist;
                        bestDef = farPt;
                        bestXyz = far_xyz;
                        bestIdx = defect[2];
                    }
                }

                if (best == DBL_MAX) {
                    hand->defects_ij.push_back(hand->center_ij);
                    hand->defects_xyz.push_back(hand->center_xyz);
                    defects_idx_filtered.push_back(-1);
                }
                else {
                    hand->defects_ij.push_back(bestDef + topLeftPt);
                    hand->defects_xyz.push_back(bestXyz);
                    defects_idx_filtered.push_back(bestIdx);
                }


                int points_to_defect = std::min(abs(bestIdx - indexFinger_idx),
                    std::min(bestIdx, indexFinger_idx) + (int)contour.size() -
                    std::max(bestIdx, indexFinger_idx));

                if (points_to_defect < 10) {
                    hand->fingers_ij.clear(); hand->fingers_xyz.clear();
                    hand->defects_ij.clear(); hand->defects_xyz.clear();
                }
                else {
#ifndef PLANE_ENABLED
                    // filter by curvature
                    int curve_near_lo = std::max(2, points_to_defect / 20);
                    int curve_near_hi = curve_near_lo + 4;
                    int curve_mid_lo = std::max(2, points_to_defect / 5);
                    int curve_mid_hi = curve_mid_lo + 5;
                    int curve_far_lo = std::max(2, points_to_defect * 9 / 10);
                    int curve_far_hi = curve_far_lo + 5;

                    double curve_near = util::contourCurvature(contour, indexFinger_idx,
                        curve_near_lo, curve_near_hi);
                    double curve_mid = util::contourCurvature(contour, indexFinger_idx,
                        curve_mid_lo, curve_mid_hi);
                    double curve_far = util::contourCurvature(contour, indexFinger_idx,
                        curve_far_lo, curve_far_hi);

                    curve_far = std::min(curve_mid, curve_far);

                    // Begin visualization
#if defined(DEBUG) && defined(DEMO)
                    cv::Scalar txtColor = cv::Scalar(0, 255, 255);
                    if (!(curve_near >= params->fingerCurveNearMin &&
                        curve_near <= params->fingerCurveNearMax &&
                        curve_far >= params->fingerCurveFarMin &&
                        curve_far <= params->fingerCurveFarMax)) {
                        txtColor = cv::Scalar(0, 0, 190);
                    }

                    cv::putText(visual,
                        std::to_string(curve_far), indexFinger_ij + Point2i(0, 10),
                        0, 0.5, txtColor, 1);

                    cv::putText(visual,
                        std::to_string(curve_near), indexFinger_ij + Point2i(0, -10),
                        0, 0.5, txtColor, 1);
#endif
                    // End visualization //*/

                    if (!(curve_near >= params->fingerCurveNearMin &&
                        curve_near <= params->fingerCurveNearMax &&
                        curve_far >= params->fingerCurveFarMin &&
                        curve_far <= params->fingerCurveFarMax)) {
                        hand->fingers_ij.clear(); hand->fingers_xyz.clear();
                        hand->defects_ij.clear(); hand->defects_xyz.clear();
                    }
                    else {
#endif
                        double fingerLen = util::euclideanDistance(indexFinger_xyz, hand->defects_xyz[0]);
                        // too long or too short
                        if (fingerLen > params->singleFingerLenMax || fingerLen < params->singleFingerLenMin) {
                            hand->fingers_xyz.clear(); hand->fingers_ij.clear();
                            hand->defects_ij.clear(); hand->defects_xyz.clear();
                            fingerTipsIdxFiltered.clear(); defects_idx_filtered.clear();
                        }
#ifndef PLANE_ENABLED
                    }
#endif
                }
            }
        }

        //
        // Begin visualization
#if defined(DEBUG) && defined(DEMO)
        cv::imshow("[Debug]", visual);
#endif
        // End visualization //*/

        // report not hand if there are too few/many fingers
        if (hand->fingers_ij.size() > 6 || hand->fingers_ij.size() < 1) {
            return nullptr;
        }

        // Final SVM check
        if (params->handUseSVM && handClassifier.isTrained()) {
            this->hasHand = true;
            this->hand = hand;

            std::vector<double> features =
                classifier::features::extractHandFeatures(*this, *cluster, topLeftPt, 1.0,
                                                          fullMapSize.width);

            hand->svm_confidence = handClassifier.classify(features);
            if (hand->svm_confidence < params->handSVMConfidenceThresh) {
                // SVM confidence value below threshold, reverse decision & destroy the hand instance
                this->hasHand = false;
                this->hand = nullptr;
                hand = nullptr;
            }
        }

        return hand;
    }


    void Object3D::checkEdgeConnected()
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

    const Hand & Object3D::getHand() const
    {
        return *hand;
    }

    const Plane & Object3D::getPlane() const
    {
        return *plane;
    }

    Point2i Object3D::getCenterIj()
    {
        if (centerIj.x == INT_MAX)
            centerIj = util::findCentroid(xyzMap) + topLeftPt;
        return centerIj;
    }

    Vec3f Object3D::getCenter()
    {
        if (centerXyz[0] == FLT_MAX)
            centerXyz =
            util::averageAroundPoint(xyzMap, getCenterIj() - topLeftPt, params->xyzAverageSize);
        return centerXyz;
    }

    float Object3D::getDepth()
    {
        if (avgDepth == -1)
            avgDepth = util::averageDepth(xyzMap);
        return avgDepth;
    }

    cv::Mat Object3D::getShape() const
    {
        return shape;
    }

    double Object3D::getSurfArea() {
        if (surfaceArea == -1) {
            // lazily compute SA on demand
            surfaceArea = util::surfaceArea(getDepthMap(), *points, *points_xyz, true);
        }
        return surfaceArea;
    }

    cv::Rect Object3D::getBoundingBox() const
    {
        return cv::Rect(topLeftPt.x, topLeftPt.y, xyzMap.cols, xyzMap.rows);
    }



    std::vector<Point2i> Object3D::getContour()
    {
        if (points == nullptr) return std::vector<Point2i>();

        if (contour.size() == 0) {
            computeContour(xyzMap, points, points_xyz, topLeftPt, num_points);
        }

        return contour;
    }

    std::vector<Point2i> Object3D::getConvexHull()
    {
        if (points == nullptr) return std::vector<Point2i>();

        if (convexHull.size() == 0) {
            if (getContour().size() > 1)
            {
                // Find convex hull
                cv::convexHull(contour, convexHull, false, true);
                cv::convexHull(contour, indexHull, false, false);
            }
        }

        return convexHull;
    }

    const cv::Mat & Object3D::getDepthMap()
    {
        if (fullXyzMap.rows == 0) {
            // lazily compute full depth map on demand
            fullXyzMap = cv::Mat::zeros(fullMapSize, CV_32FC3);
            xyzMap.copyTo(fullXyzMap(getBoundingBox()));
        }

        return fullXyzMap;
    }

    // helper for performing morphological operations
    void Object3D::morph(int erode_sz, int dilate_sz, bool dilate_first, bool gray_map) {
        if (dilate_sz == -1) dilate_sz = erode_sz;

        cv::Mat eKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erode_sz, erode_sz)),
            dKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_sz, dilate_sz));

        if (gray_map) {
            if (dilate_first) cv::dilate(grayMap, grayMap, dKernel);
            cv::erode(grayMap, grayMap, eKernel);
            if (!dilate_first) cv::dilate(grayMap, grayMap, dKernel);
        }
        else {
            if (dilate_first) cv::dilate(xyzMap, xyzMap, dKernel);
            cv::erode(xyzMap, xyzMap, eKernel);
            if (!dilate_first) cv::dilate(xyzMap, xyzMap, dKernel);
        }
    }
    
    void Object3D::computeContour(const cv::Mat & xyzMap, const std::vector<cv::Point> * points,
        const std::vector<cv::Vec3f> * points_xyz,
        cv::Point topLeftPt,
        int num_points) {
        computeGrayMap(xyzMap, points, points_xyz, topLeftPt, num_points);

        std::vector<std::vector<Point2i> > contours;
        cv::findContours(grayMap, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, 2 * topLeftPt);

        int maxId = -1;

        for (int i = 0; i < (int)contours.size(); ++i)
        {
            if (maxId < 0 || contours[i].size() > contours[maxId].size())
            {
                maxId = i;
            }
        }

        if (maxId >= 0) cv::approxPolyDP(contours[maxId], contour, 0.002, true);
        else contour.push_back(Point2i(0, 0));

        for (int i = 0; i < (int)contour.size(); ++i)
            contour[i] /= IMG_SCALE;
    }

    void Object3D::computeGrayMap(const cv::Mat & xyzMap, const std::vector<cv::Point> * points, 
        const std::vector<cv::Vec3f> * points_xyz,
        cv::Point topLeftPt, int num_points, int thresh) {

        if (xyzMap.rows == 0 || xyzMap.cols == 0 || points == nullptr) return;

        grayMap = cv::Mat::zeros(xyzMap.size(), CV_8U);

        int points_to_use = num_points;
        if (points_to_use < 0) points_to_use = (int)points->size();

        for (int i = 0; i < points_to_use; ++i) {
            uchar val = (uchar)((*points_xyz)[i][2] * 256.0);
            if (val >= thresh)
                grayMap.at<uchar>((*points)[i] - topLeftPt) = val;
        }

        morph(2, 3, false, true);

        for (int i = 1; i < IMG_SCALE; i <<= 1)
            cv::pyrUp(grayMap, grayMap);
    }

    Object3D::~Object3D()
    {
        if (points != nullptr && num_points == -1) {
            delete points;
            delete points_xyz;
        }
    }
}
