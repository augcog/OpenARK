#include "stdafx.h"
#include "version.h"

#include "Object3D.h"
#include "Hand.h"
#include "Visualizer.h"
#include "Util.h"

namespace comparer{
    /**
    * Comparator for sorting defects in order of slope
    */
    class DefectComparer {
    public:
        /**
        * Create a new comparator
        * @param contour the contour which the defects were computed from
        * @param defects list of defects
        * @param center center point from which slopes should be computed from
        */
        DefectComparer(std::vector<cv::Point> contour,
            std::vector<cv::Vec4i> defects, cv::Point center) {
            slope.resize(contour.size());

            for (unsigned i = 0; i < defects.size(); ++i) {
                cv::Point pt = contour[defects[i][ATTR_USED]] - center;
                slope[defects[i][ATTR_USED]] = Util::pointToSlope(pt);
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

Object3D::Object3D() { }

Object3D::Object3D(cv::Mat depthMap, double min_size, double max_size) {
    xyzMap = fullXyzMap = depthMap;
    topLeftPt = cv::Point(0, 0);
    fullMapSize = depthMap.size();
    points = new std::vector<cv::Point>;

    for (int r = 0; r < depthMap.rows; ++r) {
        cv::Vec3f * ptr = depthMap.ptr<cv::Vec3f>(r);

        for (int c = 0; c < depthMap.cols; ++c) {
            if (ptr[c][2] > 0.1) {
                points->push_back(cv::Point(c, r));
            }
            //if (ptr[c][2] > maxVal) {
            //    maxVal = ptr[c][2];
            //    maxLoc = cv::Point(c, r);
            //}
        }
    }

    num_points = -1;

    getSurfArea();
    if (surfaceArea < min_size || surfaceArea > max_size) return;

    // initialize object, detect hand, etc.
    initializeObject(min_size, max_size);
}

Object3D::Object3D(std::vector<cv::Point> & points, cv::Mat & depthMap, bool sorted, int points_to_use,
                   double min_size, double max_size){

    if (points_to_use < 0 || points_to_use > (int)points.size()) 
        num_points = (int)points.size(); 
    else 
        num_points = points_to_use;

    static const Util::PointComparer<cv::Point> pc(false, true);

    if (!sorted)
        std::sort(points.begin(), points.begin() + num_points, pc);

    surfaceArea = Util::surfaceArea(depthMap, points, true, num_points);
    if (surfaceArea < min_size || surfaceArea > max_size) return;

    this->points = &points;

    cv::Rect bounding (depthMap.cols, points[0].y, -1, points[num_points-1].y);

    for (int i = 0; i < num_points; ++i) {
        cv::Point pt = points[i];
        //cv::Vec3f v = depthMap.at<cv::Vec3f>(pt);

        bounding.x = std::min(pt.x, bounding.x);
        bounding.width = std::max(pt.x, bounding.width);
    }

    if (bounding.x) --bounding.x;
    if (bounding.y) --bounding.y;

    bounding.width -= bounding.x - 2;
    bounding.height -= bounding.y - 2;

    xyzMap = cv::Mat::zeros(bounding.size(), depthMap.type());
    topLeftPt = cv::Point(bounding.x, bounding.y);
    fullMapSize = depthMap.size();
    
    for (int i = 0; i < num_points; ++i) {
        cv::Point pt = points[i] - topLeftPt;
        cv::Vec3f v = depthMap.at<cv::Vec3f>(points[i]);

        //if (v[2] > maxVal) {
        //    maxVal = v[2];
        //    maxLoc = pt;
        //}

        xyzMap.at<cv::Vec3f>(pt) = cv::Vec3f(v);
    }

    // initialize object, detect hand, etc.
    initializeObject(min_size, max_size);
}

inline cv::Point Object3D::findCenter(std::vector<cv::Point> contour)
{
    //using image moments to find center of mass of the object
    //Cx=M10/M00 and Cy=M01/M00
    cv::Moments M = cv::moments(contour, false);
    cv::Point center = cv::Point(static_cast<int>(M.m10) / M.m00, static_cast<int>(M.m01) / M.m00);
    return center;
}

std::vector<cv::Point> Object3D::clusterConvexHull(std::vector<cv::Point> convexHull, int threshold)
{
    std::vector<cv::Point> clusterHull;
    int i = 0;

    while (i < convexHull.size())
    {
        // Select a point from cluster
        std::vector<cv::Point> cluster;
        cv::Point hullPoint = convexHull[i];
        cluster.push_back(hullPoint);
        i++;

        while (i < convexHull.size())
        {
            cv::Point clusterPoint = convexHull[i];
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
        cv::Point center = findCenter(convexHull);
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
void Object3D::initializeObject(double min_size, double max_size)
{
    // step 1: initialize and compute properties
    checkEdgeConnected();

    // compute gray map
    computeGrayMap();

    // Find contour & convex hull
    getConvexHull();

    // Step 2: determine whether cluster is a hand
    if (hand = checkForHand(xyzMap)) {
        hasHand = true;
        return;
    }

    // TEMPORARILY disabling plane detection
    return;

    // Step 3: determine whether there is a plane
    plane = new Plane(xyzMap);

    // Step 3.1 If there is plane, remove plane and look for hand
    std::vector<cv::Point> pts = plane->getPlaneIndicies();

    if (pts.size() != 0)
    {
        hasPlane = true;

        for (int i = 0; i < pts.size(); i++)
        {
            int x = pts[i].x;
            int y = pts[i].y;
            xyzMap.at<cv::Vec3f>(y, x)[0] = 0;
            xyzMap.at<cv::Vec3f>(y, x)[1] = 0;
            xyzMap.at<cv::Vec3f>(y, x)[2] = 0;
        }

        cv::Mat hand_cluster = cv::Mat::zeros(xyzMap.rows, xyzMap.cols, xyzMap.type());

        //determining the pixels that are similar to center and connected to it
        Util::floodFill(centerIj.x, centerIj.y, xyzMap, hand_cluster, 0.02);

        if ((hand = checkForHand(hand_cluster)) != nullptr)
        {
            double finger_length = Util::euclideanDistance3D(hand->fingers_xyz[0], hand->centroid_xyz);
            if (finger_length > 0.03 && finger_length < 0.2)
            {
                hasHand = true;
                return;
            }
        }
    }

    // Step 3.1.1 If there is plane, no hand, then the rest of points are shape
    shape = xyzMap;
    hasShape = true;
}

Hand * Object3D::checkForHand(const cv::Mat & cluster, double angle_thresh, double cluster_thresh,
                              double finger_len_min, double finger_len_max, double max_defect_angle,
                              double finger_defect_slope_min, double finger_centroid_slope_min,
                              double finger_dist_min, double centroid_defect_finger_angle_min)
{

    // amount the depth image was scaled by in computeObjectFeatures
    const int IMG_SCALE = 2;

    std::vector<cv::Point> hull = clusterConvexHull(convexHull, cluster_thresh);
    std::vector<cv::Vec4i> defects;

    if (indexHull.size() > 3)
    {
        std::vector<int> tmpHull;

        cv::convexityDefects(complexContour, indexHull, defects);
    }

    // Find center of contour
    cv::Point center = findCenter(complexContour) / IMG_SCALE - topLeftPt;

    if (!Util::pointInImage(cluster, center, IMG_SCALE)) {
        return nullptr;
    }

    Hand * hand = new Hand();

    hand->centroid_xyz = Util::averageAroundPoint(cluster, center, 15);
    hand->centroid_ij = (center  + topLeftPt) * IMG_SCALE; // SCALING

    cv::Point index, index_right, index_left;
    double farthest = 0;

    if (hull.size() > 1)
    {
        for (int i = 0; i < hull.size(); ++i)
        {
            cv::Point p1 = hull[i];
            cv::Vec3f xyzP1 = Util::averageAroundPoint(cluster, p1 / IMG_SCALE - topLeftPt, 22);

            double dist = Util::euclideanDistance3D(xyzP1, hand->centroid_xyz);
            double slope = (double)(hand->centroid_ij.y - p1.y) / abs(p1.x - hand->centroid_ij.x);

            if (slope > finger_centroid_slope_min &&
                p1.y < fullMapSize.height * IMG_SCALE - 10 && // cut off bottom points
                dist > farthest)
            {
                farthest = dist;
                index = p1;
                index_right = hull[(i + 1) % hull.size()];
                index_left = hull[(i - 1) % hull.size()];
            }
        }
    }

    comparer::DefectComparer comparer(complexContour, defects, hand->centroid_ij);
    std::sort(defects.begin(), defects.end(), comparer);

    std::vector<cv::Point> endpoints, fingerDefects;

    //cv::Mat visualL = cv::Mat::zeros(cluster.rows, cluster.cols, cluster.type());
    cv::Vec3f lastPoint;

    bool first = true;

    for (int i = 0; i < defects.size(); ++i)
    {
        cv::Vec4i defect = defects[i];
        cv::Point start = complexContour[defect[0]] / IMG_SCALE - topLeftPt,
                  end = complexContour[defect[1]] / IMG_SCALE - topLeftPt,
                  farPt = complexContour[defect[2]] / IMG_SCALE - topLeftPt;

        //int depth = defect[3]; // Depth from edge of contour

        if (!Util::pointInImage(cluster, farPt)) continue;
        if (!Util::pointInImage(cluster, start)) continue;
        if (!Util::pointInImage(cluster, end)) continue;

        // Defect conditions: depth is sufficient, inside contour, y value is above center
        // (maxLoc largest depth) (removed)
        // first condition replace with meters distance from the edge
        // second test if inside the hull (removed)
        // not too far below the center (no change)

        if (//Util::magnitude(maxLoc - center) < depth * 2 &&
            //cv::pointPolygonTest(hull, farPt, false) > 0 &&
            farPt.y < center.y + 30 * IMG_SCALE &&
            farPt.y + topLeftPt.y < fullMapSize.height - 10)
        {

            double angle = Util::angleBetweenPoints(start, end, farPt); // The angle from start through farPt to end
            if (angle > max_defect_angle) continue;

            cv::Vec3f pt1 = cluster.at<cv::Vec3f>(farPt);
            cv::Vec3f start_xyz = Util::averageAroundPoint(cluster, start, 15),
                      end_xyz = Util::averageAroundPoint(cluster, end, 15);

            double dist = Util::euclideanDistance3D(pt1, hand->centroid_xyz);
            double startEndDist = Util::euclideanDistance3D(start_xyz, end_xyz);

            if (dist > 0.01 && startEndDist > 0.01)
            {
                if (first || Util::euclideanDistance3D(lastPoint, start_xyz) > 0.02) {
                    endpoints.push_back(start);
                    fingerDefects.push_back(farPt);
                    //cv::line(visualL, start, farPt, cv::Scalar(0, 255, 0), 2);
                    //cv::circle(visualL, start, 5, cv::Scalar(0, 255, 0));
                }

                endpoints.push_back(end);
                fingerDefects.push_back(farPt);
                //cv::line(visualL, end, farPt, cv::Scalar(0, 0, 255) , 2);

                first = false;
                lastPoint = end_xyz;
            }
        }

        //cv::circle(visualL, center, 2, cv::Scalar(255, 100, 255));
        //cv::circle(visualL, maxLoc, 2, cv::Scalar(255, 100, 0));
        //cv::circle(visualL, start, 2, cv::Scalar(0, 255, 0));
        //cv::line(visualL, start, farPt, cv::Scalar(0, 255, 0));
        //cv::circle(visualL, end, 3, cv::Scalar(0, 0, 255));
        //cv::circle(visualL, farPt, 2, cv::Scalar(255, 0, 0), 2);
        //cv::putText(visualL, std::to_string(i), farPt, 0, 0.5, cv::Scalar(255,255,255), 1);
        //cv::line(visualL, end, farPt, cv::Scalar(0, 0, 255));
    }

    //cv::namedWindow("Defects");
    //cv::imshow("Defects", visualL);

    std::vector<cv::Point> fingers_ij, defects_ij;
    std::vector<cv::Vec3f> fingers_xyz, defects_xyz;

    //// Cluster fingertip locations
    //endpoints = clusterConvexHull(endpoints, cluster_thresh);

    for (unsigned i = 0; i < endpoints.size(); i++)
    {
        cv::Point endpoint = endpoints[i];
        cv::Point relatedDefect = fingerDefects[i];

        cv::Vec3f endPoint_xyz = Util::averageAroundPoint(cluster, endpoint, 15),
             closestDefect_xyz = Util::averageAroundPoint(cluster, relatedDefect, 15);

        double finger_length = Util::euclideanDistance3D(endPoint_xyz, closestDefect_xyz) / IMG_SCALE;
        double centroid_defect_dist = Util::euclideanDistance3D(hand->centroid_xyz, closestDefect_xyz);

        endpoint += topLeftPt; relatedDefect += topLeftPt; center += topLeftPt;
        endpoint *= IMG_SCALE; relatedDefect *= IMG_SCALE; center *= IMG_SCALE;

        double finger_defect_slope = (double)(relatedDefect.y - endpoint.y) / abs(relatedDefect.x - endpoint.x);
        double finger_centroid_slope = (double)(center.y - endpoint.y) / abs(center.x - endpoint.x);
        double centroid_defect_finger_angle = Util::angleBetweenPoints(hand->centroid_ij, endpoint, relatedDefect);

        endpoint /= IMG_SCALE; relatedDefect /= IMG_SCALE; center /= IMG_SCALE;
        endpoint -= topLeftPt; relatedDefect -= topLeftPt; center -= topLeftPt;

        if (finger_length < finger_len_max && finger_length > finger_len_min &&
            finger_defect_slope > finger_defect_slope_min && finger_centroid_slope > finger_centroid_slope_min &&
            centroid_defect_finger_angle > centroid_defect_finger_angle_min && endPoint_xyz[2] != 0)
        {

            fingers_xyz.push_back(endPoint_xyz);
            fingers_ij.push_back((cv::Point2i(endpoint.x, endpoint.y) + topLeftPt) * IMG_SCALE); // SCALING
            defects_xyz.push_back(closestDefect_xyz);
            defects_ij.push_back((cv::Point2i(relatedDefect.x, relatedDefect.y) + topLeftPt) * IMG_SCALE); // SCALING
        }
    }

    // remove close fingers
    for (int i = 0; i < fingers_xyz.size(); ++i) {
        double mindist = DBL_MAX;

        for (int j = 0; j < fingers_xyz.size(); ++j) {
            if (fingers_xyz[i][1] >= fingers_xyz[j][1]) continue;
            
            double dist = Util::euclideanDistance3D(fingers_xyz[i], fingers_xyz[j]);
            if (dist < mindist) {
                mindist = dist;
            }
        }

        if (mindist < finger_dist_min) continue;

        hand->fingers_ij.push_back(fingers_ij[i]);
        hand->fingers_xyz.push_back(fingers_xyz[i]);
        hand->defects_ij.push_back(defects_ij[i]);
        hand->defects_xyz.push_back(defects_xyz[i]);
    }

    // If there is one or less visible fingers
    if (hand->fingers_xyz.size() <= 1)
    {
        hand->fingers_xyz.clear();
        hand->fingers_ij.clear();

        cv::Vec3f indexFinger = 
            Util::averageAroundPoint(cluster, index / IMG_SCALE - topLeftPt, 22);
            //cv::Vec3f defectxyz; cv::Point defectij;

            //if (hand->defects_ij.size() == 0) {
            double angle = Util::angleBetweenPoints(index_left, index_right, index);

            if (angle <= angle_thresh) {
                // angle too small
                delete hand;
                return nullptr;
            }

            //hand->centroid_xyz;
            //hand->centroid_ij;
        //}

        //else {
        //    defectxyz = hand->defects_xyz[0];
        //    defectij = hand->defects_ij[0];
        //}

        hand->fingers_xyz.push_back(indexFinger);
        hand->fingers_ij.push_back(cv::Point2i(index.x, index.y)); // SCALING

        hand->defects_ij.clear(); hand->defects_xyz.clear();

        // use centroid as "defect" for index finger
        hand->defects_ij.push_back(hand->centroid_ij);
        hand->defects_xyz.push_back(hand->centroid_xyz);
    }

    if (hand->defects_ij.size() == 0 || hand->fingers_ij.size() == 0 || hand->fingers_ij.size() > 6) {
        // Too many or too few fingers
        delete hand;
        return nullptr;
    }

    // TODO: add further hand checking here
    return hand;
}


void Object3D::checkEdgeConnected()
{
    int cols = fullMapSize.width, rows = fullMapSize.height;

    // bottom Sweep
    int r1 = rows * 9 / 10 - topLeftPt.y;
    if (r1 >= 0 && r1 < xyzMap.rows) {
        for (int c1 = 0; c1 < cols / 4 - topLeftPt.x; ++c1)
        {
            if (xyzMap.at<cv::Vec3f>(r1, c1)[2] != 0)
            {
                leftEdgeConnected = true;
                break;
            }
        }
    }

    // Left Side Sweep
    int c2 = cols * 2 / 10 - topLeftPt.x;
    if (c2 >= 0 && c2 < xyzMap.cols) {
        for (int r2 = 0; r2 < rows - topLeftPt.y; ++r2)
        {
            if (xyzMap.at<cv::Vec3f>(r2, c2)[2] != 0)
            {
                leftEdgeConnected = true;
                break;
            }
        }
    }

    // Bottom Sweep
    int r3 = rows * 9 / 10 - topLeftPt.y;
    if (r3 >= 0 && r3 < xyzMap.rows) {
        for (int c3 = cols - (cols / 4) - topLeftPt.x; c3 < cols - topLeftPt.x; ++c3)
        { 
            if (c3 < 0 || c3 >= xyzMap.cols) continue;
            if (xyzMap.at<cv::Vec3f>(r3, c3)[2] != 0)
            {
                rightEdgeConnected = true;
                break;
            }
        }
    }

    // Right Side Sweep
    int c4 = cols * 8 / 10 - topLeftPt.x;
    if (c4 >= 0 && c4 < xyzMap.cols) {
        for (int r4 = rows / 2 - topLeftPt.y; r4 < rows - topLeftPt.y; ++r4)
        {
            if (r4 < 0 || r4 >= xyzMap.rows) continue;
            if (xyzMap.at<cv::Vec3f>(r4, c4)[2] != 0)
            {
                rightEdgeConnected = true;
                break;
            }
        }
    }

}

Hand Object3D::getHand() const
{
    return *hand;
}

Plane Object3D::getPlane() const
{
    return *plane;
}

cv::Point Object3D::getCenterIj() 
{
    if (centerIj.x == INT_MAX)
        centerIj = Util::findCentroid(xyzMap) + topLeftPt;
    return centerIj;
}

cv::Vec3f Object3D::getCenter() 
{
    if (centerXyz[0] == FLT_MAX)
        centerXyz = Util::averageAroundPoint(xyzMap, getCenterIj() - topLeftPt, 15);
    return centerXyz;
}

float Object3D::getDepth() 
{
    if (avgDepth == -1)
        avgDepth = Util::averageDepth(xyzMap);
    return avgDepth;
}

cv::Mat Object3D::getShape() const
{
    return shape;
}

double Object3D::getSurfArea() {
    if (surfaceArea == -1) {
        // lazily compute SA on demand
        surfaceArea = Util::surfaceArea(getDepthMap(), *points, true);
    }
    return surfaceArea;
}

cv::Rect Object3D::getBoundingBox() const
{
    return cv::Rect(topLeftPt.x, topLeftPt.y, xyzMap.cols, xyzMap.rows);
}

std::vector<cv::Point> Object3D::getContour()
{
    if (complexContour.size() == 0) {
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(grayMap, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, 2*topLeftPt);

        unsigned maxId = 0;

        for (unsigned i = 1; i < contours.size(); ++i)
        {
            if (contours[i].size() > contours[maxId].size())
            {
                maxId = i;
            }
        }

        cv::approxPolyDP(contours[maxId], complexContour, 0.002, true);
    }

    return complexContour;
}

std::vector<cv::Point> Object3D::getConvexHull()
{
    if (convexHull.size() == 0) {
        getContour();
        
        if (complexContour.size() > 1)
        {
            // Find convex hull
            cv::convexHull(complexContour, convexHull, false, true);
            cv::convexHull(complexContour, indexHull, false, false);
        }
    }

    return convexHull;
}

const cv::Mat & Object3D::getDepthMap()
{
    if (fullXyzMap.rows == 0) {
        // lazily compute full depth map on demand
        fullXyzMap = cv::Mat::zeros(fullMapSize, CV_32FC3);
        xyzMap.copyTo(xyzMap, fullXyzMap(getBoundingBox()));
    }

    return fullXyzMap;
}

void Object3D::morph(int erodeAmt, int dilateAmt, bool dilateFirst, bool useGrayMap) {
    if (dilateAmt == -1) dilateAmt = erodeAmt;

    cv::Mat eKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeAmt, erodeAmt)),
            dKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilateAmt, dilateAmt));

    if (useGrayMap) {
        if (dilateFirst) cv::dilate(grayMap, grayMap, dKernel);
        cv::erode(grayMap, grayMap, eKernel);
        if (!dilateFirst) cv::dilate(grayMap, grayMap, dKernel);
    }
    else{
        if (dilateFirst) cv::dilate(xyzMap, xyzMap, dKernel);
        cv::erode(xyzMap, xyzMap, eKernel);
        if (!dilateFirst) cv::dilate(xyzMap, xyzMap, dKernel);
    }
}

void Object3D::computeGrayMap(int thresh) {
    grayMap =  cv::Mat::zeros(xyzMap.size(), CV_8U);

    int points_to_use = num_points;
    if (points_to_use < 0) points_to_use = (int)points->size();

    for (int i = 0; i < num_points; ++i) {
        uchar & gv = grayMap.at<uchar>((*points)[i] - topLeftPt);
        gv = (uchar)(xyzMap.at<cv::Vec3f>((*points)[i] - topLeftPt)[2] * 256.0);
        if (gv < thresh) 
            gv = 0;
    }

    morph(2, 3, false, true);
    cv::pyrUp(grayMap, grayMap);
}

Object3D::~Object3D()
{
    if (points != nullptr && num_points == -1) {
        delete points;
    }
}