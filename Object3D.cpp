#include "stdafx.h"
#include "version.h"

#include "Object3D.h"
#include "Hand.h"
#include "Visualizer.h"
#include "Util.h"

#include "HandFeatureExtractor.h"

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

static const char * SVM_PATH = "svm/";
// The SVM hand classifier
classifier::HandClassifier & Object3D::handClassifier = classifier::SVMHandClassifier(SVM_PATH);

// Amount image is scaled by for contour finding
static const int IMG_SCALE = 2;

Object3D::Object3D() { }

Object3D::Object3D(cv::Mat depthMap, double min_size, double max_size) {
    fullXyzMap = depthMap;
    fullMapSize = depthMap.size();
    points = new std::vector<cv::Point>;

    int minc = INT_MAX, minr = INT_MAX, maxc = 0, maxr = 0; 
    for (int r = 0; r < depthMap.rows; ++r) {
        cv::Vec3f * ptr = depthMap.ptr<cv::Vec3f>(r);

        for (int c = 0; c < depthMap.cols; ++c) {
            if (ptr[c][2] > 0) {
                points->push_back(cv::Point(c, r));
                minc = std::min(minc, c);
                maxc = std::max(maxc, c);
                minr = std::min(minr, r);
                maxr = std::max(maxr, r);
            }
        }
    }

    topLeftPt = cv::Point(minc, minr);
    fullXyzMap(cv::Rect(minc, minr, maxc - minc + 1, maxr - minr + 1)).copyTo(xyzMap);

    num_points = -1;

    surfaceArea = Util::surfaceArea(depthMap, *points, true);
    if (surfaceArea < min_size || surfaceArea > max_size) return;

    // initialize object, detect hand, etc.
    initializeObject(min_size, max_size);
}

Object3D::Object3D(std::vector<cv::Point> & points, cv::Mat & depth_map, bool sorted, int points_to_use,
                   double min_size, double max_size){

    if (points_to_use < 0 || points_to_use > (int)points.size()) 
        num_points = (int)points.size(); 
    else 
        num_points = points_to_use;

    if (!sorted)
        Util::radixSortPoints(points, depth_map.cols, depth_map.rows, num_points);
    
    surfaceArea = Util::surfaceArea(depth_map, points, true, num_points);

    if (surfaceArea < min_size || surfaceArea > max_size) return;

    this->points = &points;

    cv::Rect bounding (depth_map.cols, points[0].y, -1, points[num_points-1].y);

    for (int i = 0; i < num_points; ++i) {
        cv::Point pt = points[i];
        //cv::Vec3f v = depthMap.at<cv::Vec3f>(pt);

        bounding.x = std::min(pt.x, bounding.x);
        bounding.width = std::max(pt.x, bounding.width);
    }

    bounding.width -= bounding.x - 1;
    bounding.height -= bounding.y - 1;

    xyzMap = cv::Mat::zeros(bounding.size(), depth_map.type());
    topLeftPt = cv::Point(bounding.x, bounding.y);
    fullMapSize = depth_map.size();

    for (int i = 0; i < num_points; ++i) {
        cv::Point pt = points[i] - topLeftPt;
        cv::Vec3f v = depth_map.at<cv::Vec3f>(points[i]);

        xyzMap.at<cv::Vec3f>(pt) = v;
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

std::vector<cv::Point> Object3D::clusterConvexHull(std::vector<cv::Point> convex_hull, int threshold)
{
    std::vector<cv::Point> clusterHull;
    int i = 0;

    while (i < convex_hull.size())
    {
        // Select a point from cluster
        std::vector<cv::Point> cluster;
        cv::Point hullPoint = convex_hull[i];
        cluster.push_back(hullPoint);
        i++;

        while (i < convex_hull.size())
        {
            cv::Point clusterPoint = convex_hull[i];
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
        cv::Point center = findCenter(convex_hull);
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
void Object3D::initializeObject(double min_size, double max_size, double svm_confidence_thresh)
{
    // step 1: initialize and compute properties
    checkEdgeConnected();

    // Find contour & convex hull
    getConvexHull();

    // Step 2: determine whether cluster is a hand
    if (hand = checkForHand(xyzMap)) {
        hasHand = true;

        // Step 2.1: final SVM check
        std::vector<double> features = 
            classifier::features::extractHandFeatures(*this, xyzMap, topLeftPt, 1.0, fullMapSize.width);

        if ((hand->svm_confidence = handClassifier.classify(features)) < svm_confidence_thresh) {
            // SVM confidence value below threshold, reverse decision & destroy the hand instance
            delete hand;
            hand = nullptr;
            hasHand = false;
        }
        else return;
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

Hand * Object3D::checkForHand(const cv::Mat & cluster, 
                              double angle_thresh, double cluster_thresh,
                              double finger_len_min, double finger_len_max,
                              double single_finger_len_min, double single_finger_len_max,
                              double max_defect_angle,
                              double finger_defect_slope_min, double finger_centroid_slope_min,
                              double finger_dist_min, double centroid_defect_finger_angle_min) const
{    
    // hand instance
    Hand * hand = new Hand();

    std::vector<cv::Point> hull = clusterConvexHull(convexHull, cluster_thresh);
    std::vector<cv::Vec4i> defects;

    if (indexHull.size() > 3)
    {
        std::vector<int> tmpHull;

        cv::convexityDefects(complexContour, indexHull, defects);
    }

    // Find center of contour
    cv::Point center = findCenter(complexContour) - topLeftPt;

    // Make sure center is on cluster
    center = Util::findPointOnCluster(xyzMap, center);

    hand->centroid_ij = center + topLeftPt;
    hand->centroid_xyz = Util::averageAroundPoint(cluster, center, 15);

    cv::Point index, index_right, index_left;
    double farthest = 0;

    // find farthest point on the convex hull & record the points to the left and right of it
    if (hull.size() > 1)
    {
        for (int i = 0; i < hull.size(); ++i)
        {
            cv::Point p1 = hull[i];
            cv::Vec3f xyzP1 = Util::averageAroundPoint(cluster, p1 - topLeftPt, 22);

            double dist = Util::euclideanDistance3D(xyzP1, hand->centroid_xyz);
            double slope = (double)(hand->centroid_ij.y - p1.y) / abs(p1.x - hand->centroid_ij.x);

            if (slope > -0.1 &&
                p1.y < fullMapSize.height - 10 && // cut off bottom points
                dist > farthest)
            {
                farthest = dist;
                index = p1;
                index_right = hull[(i + 1) % hull.size()];
                index_left = hull[(i - 1) % hull.size()];
            }
        }
    }

    // sort all defects found by angle
    DefectComparer comparer(complexContour, defects, hand->centroid_ij);
    std::sort(defects.begin(), defects.end(), comparer);

    std::vector<cv::Point> endpoints, fingerDefects;

    //cv::Mat visualL = cv::Mat::zeros(cluster.rows, cluster.cols, cluster.type());
    cv::Vec3f lastPoint;

    bool first = true;

    // process defects
    for (int i = 0; i < defects.size(); ++i)
    {
        cv::Vec4i defect = defects[i];
        cv::Point start = complexContour[defect[0]] - topLeftPt,
                  end = complexContour[defect[1]] - topLeftPt,
                  farPt = complexContour[defect[2]] - topLeftPt;

        start = Util::findPointOnCluster(xyzMap, start);
        end = Util::findPointOnCluster(xyzMap, end);
        farPt = Util::findPointOnCluster(xyzMap, farPt);

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
            farPt.y < center.y + 30 &&
            farPt.y + topLeftPt.y < fullMapSize.height - 10)
        {

            // The angle from start through farPt to end
            double angle = Util::angleBetweenPoints(start, end, farPt); 
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

    // infer fingers from endpoints and defects
    std::vector<cv::Point> fingers_ij, defects_ij;
    std::vector<cv::Vec3f> fingers_xyz, defects_xyz;

    for (unsigned i = 0; i < endpoints.size(); i++)
    {
        cv::Point endpoint = endpoints[i];
        cv::Point relatedDefect = fingerDefects[i];

        cv::Vec3f endPoint_xyz = Util::averageAroundPoint(cluster, endpoint, 15),
             closestDefect_xyz = Util::averageAroundPoint(cluster, relatedDefect, 15);

        double finger_length = Util::euclideanDistance3D(endPoint_xyz, closestDefect_xyz);
        double centroid_defect_dist = Util::euclideanDistance3D(hand->centroid_xyz, closestDefect_xyz);

        endpoint += topLeftPt; relatedDefect += topLeftPt; center += topLeftPt;

        double finger_defect_slope = (double)(relatedDefect.y - endpoint.y) / abs(relatedDefect.x - endpoint.x);
        double finger_centroid_slope = (double)(center.y - endpoint.y) / abs(center.x - endpoint.x);
        double centroid_defect_finger_angle = Util::angleBetweenPoints(hand->centroid_ij, endpoint, relatedDefect);

        endpoint -= topLeftPt; relatedDefect -= topLeftPt; center -= topLeftPt;

        if (finger_length < finger_len_max && finger_length > finger_len_min &&
            finger_defect_slope > finger_defect_slope_min && finger_centroid_slope > finger_centroid_slope_min &&
            centroid_defect_finger_angle > centroid_defect_finger_angle_min && endPoint_xyz[2] != 0)
        {

            fingers_xyz.push_back(endPoint_xyz);
            fingers_ij.push_back(cv::Point2i(endpoint.x, endpoint.y) + topLeftPt);
            defects_xyz.push_back(closestDefect_xyz);
            defects_ij.push_back(cv::Point2i(relatedDefect.x, relatedDefect.y) + topLeftPt); // SCALING
        }
    }

    // threshold out close fingers
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

    // special case for one or less visible fingers
    if (hand->fingers_xyz.size() <= 1)
    {
        hand->fingers_xyz.clear();
        hand->fingers_ij.clear();

        index = Util::findPointOnCluster(cluster, index - topLeftPt, 10000) + topLeftPt;

        cv::Vec3f indexFinger = 
            Util::averageAroundPoint(cluster, index - topLeftPt, 10);
            //cv::Vec3f defectxyz; cv::Point defectij;

            //if (hand->defects_ij.size() == 0) {
        double angle = Util::angleBetweenPoints(index_left, index_right, index);
        double fingerLen = Util::euclideanDistance3D(indexFinger, hand->centroid_xyz);

        if (angle <= angle_thresh || fingerLen > single_finger_len_max || fingerLen < single_finger_len_min){
            // angle too small or too large
            delete hand;
            return nullptr;
        }

        hand->fingers_xyz.push_back(indexFinger);
        hand->fingers_ij.push_back(cv::Point2i(index.x, index.y));

        hand->defects_ij.clear(); hand->defects_xyz.clear();

        // use centroid as "defect" for index finger
        hand->defects_ij.push_back(hand->centroid_ij);
        hand->defects_xyz.push_back(hand->centroid_xyz);
    }

    // report not hand if there are too many or too few fingers
    if (hand->defects_ij.size() == 0 || hand->fingers_ij.size() == 0 || hand->fingers_ij.size() > 6) {
        delete hand;
        return nullptr;
    }

    return hand;
}


void Object3D::checkEdgeConnected()
{
    int cols = fullMapSize.width, rows = fullMapSize.height;

    // bottom Sweep
    int r1 = rows * 9 / 10 - topLeftPt.y;
    if (r1 >= 0 && r1 < xyzMap.rows) {
        for (int c1 = 0; c1 < std::min(cols / 4 - topLeftPt.x, xyzMap.cols); ++c1)
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
        for (int r2 = 0; r2 < std::min(rows - topLeftPt.y, xyzMap.rows); ++r2)
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
    if (points == nullptr) return std::vector<cv::Point>();

    if (complexContour.size() == 0) {
        computeGrayMap();

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(grayMap, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, 2*topLeftPt);

        int maxId = -1;

        for (int i = 0; i < (int)contours.size(); ++i)
        {
            if (maxId < 0 || contours[i].size() > contours[maxId].size())
            {
                maxId = i;
            }
        }

        if (maxId >= 0) cv::approxPolyDP(contours[maxId], complexContour, 0.002, true);
        else complexContour.push_back(cv::Point(0, 0));

        for (int i = 0; i < (int)complexContour.size(); ++i)
            complexContour[i] /= IMG_SCALE;
    }

    return complexContour;
}

std::vector<cv::Point> Object3D::getConvexHull()
{
    if (points == nullptr) return std::vector<cv::Point>();

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
        xyzMap.copyTo(fullXyzMap(getBoundingBox()));
    }

    return fullXyzMap;
}

void Object3D::morph(int erode_sz, int dilate_sz, bool dilate_first, bool gray_map) {
    if (dilate_sz == -1) dilate_sz = erode_sz;

    cv::Mat eKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erode_sz, erode_sz)),
            dKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_sz, dilate_sz));

    if (gray_map) {
        if (dilate_first) cv::dilate(grayMap, grayMap, dKernel);
        cv::erode(grayMap, grayMap, eKernel);
        if (!dilate_first) cv::dilate(grayMap, grayMap, dKernel);
    }
    else{
        if (dilate_first) cv::dilate(xyzMap, xyzMap, dKernel);
        cv::erode(xyzMap, xyzMap, eKernel);
        if (!dilate_first) cv::dilate(xyzMap, xyzMap, dKernel);
    }
}

void Object3D::computeGrayMap(int thresh) {
    if (grayMap.cols != 0 || points == nullptr) return;

    grayMap =  cv::Mat::zeros(xyzMap.size(), CV_8U);

    int points_to_use = num_points;
    if (points_to_use < 0) points_to_use = (int)points->size();

    for (int i = 0; i < points_to_use; ++i) {
        uchar & gv = grayMap.at<uchar>((*points)[i] - topLeftPt);
        gv = (uchar)(xyzMap.at<cv::Vec3f>((*points)[i] - topLeftPt)[2] * 256.0);
        if (gv < thresh) 
            gv = 0;
    }

    morph(2, 3, false, true);

    for (int i = 1; i < IMG_SCALE; i <<= 1) 
        cv::pyrUp(grayMap, grayMap);
}

Object3D::~Object3D()
{
    if (points != nullptr && num_points == -1) {
        delete points;
    }
}
