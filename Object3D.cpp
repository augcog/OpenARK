#include "stdafx.h"
#include "version.h"

#include "Object3D.h"
#include "Hand.h"
#include "Visualizer.h"
#include "Util.h"

#ifdef USE_SVM
#include "HandFeatureExtractor.h"
#include "HandClassifier.h"
#endif

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

#ifdef USE_SVM
const char* SVM_MODEL_PATH = "svm/";
classifier::SVMHandClassifier Object3D::handClassifier = classifier::SVMHandClassifier(SVM_MODEL_PATH);
#endif

Object3D::Object3D()
{

}

Object3D::Object3D(cv::Mat cluster) {
    // Initialize variables
    rightEdgeConnected = false;
    leftEdgeConnected = false;
    hasHand = false;
    hasPlane = false;
    hasShape = false;

    checkEdgeConnected(cluster);

    // Compute needed information

    computeObjectFeatures(cluster);

    // Step 1: determine whether cluster is hand

    //if (checkForHand(cluster, 0.005, 0.25))  //original
    if (hand = checkForHand(cluster)) {
        surfaceArea = Util::surfaceArea(cluster);
        centerIj = Util::findCentroid(cluster);
        centerXyz = Util::averageAroundPoint(cluster, centerIj, 15);
        avgDepth = Util::averageDepth(cluster);

        hasHand = true;
#ifdef USE_SVM
        std::vector<double> features = classifier::features::extractHandFeatures(*this, cluster);
        double confidence = handClassifier.classify(features);

        if (confidence < 0.05) {
            hasHand = false;
        }
#endif
        return;

    }
    // TEMPORARILY disabling plane detection
    return;

    // Step 2: determine whether there is a plane
    plane = new Plane(cluster);

    // Step 2.1 If there is plane, remove plane and look for hand
    std::vector<cv::Point> points = plane->getPlaneIndicies();

    if (points.size() != 0)
    {
        hasPlane = true;

        for (int i = 0; i < points.size(); i++)
        {
            int x = points[i].x;
            int y = points[i].y;
            cluster.at<cv::Vec3f>(y, x)[0] = 0;
            cluster.at<cv::Vec3f>(y, x)[1] = 0;
            cluster.at<cv::Vec3f>(y, x)[2] = 0;
        }

        cv::Mat hand_cluster = cv::Mat::zeros(cluster.rows, cluster.cols, cluster.type());

        //determining the pixels that are similar to center and connected to it
        Util::floodFill(centerIj.x, centerIj.y, cluster, hand_cluster, 0.02);

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

    // Step 2.1.1 If there is plane, no hand, then the rest of points are shape
    shape = cluster;
    hasShape = true;
}

inline std::vector<cv::Point> Object3D::findComplexContour(std::vector< std::vector<cv::Point> > contours)
{
    size_t maxPoints = 0; unsigned maxId = 0;

    for (unsigned i = 0; i < contours.size(); ++i)
    {
        if (contours[i].size() > maxPoints)
        {
            maxPoints = contours[i].size();
            maxId = i;
        }
    }

    return contours[maxId];
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

// compute contour, convex hull of this 3D object
void Object3D::computeObjectFeatures(cv::Mat cluster)
{
    cv::Mat input;
    input.create(cluster.size(), CV_8UC1);
    for (uint r = 0; r < cluster.rows; ++r) {
        const cv::Vec3f * ptr = cluster.ptr<cv::Vec3f>(r);
        uchar * inputPtr = input.ptr<uchar>(r);

        for (uint c = 0; c < cluster.cols; ++c) {
            inputPtr[c] = (uchar)(ptr[c][2] * 255);
            if (inputPtr[c] < 3) inputPtr[c] = 0;
        }
    }

    const cv::Mat eKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)),
                  dKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    cv::erode(input, input, eKernel);
    cv::dilate(input, input, dKernel);

    // Resize input
    cv::pyrUp(input, input);

    //cv::threshold(input, input, 100, 255, cv::THRESH_BINARY);

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(input, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Find largest contour
    complexContour = findComplexContour(contours);

    // Simplify contour using DP algorithm
    cv::approxPolyDP(complexContour, complexContour, 0.002, true);

    std::vector<cv::Point> hull;
    if (complexContour.size() > 1)
    {
        // Find convex hull
        cv::convexHull(complexContour, convexHull, false, true);
        cv::convexHull(complexContour, indexHull, false, false);
    }
}

Hand * Object3D::checkForHand(const cv::Mat cluster, double angle_thresh, double cluster_thresh,
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

    // Find max and min distances
    double maxVal = 0;
    cv::Point maxLoc;

    for (uint r = 0; r < cluster.rows; ++r) {
        const cv::Vec3f * ptr = cluster.ptr<cv::Vec3f>(r);

        for (uint c = 0; c < cluster.cols; ++c) {
            if (ptr[c][2] > maxVal) {
                maxVal = ptr[c][2];
                maxLoc = cv::Point(c, r);
            }
        }
    }

    // Find center of contour
    cv::Point center = findCenter(complexContour);

    if (!Util::pointInImage(cluster, center, IMG_SCALE)) {
        return nullptr;
    }

    Hand * hand = new Hand();

    hand->centroid_xyz = cluster.at<cv::Vec3f>(center / IMG_SCALE);
    hand->centroid_ij = cv::Point2i(center.x, center.y); // SCALING

    cv::Point index, index_right, index_left;
    double farthest = 0;

    if (hull.size() > 1)
    {
        for (int i = 0; i < hull.size(); ++i)
        {
            cv::Point p1 = hull[i];
            cv::Vec3f xyzP1 = Util::averageAroundPoint(cluster, p1 / 2, 22);

            double dist = Util::euclideanDistance3D(xyzP1, hand->centroid_xyz);
            double slope = (double)(hand->centroid_ij.y - p1.y) / abs(p1.x - hand->centroid_ij.x);

            if (slope > finger_centroid_slope_min &&
                p1.y < cluster.rows * IMG_SCALE - 10 && // cut off bottom points
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

    //cv::Mat visualL = cv::Mat::zeros(cluster.rows * 4, cluster.cols * 4, cluster.type());
    cv::Vec3f lastPoint;

    for (int i = 0; i < defects.size(); ++i)
    {
        cv::Vec4i defect = defects[i];
        cv::Point start = complexContour[defect[0]],
                  end = complexContour[defect[1]],
                  farPt = complexContour[defect[2]];

        int depth = defect[3]; // Depth from edge of contour

        // Defect conditions: depth is sufficient, inside contour, y value is above center
        // maxLoc largest depth
        // first condition replace with meters distance from the edge
        // second test if inside the hull (removed)
        // not too far below the center (no change)

        if (cv::norm(maxLoc - center) * 4 < depth && /*
            cv::pointPolygonTest(hull, farPt, false) > 0 && */
            farPt.y < center.y + 30 * IMG_SCALE &&
            farPt.y < cluster.rows * IMG_SCALE - 10)
        {

            double angle = Util::angleBetweenPoints(start, end, farPt); // The angle from start through farPt to end
            if (angle > max_defect_angle) continue;

            cv::Vec3f pt1 = cluster.at<cv::Vec3f>(farPt.y / IMG_SCALE, farPt.x / IMG_SCALE);
            cv::Vec3f start_xyz = Util::averageAroundPoint(cluster, cv::Point2i(start.x / IMG_SCALE, start.y / IMG_SCALE), 15),
                   end_xyz = Util::averageAroundPoint(cluster, cv::Point2i(end.x / IMG_SCALE, end.y / IMG_SCALE), 15);

            double dist = Util::euclideanDistance3D(pt1, hand->centroid_xyz);

            if (dist > 0.01)
            {
                if (i == 0 || Util::euclideanDistance3D(lastPoint, start_xyz) > 0.08) {
                    endpoints.push_back(start);
                    fingerDefects.push_back(farPt);

                    //cv::circle(visualL, start, 2, cv::Scalar(0, 255, 0));
                    //cv::line(visualL, start, farPt, cv::Scalar(0, 255, 0));
                }

                endpoints.push_back(end);
                fingerDefects.push_back(farPt);

                lastPoint = end_xyz;

                //cv::circle(visualL, end, 3, cv::Scalar(0, 0, 255));
                //cv::circle(visualL, farPt, 2, cv::Scalar(255, 0, 0), 2);
                //cv::putText(visualL, std::to_string(i), farPt, 0, 2, cv::Scalar(255,255,255),2);
                //cv::line(visualL, end, farPt, cv::Scalar(0, 0, 255));
            }
        }
    }

    //cv::Mat visual; cv::resize(visualL, visual, cv::Size(1280, 720));
    //cv::namedWindow("Defects");
    //cv::imshow("Defects", visual);

    std::vector<cv::Point> fingers_ij, defects_ij;
    std::vector<cv::Vec3f> fingers_xyz, defects_xyz;

    //// Cluster fingertip locations
    //endpoints = clusterConvexHull(endpoints, cluster_thresh);

    for (unsigned i = 0; i < endpoints.size(); i++)
    {
        cv::Point endpoint = endpoints[i];
        cv::Point relatedDefect = fingerDefects[i];

        cv::Vec3f endPoint_xyz = Util::averageAroundPoint(cluster, cv::Point2i(endpoint.x / IMG_SCALE, endpoint.y / IMG_SCALE), 15),
             closestDefect_xyz = Util::averageAroundPoint(cluster, cv::Point2i(relatedDefect.x / IMG_SCALE, relatedDefect.y / IMG_SCALE), 15);

        double finger_length = Util::euclideanDistance3D(endPoint_xyz, closestDefect_xyz) / IMG_SCALE;
        double centroid_defect_dist = Util::euclideanDistance3D(hand->centroid_xyz, closestDefect_xyz);

        double finger_defect_slope = (double)(relatedDefect.y - endpoint.y) / abs(relatedDefect.x - endpoint.x);
        double finger_centroid_slope = (double)(center.y - endpoint.y) / abs(center.x - endpoint.x);
        double centroid_defect_finger_angle = Util::angleBetweenPoints(hand->centroid_ij, endpoint, relatedDefect);

        if (finger_length < finger_len_max && finger_length > finger_len_min &&
            finger_defect_slope > finger_defect_slope_min && finger_centroid_slope > finger_centroid_slope_min &&
            centroid_defect_finger_angle > centroid_defect_finger_angle_min && endPoint_xyz[2] != 0)
        {
            fingers_xyz.push_back(endPoint_xyz);
            fingers_ij.push_back(cv::Point2i(endpoint.x, endpoint.y)); // SCALING
            defects_xyz.push_back(closestDefect_xyz);
            defects_ij.push_back(cv::Point2i(relatedDefect.x, relatedDefect.y)); // SCALING
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
            Util::averageAroundPoint(cluster, cv::Point2i(index.x / IMG_SCALE, index.y / IMG_SCALE), 22);
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


void Object3D::checkEdgeConnected(cv::Mat & cluster)
{
    int cols = cluster.cols, rows = cluster.rows;

    // bottom Sweep
    int r1 = static_cast<int>(rows * 0.9);
    for (int c1 = 0; c1 < cols / 4; c1++)
    {
        if (cluster.at<cv::Vec3f>(r1, c1)[2] != 0)
        {
            leftEdgeConnected = true;
            break;
        }
    }

    // Left Side Sweep
    int c2 = static_cast<int>(cols * 0.2);

    for (int r2 = 0; r2 < rows; r2++)
    {
        if (cluster.at<cv::Vec3f>(r2, c2)[2] != 0)
        {
            leftEdgeConnected = true;
            break;
        }
    }

    // Bottom Sweep
    int r3 = static_cast<int>(rows * 0.9);
    //for (auto c3 = cols / 4; c3 < cols; c3++) //original
    for (int c3 = cols - (cols / 4); c3 < cols; c3++)
    { //fixed
        if (cluster.at<cv::Vec3f>(r3, c3)[2] != 0)
        {
            rightEdgeConnected = true;
            break;
        }
    }

    // Right Side Sweep
    int c4 = static_cast<int>(cols * 0.8);
    for (int r4 = rows / 2; r4 < rows; r4++)
    {
        if (cluster.at<cv::Vec3f>(r4, c4)[2] != 0)
        {
            rightEdgeConnected = true;
            break;
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

cv::Point Object3D::getCenterIj() const
{
    return centerIj;
}

cv::Vec3f Object3D::getCenter() const
{
    return centerXyz;
}

float Object3D::getDepth() const
{
    return avgDepth;
}

cv::Mat Object3D::getShape() const
{
    return shape;
}

double Object3D::getSurfaceArea() const {
    return surfaceArea;
}

std::vector<cv::Point> Object3D::getComplexContour() const
{
    return complexContour;
}

std::vector<cv::Point> Object3D::getConvexHull() const
{
    return convexHull;
}

Object3D::~Object3D()
{
}