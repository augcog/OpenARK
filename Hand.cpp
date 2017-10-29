#include "Hand.h"

#include "Util.h"
#include "Visualizer.h"

Hand::Hand()
{

}

Hand::Hand(cv::Mat xyzMap, float angle_threshhold, int cluster_thresh)
{
    CLUSTER_THRESHOLD = cluster_thresh;
    ANGLE_THRESHHOLD = angle_threshhold;

    analyzeHand(xyzMap);
}

Hand::~Hand()
{

}

void Hand::analyzeHand(cv::Mat xyzMap)
{

    cv::Mat normalizedDepthMap;
    cv::Mat channel[3];
    cv::split(xyzMap, channel);
    cv::normalize(channel[2], normalizedDepthMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // Resize input
    cv::Mat input;
    cv::pyrUp(normalizedDepthMap, input, cv::Size(normalizedDepthMap.cols * 2, normalizedDepthMap.rows * 2));
    cv::pyrUp(input, input, cv::Size(input.cols * 2, input.rows * 2));
    cv::Mat threshold_output;
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find contours
    cv::threshold(input, threshold_output, 100, 255, cv::THRESH_BINARY);
    cv::findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Find contour polygon
    std::vector< std::vector< cv::Point> > contours_poly(contours.size());

    for (auto i = 0; i < contours.size(); i++)
    {
        //using Douglas-Peucker algorithm for approximating the contour curve
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    }

    // Find largest contour
    auto contour = Hand::findComplexContour(contours);

    // Find approximated convex hull

    std::vector<cv::Point> hull;
    std::vector<cv::Point> completeHull;
    std::vector<int> indexHull;
    if (contour.size() > 1)
    {
        cv::convexHull(contour, completeHull, false, true);
        cv::convexHull(contour, indexHull, false, false);
        hull = Hand::clusterConvexHull(completeHull, Hand::CLUSTER_THRESHOLD);
    }

    // Find convexity defects
    std::vector<cv::Vec4i> defects;

    if (indexHull.size() > 3)
    {
        cv::convexityDefects(contour, indexHull, defects);
    }

    // Find max and min distances
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(channel[2], &minVal, &maxVal, &minLoc, &maxLoc);
    // Find center of contour
    auto center = Hand::findCenter(contour);
    centroid_xyz = xyzMap.at<cv::Vec3f>(center.y / 4, center.x / 4);
    centroid_ij = cv::Point2i(center.x, center.y); // SCALING

    // Generate visual
    cv::Mat img = cv::Mat::zeros(input.rows, input.cols, CV_8UC3);
    auto color = cv::Scalar(0, 255, 0);

    // Draw contours
    cv::circle(img, center, 5, cv::Scalar(255, 0, 0), 2);

    for (auto i = 0; i < contours.size(); i++)
    {
        cv::drawContours(img, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    // Draw hull
    cv::Point index;
    cv::Point index_right;
    cv::Point index_left;
    double farthest = 0;

    if (hull.size() > 1)
    {
        for (auto i = 0; i < hull.size(); i++)
        {
            auto p1 = hull[i];
            auto p2 = hull[(i + 1) % hull.size()];
            cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 1);
            if (p1.y < centroid_ij.y && Util::euclideanDistance2D(p1, centroid_ij) > farthest)
            {
                farthest = Util::euclideanDistance2D(p1, centroid_ij);
                index = p1;
                index_right = hull[(i + 1) % hull.size()];
                index_left = hull[(i - 1) % hull.size()];
            }
        }
    }

    // Draw defects (filter)
    std::vector<cv::Point> endpoints;
    std::vector<cv::Point> fingerDefects;
    cv::Point lastStart;
    auto found = -1;
    for (auto i = 0; i < defects.size(); i++)
    {
        auto defect = defects[i];
        auto start = contour[defect[0]];
        auto end = contour[defect[1]];
        auto farPt = contour[defect[2]];
        // Depth from edge of contour
        // Defect conditions: depth is sufficient, inside contour, y value is above center
        auto depth = defect[3];
        // maxLoc largest depth
        // first condition replace with meters distance from the edge
        // second test if inside the hull (no change)
        // above the center (no change)
        //if (  cv::pointPolygonTest(hull, farPt, false) > 0 && farPt.y < center.y)
        //if (cv::norm(maxLoc - center) * 15 < depth && cv::pointPolygonTest(hull, farPt, false) > 0 && farPt.y < center.y) //original
        if (cv::norm(maxLoc - center) * 10 < depth && cv::pointPolygonTest(hull, farPt, false) > 0 && farPt.y < center.y)
        {
            auto pt1 = xyzMap.at<cv::Vec3f>(farPt.y / 4, farPt.x / 4);
            if (Util::euclidianDistance3D(pt1, centroid_xyz) > 0.05)
            {
                endpoints.push_back(start);
                endpoints.push_back(end);
                fingerDefects.push_back(farPt);
            }
        }
    }

    // Cluster fingertip locations
    endpoints = Hand::clusterConvexHull(endpoints, Hand::CLUSTER_THRESHOLD);

    for (auto i = 0; i < endpoints.size(); i++)
    {
        auto endpoint = endpoints[i];
        cv::Point closestDefect;
        auto minDefectDistance = 1 << 29;
        for (auto j = 0; j < fingerDefects.size(); j++)
        {
            if (cv::norm(endpoint - fingerDefects[j]) < minDefectDistance)
            {
                minDefectDistance = cv::norm(endpoint - fingerDefects[j]);
                closestDefect = fingerDefects[j];
            }
        }
        auto endPoint_xyz = Util::averageAroundPoint(xyzMap, cv::Point2i(endpoint.x / 4, endpoint.y / 4), 10); //original
        auto closestDefect_xyz = Util::averageAroundPoint(xyzMap, cv::Point2i(closestDefect.x / 4, closestDefect.y / 4), 10); //original
        auto finger_length = Util::euclidianDistance3D(endPoint_xyz, closestDefect_xyz);
        if (finger_length < 0.08 && finger_length > 0.005 && endpoint.y < closestDefect.y)
        //if (finger_length < 0.08 && finger_length > 0.025 && endpoint.y < closestDefect.y) //original
        {
            fingers_xyz.push_back(endPoint_xyz);
            fingers_ij.push_back(cv::Point2i(endpoint.x, endpoint.y)); // SCALING
            defects_xyz.push_back(Util::averageAroundPoint(xyzMap, cv::Point2i(closestDefect.x / 4, closestDefect.y / 4), 5));
            defects_ij.push_back(cv::Point2i(closestDefect.x, closestDefect.y)); // SCALING
        }
    }

    if (static_cast<float>(cv::countNonZero(channel[2])) / (xyzMap.rows*xyzMap.cols) > 0.3)
    {
        return;
    }

    // If there is one or less visible fingers
    if (fingers_xyz.size() <= 1)
    {
        fingers_xyz.clear();
        fingers_ij.clear();

        auto indexFinger = Util::averageAroundPoint(xyzMap, cv::Point2i(index.x / 4, index.y / 4), 10);
        fingers_xyz.push_back(indexFinger);
        fingers_ij.push_back(cv::Point2i(index.x, index.y)); // SCALING
        auto angle = Util::TriangleAngleCalculation(index_left.x, index_left.y, index.x, index.y, index_right.x, index_right.y);

        if (defects_ij.size() != 0)
        {
            for (auto i = 0; i < fingers_xyz.size(); i++)
            {
                cv::circle(img, fingers_ij[i], 5, cv::Scalar(0, 0, 255), 3);

                cv::line(img, defects_ij[i], fingers_ij[i], cv::Scalar(255, 0, 255), 2);
                cv::circle(img, defects_ij[i], 5, cv::Scalar(0, 255, 255), 2);

                cv::line(img, defects_ij[i], centroid_ij, cv::Scalar(255, 0, 255), 2);
            }
        }

        else if (angle > ANGLE_THRESHHOLD)
        {
            cv::circle(img, fingers_ij[0], 5, cv::Scalar(0, 0, 255), 3);

            cv::line(img, fingers_ij[0], centroid_ij, cv::Scalar(255, 0, 255), 2);
        }

    }

    else {

        for (auto i = 0; i < fingers_xyz.size(); i++)
        {
            cv::circle(img, fingers_ij[i], 5, cv::Scalar(0, 0, 255), 3);

            cv::line(img, defects_ij[i], fingers_ij[i], cv::Scalar(255, 0, 255), 2);
            cv::circle(img, defects_ij[i], 3, cv::Scalar(0, 255, 255), 2);

            cv::line(img, defects_ij[i], centroid_ij, cv::Scalar(255, 0, 255), 2);
        }
    }


    cv::Mat img_dst;
    cv::resize(img, img_dst, cv::Size(640, 480), 0, 0, cv::INTER_AREA);

    cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    cv::imshow("Contours", img_dst);
}

//find the contour with the maximum number of points
std::vector<cv::Point> Hand::findComplexContour(std::vector< std::vector<cv::Point> > contours)
{
    std::vector<cv::Point> contour;
    auto maxPoints = 0;

    for (auto i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() > maxPoints)
        {
            maxPoints = contours[i].size();
            contour = contours[i];
        }
    }

    return contour;
}

std::vector<cv::Point> Hand::clusterConvexHull(std::vector<cv::Point> convexHull, int threshold) const
{
    std::vector<cv::Point> clusterHull;
    auto i = 0;

    while (i < convexHull.size())
    {
        // Select a point from cluster
        std::vector<cv::Point> cluster;
        auto hullPoint = convexHull[i];
        cluster.push_back(hullPoint);
        i++;

        while (i < convexHull.size())
        {
            auto clusterPoint = convexHull[i];
            auto distance = cv::norm(hullPoint - clusterPoint);

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
        auto center = findCenter(convexHull);
        int maxDist = cv::norm(hullPoint - center);

        for (auto j = 0; j < cluster.size(); j++)
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

cv::Point Hand::findCenter(std::vector<cv::Point> contour)
{
    //using image moments to find center of mass of the object
    //Cx=M10/M00 and Cy=M01/M00
    auto M = cv::moments(contour, false);
    auto center = cv::Point(static_cast<int>(M.m10) / M.m00, static_cast<int>(M.m01) / M.m00);
    return center;
}

bool Hand::touchObject(std::vector<double> &equation, const double threshold)
{
    if (equation.size() == 0)
    {
        return false;
    }

    for (auto i = 0; i < fingers_xyz.size(); i++)
    {
        double x = fingers_xyz[i][0];
        double y = fingers_xyz[i][1];
        double z = fingers_xyz[i][2];

        if (z == 0)
        {
            return false;
        }

        auto z_hat = equation[0] * x + equation[1] * y + equation[2];
        auto r_squared = (z - z_hat) * (z - z_hat);

        if (r_squared < threshold)
        {
            return true;
        }
    }

    return false;
}