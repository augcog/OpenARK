#include "stdafx.h"
#include "FramePlane.h"
#include "Util.h"

namespace ark {
    FramePlane::FramePlane() : equation(0, 0, 0), FrameObject()
    {

    }

    FramePlane::FramePlane(const Vec3f & v, const cv::Mat & cluster_depth_map,
        const ObjectParams * params) 
        : equation(v), FrameObject(cluster_depth_map, params) 
    { 
        surfaceArea = util::surfaceArea(fullMapSize, *points, *points_xyz);
    }

    FramePlane::FramePlane(Vec3f v, boost::shared_ptr<std::vector<Point2i>> points_ij,
        boost::shared_ptr<std::vector<Vec3f>> points_xyz, const cv::Mat & depth_map, 
        const ObjectParams * params, bool sorted, int points_to_use) 
        :  equation(v), 
           FrameObject(points_ij, points_xyz, depth_map, params, sorted, points_to_use)
    {
        surfaceArea = util::surfaceArea(fullMapSize, *points, *points_xyz);
    }

    Vec3f FramePlane::getNormalVector()
    {
        cv::Vec3f normal(equation[0], equation[1], -1.0);
        return cv::normalize(normal);
    }

    float FramePlane::getZ(float x, float y)
    {
        return equation[0] * x + equation[1] * y + equation[2];
    }

    bool FramePlane::touching(const Vec3f & point, const Point2i & index, float norm_thresh, bool check_bounds) const
    {
        if (normToPoint(point) > norm_thresh) return false;
        else if (!check_bounds) return true;
        return convexHull.size() && (cv::pointPolygonTest(convexHull, index + topLeftPt, false) > 0);
    }

    float FramePlane::normToPoint(const Vec3f & point) const
    {
        return util::pointPlaneNorm(point, equation);
    }

    float FramePlane::distanceToPoint(const Vec3f & point) const
    {
        return util::pointPlaneDistance(point, equation);
    }

    void FramePlane::cutFromXYZMap(cv::Mat & xyz_map, float threshold,
        cv::Mat * mask, uchar mask_color)
    {
        util::removePlane(xyz_map, equation, threshold, mask, mask_color);
    }
}
