#include "stdafx.h"
#include "FramePlane.h"
#include "Util.h"

namespace ark {
    FramePlane::FramePlane() : equation(0, 0, 0), FrameObject()
    {

    }

    FramePlane::FramePlane(const Vec3f & v, const cv::Mat & cluster_depth_map,
        DetectionParams::Ptr params) 
        : equation(v), FrameObject(cluster_depth_map, params) 
    { 
        initializePlane();
    }

    FramePlane::FramePlane(Vec3f v, VecP2iPtr points_ij, VecV3fPtr points_xyz, 
        const cv::Mat & depth_map, DetectionParams::Ptr params, bool sorted, int points_to_use) 
        :  equation(v), 
           FrameObject(points_ij, points_xyz, depth_map, params, sorted, points_to_use)
    {
        initializePlane();
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
        if (squaredDistanceToPoint(point) > norm_thresh) return false;
        else if (!check_bounds) return true;
        return cv::pointPolygonTest(boundingRect, index, false) >= 0;
    }

    float FramePlane::squaredDistanceToPoint(const Vec3f & point) const
    {
        return util::pointPlaneSquaredDistance(point, equation);
    }

    float FramePlane::distanceToPoint(const Vec3f & point) const
    {
        return util::pointPlaneDistance(point, equation);
    }
    const std::vector<Point2f> & FramePlane::getPlaneBoundingRect() const
    {
        return boundingRect;
    }

    void FramePlane::initializePlane()
    {
        surfaceArea = util::surfaceArea(fullMapSize, *points, *points_xyz);
        cv::RotatedRect boundingRect = cv::minAreaRect(*points);
        Point2f pts[4];
        boundingRect.points(pts);
        this->boundingRect = std::vector<Point2f>(pts, pts + 4);
    }
}
