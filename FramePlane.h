#pragma once

#include "stdafx.h"
#include "version.h"
#include "FrameObject.h"

namespace ark {
    /**
    * Class representing a plane object visible in the current frame.
    * Defined by the coefficients of its (reduced) equation: ax + by - z + d = 0
    * Example on tracking hand and background plane object simulateously:
    * @include HandandPlane.cpp
    */
    class FramePlane : public FrameObject
    {
    public:
        /** Construct a default plane (passing through (0, 0, 0) with normal (0, 0, -1)) */
        FramePlane();

        /** 
         * Construct a plane with the given parameter vector and the given 3D object information
         * v[0]x + v[1]y - z + v[2] = 0
         * aka. z = v[0]x + v[1]y + v[2]
         * @param v vector of plane parameters
         * @see FrameObject
         */
        FramePlane(const Vec3f & v, const cv::Mat & cluster_depth_map, const ObjectParams * params = nullptr);

        /** 
         * Construct a plane with the given parameter vector and the given 3D object information
         * v[0]x + v[1]y - z + v[2] = 0
         * aka. z = v[0]x + v[1]y + v[2]
         * @param v vector of plane parameters
         * @param [in] points vector of all points (in screen coordinates) belonging to the object
         * @param [in] depth_map the reference point cloud. (CAN contain points outside this object)
         * @param params parameters for object/hand detection (if not specified, uses default params)
         * @param sorted if true, assumes that 'points' is already ordered and skips sorting to save time.
         * @param points_to_use optionally, the number of points in 'points' to use for the object. By default, uses all points.
         */
        FramePlane(Vec3f v, boost::shared_ptr<std::vector<Point2i>> points_ij, 
                boost::shared_ptr<std::vector<Vec3f>> points_xyz, const cv::Mat & depth_map,
                const ObjectParams * params = nullptr, bool sorted = false, int points_to_use = -1);

        /**
         * Contains the coefficients of the plane equation
         * v[0]x + v[1]y - z + v[2] = 0
         * aka. z = v[0]x + v[1]y + v[2]
         */
        const Vec3f equation;

        /** 
         * Returns the normalized (length 1.0) normal vector of this plane, pointing towards the viewer.
         */
        Vec3f getNormalVector();

        /**
         * Find z-coordinate of a point on the plane with the given x and y coordinates
         * @param x x-coordinate of point
         * @param y y-coordinate of point
         * @return z coordinate of point
         */
        float getZ(float x, float y);

        /**
         * Returns true if the plane is touching the given point (with screen & world coordinates)
         * @param point the point's 3D coordinates
         * @param index the point's screen coordinates
         * @param norm_thresh maximum norm to consider the pland and the point to be "touching"
         * @param check_bounds if true, checks that the point is within the bounds of the screen.
         */
        bool touching(const Vec3f & point, 
            const Point2i & index,
            float norm_thresh = 0.000064, bool check_bounds = false) const;

        /**
         * Find the euclidean norm from the plane to a given point
         * @param point the point
         * @return euclidean distance
         */
        float normToPoint(const Vec3f & point) const;

        /**
         * Find the euclidean distance from the plane to a given point
         * @param point the point
         * @return euclidean distance
         */
        float distanceToPoint(const Vec3f & point) const;

        /**
         * Remove all points fitting a plane's equation from the given point cloud
         * @see util::removePlane
         * @param [in, out] xyz_map the input point cloud
         * @param threshold the thickness of the plane, 
         *        i.e. the maximum distance from the plane to a removed point
         * @param mask, mask_color optional mask matrix whose value must equal 'mask_color'
         *                         at the correspoinding index
         *                         for a point to be removed from the point cloud
         */
        void cutFromXYZMap(cv::Mat & xyz_map, float threshold = 0.000075,
                           cv::Mat * mask = nullptr, uchar mask_color = 0);
    };
}