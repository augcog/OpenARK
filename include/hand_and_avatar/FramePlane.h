#pragma once

#include <vector>
#include "Version.h"
#include "util/FrameObject.h"

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
        FramePlane(const Vec3f & v, const cv::Mat & cluster_depth_map, DetectionParams::Ptr params = nullptr);

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
        FramePlane(Vec3f v, VecP2iPtr points_ij, VecV3fPtr points_xyz,
            const cv::Mat & depth_map, DetectionParams::Ptr params = nullptr,
            bool sorted = false, int points_to_use = -1);

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
         * @param check_bounds if true, checks that the point is within the bounds of the plane on the screen.
         */
        bool touching(const Vec3f & point, 
            const Point2i & index,
            float norm_thresh = 0.000064, bool check_bounds = true) const;

        /**
         * Find the euclidean norm from the plane to a given point
         * @param point the point
         * @return euclidean distance
         */
        float squaredDistanceToPoint(const Vec3f & point) const;

        /**
         * Find the euclidean distance from the plane to a given point
         * @param point the point
         * @return euclidean distance
         */
        float distanceToPoint(const Vec3f & point) const;

        /** 
         * Get vertices of rotated rectangle (2D) that bounds the relevant region of the plane
         */
        const std::vector<Point2f> & getPlaneBoundingRect() const;

        /** Shared pointer to a FramePlane */
        typedef std::shared_ptr<FramePlane> Ptr;

    private:

        /** Helper for computing surface area + bounding rectangle */
        void initializePlane();

        /** Vertices of bounding (rotated) rectangle */
        std::vector<Point2f> boundingRect;
    };
}
