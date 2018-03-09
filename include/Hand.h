#pragma once

#include "FrameObject.h"
#include "FramePlane.h"
#include "Version.h"

namespace ark {
    /**
    * Class representing a hand visible in the current frame.
    * Example on tracking hand and background plane object simulateously:
    * @include HandandPlane.cpp
    */
    class Hand : public FrameObject
    {
    public:

        // Public constructors

        /**
        * Default constructor (warning: creates invalid hand)
        */
        Hand();

        /**
        * Constructs a Hand instance based on an isolated point cloud.
        * Note: points not on the cluster must have 0 z-coordinate in the point cloud.
        * @param cluster_depth_map point cloud containing the object
        * @param params parameters for object/hand detection (if not specified, uses default params)
        */
        explicit Hand(const cv::Mat & cluster_depth_map, DetectionParams::Ptr params = nullptr);

        /**
        * Construct a Hand instance from a vector of points.
        * @param [in] points vector of all points (in screen coordinates) belonging to the object
        * @param [in] depth_map the reference point cloud. (CAN contain points outside this object)
        * @param params parameters for object/hand detection (if not specified, uses default params)
        * @param sorted if true, assumes that 'points' is already ordered and skips sorting to save time.
        * @param points_to_use optionally, the number of points in 'points' to use for the object. By default, uses all points.
        */
        Hand(std::shared_ptr<std::vector<Point2i>> points_ij,
            std::shared_ptr<std::vector<Vec3f>> points_xyz,
            const cv::Mat & depth_map,
            DetectionParams::Ptr params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );

        /**
        * Destructs the hand instance
        */
        ~Hand();

        // Public variables

        // Public methods

        /**
        * Get the number of fingers on this hand
        */
        int getNumFingers() const;

        /**
        * Check whether the object is a hand
        * @param params parameters for hand detection (defaults to this->params)
        */
        bool checkForHand();

        /**
        * Get the (x,y,z) position of the center of the hand's palm
        */
        const Vec3f & getPalmCenter() const;

        /**
        * Get the (i,j) position of the center of the hand's palm
        */
        const Point2i & getPalmCenterIJ() const;

        /**
        * Get the (x,y,z) positions of all detected finger tips
        */
        const std::vector<Vec3f> & getFingers() const;

        /**
        * Get the (i,j) positions of all detected finger tips
        */
        const std::vector<Point2i> & getFingersIJ() const;

        /**
        * Get the (x,y,z) positions of all detected defects (bases of fingers)
        */
        const std::vector<Vec3f> & getDefects() const;;

        /**
        * Get the (i,j) positions of all detected defects (bases of fingers)
        */
        const std::vector<Point2i> & getDefectsIJ() const;;

        /**
        * Get the (x,y,z) coordinates of the sides of the wrist ([0] is left, [1] is right)
        */
        const std::vector<Vec3f> & getWrist() const;

        /**
        * Get the (i,j) coordinates of the sides of the wrist ([0] is left, [1] is right)
        */
        const std::vector<Point2i> & getWristIJ() const;

        /**
        * Get the radius of the largest (2D) inscribed circle within the hand, centered at getPalmCenter()
        */
        double getCircleRadius() const;

        /**
        * Get a unit vector pointing towards the "dominant" direction of the hand
        * i.e. from the center towards the fingers
        */
        Point2f getDominantDirection() const;

        /**
        * Get the SVM confidence rating of this object, [0.0, 1.0] (higher = more likely to be hand)
        */
        float getSVMConfidence() const;

        /**
        * True if hand cluster touches the bottom edge
        * Touching edge implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool touchingEdge() const;

        /**
        * True if hand cluster touches the bottom half of the left edge of the screen or the 
        * left half of the bottom edge of the screen.
        * Touching edge implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool touchingLeftEdge() const;

        /**
        * True if hand cluster touches the bottom half of the right edge of the screen or the 
        * right half of the bottom edge of the screen.
        * Touching edge implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool touchingRightEdge() const;

        /**
        * Retrieve a list of fingers in contact with the specified plane.
        * @param plane [in] plane object
        * @param output [out] output vector containing the indices of the fingers that are touching the plane.
        * @param threshold thickness of the plane, i.e. the maximum distance to the plane at which a point is considered to be 'touching' it
        * @param extrapolate if true, extrapolates the plane infinitely. 
        *        Else, only use the detected portion of the plane.
        * @return the number of fingers in contact with the plane
        */
        int touchingPlane(const FramePlane & plane, std::vector<int> & output, 
            double threshold = 0.0002,
            bool extrapolate = true) const;

        /**
        * Retrieve a list of fingers in contact with the planes.
        * @param planes [in] vector of planes
        * @param output [out] output vector containing
               (.first) the index of the finger and 
               (.second) the indices of the planes in contact with the finger
               ordered by finger
        * @param threshold thickness of the plane, i.e. the maximum distance to the plane at which a point is considered to be 'touching' it
        * @param extrapolate if true, extrapolates the planes infinitely. 
        *        else, only use the detected portion of the plane.
        * @return the number of fingers in contact with any plane at all
        */
        int touchingPlanes(const std::vector<std::shared_ptr<FramePlane> > & planes,
            std::vector<std::pair<int, std::vector<int> > > & output, 
            double threshold = 0.0002,
            bool extrapolate = true) const;

        /**
        * True if this object is a valid hand (queryFrameObjects/queryFrameHands will only return valid hands).
        */
        bool isValidHand() const;

        /** Shared pointer to a Hand */
        typedef std::shared_ptr<Hand> Ptr;

    protected:
        /**
         * Amount the gray map is scaled by for contour finding.
         * Must be a power of 2, at least 1.
         * The higher the scale, the more time-consuming the contour detection 
         * process but the smoother the contour.
         * Set to 2 for hand (up from 1 in other objects)
         */
        int getContourScalingFactor() const override;

    private:
        /**
         * Determine whether the object is connected to an edge.
         */
        void checkEdgeConnected();

        /**
        * (x,y,z) position of the center of the hand
        */
        Vec3f palmCenterXYZ;

        /**
        * (i,j) coordinates of the center of the hand
        */
        Point2i palmCenterIJ;

        /**
        * (x,y,z) position of all detected finger tips
        */
        std::vector<Vec3f> fingersXYZ;

        /**
        * (i,j) coordinates of all detected finger tips
        */
        std::vector<Point2i> fingersIJ;

        /**
        * (x,y,z) position of detected defects (bases of fingers)
        */
        std::vector<Vec3f> defectsXYZ;

        /**
        * (i,j) position of detected defects (bases of fingers)
        */
        std::vector<Point2i> defectsIJ;

        /**
         * (x,y,z) coordinates of the sides of the wrist ([0] is left, [1] is right)
         */
        std::vector<Vec3f> wristXYZ;

        /**
         * (i,j) coordinates of the sides of the wrist ([0] is left, [1] is right)
         */
        std::vector<Point2i> wristIJ;

        /**
         * radius of largest inscribed circle
         */
        double circleRadius;

        /**
         * stores the dominant direction of hand
         */
        Point2f dominantDir;

        /**
        * The confidence value (in [0, 1]) assigned to this hand by the SVM classifier,
        * higher = more likely to be hand
        */
        float svmConfidence;

        /**
        * Whether the hand object is actually valid
        */
        bool isHand = false;

        /**
        * Whether the object is attached to the left edge of the frame.
        * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool leftEdgeConnected = false;

        /**
        * Whether the object is attached to the right edge of the frame.
        * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
        */
        bool rightEdgeConnected = false;
    };
}
