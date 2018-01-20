#pragma once
#include "stdafx.h"
#include "version.h"

namespace ark {
    class Hand
    {
    public:

        // Public constructors

        /**
        * Default constructor for a hand object
        */
        Hand();

        /**
        * Constructs a hand object from centroid, defect, and finger points
        * @param xyzMap Input point cloud containing only the hand (nothing else can be in this point cluod)
        * @param angle_treshhold Sharpest allowable angle formed by finger tip and neighboring defects
        * @param cluster_thresh Maximum allowable distance between two finger tips
        */
        Hand(Vec3f centroid_xyz, Point2i centroid_ij,
            std::vector<Vec3f> fingers_xyz, std::vector<Point2i> fingers_ij,
            std::vector<Vec3f> defects_xyz, std::vector<Point2i> defects_ij,
            double svm_confidence);

        /**
        * Destructs the hand instance
        */
        ~Hand();

        // Public variables

        /**
        * (x,y,z) position of the center of the hand
        */
        Vec3f center_xyz;

        /**
        * (i,j) coordinates of the center of the hand
        */
        Point2i center_ij;

        /**
        * (x,y,z) position of all detected finger tips
        */
        std::vector<Vec3f> fingers_xyz;

        /**
        * (i,j) coordinates of all detected finger tips
        */
        std::vector<Point2i> fingers_ij;

        /**
        * (x,y,z) position of detected defects (bases of fingers)
        */
        std::vector<Vec3f> defects_xyz;

        /**
        * (i,j) position of detected defects (bases of fingers)
        */
        std::vector<Point2i> defects_ij;

        /**
         * (x,y,z) coordinates of the sides of the wrist ([0] is left, [1] is right)
         */
        Vec3f wrist_xyz[2];

        /**
         * (i,j) coordinates of the sides of the wrist ([0] is left, [1] is right)
         */
        Point2i wrist_ij[2];

        /**
         * radius of largest inscribed circle
         */
        double circle_radius;

        /**
        * The confidence value (in [0, 1]) assigned to this hand by the SVM classifier,
        * higher = more likely to be hand
        */
        double svm_confidence;

        // Public methods

        /**
        * Get the number of fingers on this hand
        */
        int numFingers() const;

        /**
        * Determine whether one of the fingers is in contact with a tracked object.
        * @param equation Regression equation of the object
        * @param threshold Thickness of the plane modeled by the regression equation
        * @return TRUE if hand is contacting a tracked object. FALSE otherwise
        */
        bool touchObject(std::vector<double> &equation, const double threshold) const ;

    };
}
