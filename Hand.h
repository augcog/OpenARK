#pragma once
#include "stdafx.h"

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
    Hand(cv::Vec3f centroid_xyz, cv::Point2i centroid_ij,
        std::vector<cv::Vec3f> fingers_xyz, std::vector<cv::Point2i> fingers_ij, 
        std::vector<cv::Vec3f> defects_xyz, std::vector<cv::Point2i> defects_ij, 
        double svm_confidence);

    /**
    * Destructs the hand instance
    */
    ~Hand();

    // Public variables

    /**
    * (x,y,z) position of all detected finger tips
    */
    std::vector<cv::Vec3f> fingers_xyz;

    /**
    * (i,j) (screen) coordinates of all detected finger tips
    */
    std::vector<cv::Point2i> fingers_ij;

    /**
    * (x,y,z) position of hand centroid
    */
    cv::Vec3f centroid_xyz;

    /**
    * (i,j) (screen) coordinates of hand centroid
    */
    cv::Point2i centroid_ij;

    /**
    * (x,y,z) position of detected defects
    */
    std::vector<cv::Vec3f> defects_xyz;

    /**
    * (i,j) (screen) position of detected defects
    */
    std::vector<cv::Point2i> defects_ij;

    // Public functions

    /**
    * Determine whether one of the fingers is in contact with a tracked object.
    * @param equation Regression equation of the object
    * @param threshold Thickness of the plane modeled by the regression equation
    * @return TRUE if hand is contacting a tracked object. FALSE otherwise
    */
    bool touchObject(std::vector<double> &equation, const double threshold);

    /**
    * The confidence value (in [0, 1]) assigned to this hand by the SVM classifier, 
    * higher = more likely to be hand
    */
    double svm_confidence;

    /**
    * Get the number of fingers on this hand
    */
    int numFingers();

};
