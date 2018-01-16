#pragma once

#include "stdafx.h"

namespace ark {
    /**
     * Parameters for 3D object, plane, and hand detection
     */
    class ObjectParams {
    public:
        // Construct an instance of ObjectParams using default parameter values
        ObjectParams() {}

        // ** General parameters **

        /**
         * number of pixels around a point on a depth image to average when
         * converting ij (image) coordinates to xyz (world) coordinates
         * default: 5
         */
        int xyzAverageSize = 5;

        /**
         * pixels from the bottom edge of the depth map where a point is
         * considered to be connected to the edge
         * default: 30
         */
        int bottomEdgeThresh = 30;

        /**
         * pixels from the left/right edges of the depth map where a point is
         * considered to be connected to the edge
         * default: 30
         */
        int sideEdgeThresh = 30;

        // ** Clustering parameters ** 

        /**
         * maximum distance in meters between points in the same cluster
         * (used in flood fill)
         * default: 0.005
         */
        double clusterMaxDistance = 0.005;

        /**
         * minimum number of points in a cluster for it to be considered as a 3D object
         * set to 0 to ignore, or -1 to set to NUM_POINTS / 30
         * default: -1
         */
        int clusterMinPoints = -1;

        /**
         * number of pixels between consecutive seed points when initiating the flood fill
         * default: 10
         */
        int clusterInterval = 10;


        // ** Hand detection parameters ** 

        /**
         * minimum surface area (square meters) of hand
         * default: 0.01
         */
        double handMinArea = 0.01;

        /**
         * maximum surface area (square meters) of hand
         * default: 0.056
         */
        double handMaxArea = 0.056;

        /**
         * if true, hand objects must touch the bottom/bottom-left/bottom-right
         * edge of the visible region
         * default: true
         */
        bool handRequireEdgeConnected = true;

        /**
         * max y-coordinate on the left and right sides (as fraction of image height)
         * to consider a cluster to be connected to the edge
         * default: 0.50
         */
        double handEdgeConnectMaxY = 0.50;

        /**
         * if set to false, disables the SVM used to eliminate objects unlikely to be hands.
         * default: true
         */
        bool handUseSVM = true;

        /**
         * @see handSVMHighConfidenceThresh
         * minimum SVM confidence value ([0...1]) for first hand object
         * default: 0.45
         */
        double handSVMConfidenceThresh = 0.45;

        /**
         * @see handSVMConfidenceThresh
         * minimum SVM confidence value ([0...1]) for additional hand objects
         * (only applied in queryHands)
         * default: 0.61
         */
        double handSVMHighConfidenceThresh = 0.61;

        /**
         * maximum distance between the center of the hand and the top point in the hand cluster (m)
         * used when detecting the hand's center
         */
        float centerMaxDistFromTop = 0.150;

        /**
         * pixels from the bottom edge of the depth map at which contour points are considered to be on the edge
         * usd while detecting contact points
         * default: 8
         */
        int contactBotEdgeThresh = 8;

        /**
         * pixels from the side edges of the depth map at which contour points are considered to be on the edge
         * usd while detecting contact points
         * default: 25
         */
        int contactSideEdgeThresh = 25;

        /**
         * minimum width of region contacting edge (m)
         * default: 0.037
         */
        float contactWidthMin = 0.037;

        /**
         * maximum width of region contacting edge (m)
         * default: 0.150
         */
        float contactWidthMax = 0.150;

        /**
         * minimum wrist width (m)
         * default: 0.030
         */
        float wristWidthMin = 0.030;

        /**
         * maximum wrist width (m)
         * default: 0.085
         */
        float wristWidthMax = 0.085;

        /**
         * minimum wrist angle (rad)
         * default: 1.00
         */
        double wristAngleMin = 1.00;

        /**
         * maximum wrist angle (rad)
         * default: PI (disabled)
         */
        double wristAngleMax = PI; // disable

        /**
         * maximum distance from the wrist to the center of the hand
         * default: 0.075
         */
        double wristCenterDistThresh = 0.075;

        /**
         * minimum finger length
         * default: 0.014
         */
        double fingerLenMin = 0.014;

        /**
         * maximum finger length
         * default: 0.12
         */
        double fingerLenMax = 0.12;

        /**
         * minimum distance between two finger tips
         * default: 0.015
         */
        double fingerDistMin = 0.015;

        /**
         * minimum value of (finger_y - defect_y)/abs(finger_x - defect_x) for any finger.
         * used to filter out low fingers.
         * default: -1.0
         */
        double fingerDefectSlopeMin = -1.0;

        /**
         * minimum value of (finger_y - center_y)/abs(finger_x - center_x) for any finger.
         * used to filter out low fingers.
         * default: -0.45
         */
        double fingerCenterSlopeMin = -0.45;

        /**
         * minimum curvature of the cluster's contour next to a finger tip
         * default: 1.30
         */
        double fingerCurveNearMin = 1.30;

        /**
         * maximum curvature of the cluster's contour next to a finger tip
         * default: 2.80
         */
        double fingerCurveNearMax = 2.80;

        /**
         * minimum curvature of the cluster's contour at a significant distance from the finger tip
         * default: 0.05
         */
        double fingerCurveFarMin = 0.05;

        /**
         * maximum curvature of the cluster's contour at a significant distance from the finger tip
         * default: 1.10
         */
        double fingerCurveFarMax = 1.10;

        /*
         * minimum finger length used when only one finger is detected
         * default: 0.04
         */
        double singleFingerLenMin = 0.04;

        /**
         * maximum finger length used when only one finger is detected
         * default: 0.11
         */
        double singleFingerLenMax = 0.11;

        /**
         * minimum angle formed by finger tip and neighboring defects
         * default: 0.08
         */
        double singleFingerAngleThresh = 0.08;

        /**
         * maximum angle formed by the start, far, and end points of a defect
         * default: 0.70 * PI
         */
        double defectMaxAngle = 0.70 * PI;

        /**
         * minimum distance from the end point of the previous defect to consider the
         * start point of the current defect as a finger candidate
         * default: 0.02
         */
        double defectMinDist = 0.02;

        /**
         * minimum distance from a defect's far point to the center (m)
         * default: 0.01
         */
        double defectFarCenterMinDist = 0.01;

        /**
         * minimum distance from a defect's far point to the center (m)
         * default: 0.105
         */
        double defectFarCenterMaxDist = 0.105;

        /**
         * minimum distance between the start and end points of a defect (m)
         * default: 0.01
         */
        double defectStartEndMinDist = 0.01;

        /**
         * maximum y-coord of a defect below the y-coord of the center point
         * default: 30
         */
        int defectMaxYFromCenter = 30;

        /**
         * minimum angle between centroid, defect, and finger
         * default: 0.40 * PI
         */
        double centroidDefectFingerAngleMin = 0.40 * PI;
    };
}
