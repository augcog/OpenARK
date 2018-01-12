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
         */
        int xyzAverageSize = 5;

        /**
         * pixels from the bottom edge of the depth map where a point is
         * considered to be connected to the edge
         */
        int bottomEdgeThresh = 30;

        /**
         * pixels from the left/right edges of the depth map where a point is
         * considered to be connected to the edge
         */
        int sideEdgeThresh = 30;

        // ** Clustering parameters ** 

        /**
         * maximum distance in meters between points in the same cluster
         * (used in flood fill)
         */
        double clusterMaxDistance = 0.005;

        /**
         * minimum number of points in a cluster for it to be considered as a 3D object
         */
        int clusterMinPoints = 10000;

        /**
         * number of pixels between consecutive seed points when initiating the flood fill
         */
        int clusterInterval = 10;


        // ** Hand detection parameters ** 

        /**
         * minimum surface area (square meters) of hand
         */
        double handMinArea = 0.01;

        /**
         * maximum surface area (square meters) of hand
         */
        double handMaxArea = 0.056;

        /**
         * if true, hand objects must touch the bottom/bottom-left/bottom-right
         * edge of the visible region
         */
        bool handRequireEdgeConnected = true;

        /**
         * max y-coordinate on the left and right sides (as fraction of image height)
         * to consider a cluster to be connected to the edge
         */
        double handEdgeConnectMaxY = 1.0 / 2.0;

        /**
         * if set to false, disables the SVM used to eliminate objects unlikely to be hands.
         */
        bool handUseSVM = true;

        /**
         * @see handSVMHighConfidenceThresh
         * minimum SVM confidence value ([0...1]) for first hand object
         */
        double handSVMConfidenceThresh = 0.45;

        /**
         * @see handSVMConfidenceThresh
         * minimum SVM confidence value ([0...1]) for additional hand objects
         * (only applied in queryHands)
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
         */
        int contactBotEdgeThresh = 8;

        /**
         * pixels from the side edges of the depth map at which contour points are considered to be on the edge
         * usd while detecting contact points
         */
        int contactSideEdgeThresh = 25;

        /**
         * minimum width of region contacting edge (m)
         */
        float contactWidthMin = 0.037;

        /**
         * maximum width of region contacting edge (m)
         */
        float contactWidthMax = 0.150;

        /**
         * minimum wrist width (m)
         */
        float wristWidthMin = 0.030;

        /**
         * maximum wrist width (m)
         */
        float wristWidthMax = 0.085;

        /**
         * minimum wrist angle (rad)
         */
        double wristAngleMin = 1.00;

        /**
         * maximum wrist angle (rad)
         */
        double wristAngleMax = PI; // disable

        /**
         * maximum distance from the wrist to the center of the hand
         */
        double wristCenterDistThresh = 0.075;

        /**
         * minimum finger length
         */
        double fingerLenMin = 0.014;

        /**
         * maximum finger length
         */
        double fingerLenMax = 0.12;

        /**
         * minimum distance between two finger tips
         */
        double fingerDistMin = 0.015;

        /**
         * minimum value of (finger_y - defect_y)/abs(finger_x - defect_x) for any finger.
         * used to filter out low fingers.
         */
        double fingerDefectSlopeMin = -1.0;

        /**
         * minimum value of (finger_y - center_y)/abs(finger_x - center_x) for any finger.
         * used to filter out low fingers.
         */
        double fingerCenterSlopeMin = -0.45;

        /**
         * minimum curvature of the cluster's contour next to a finger tip
         */
        double fingerCurveNearMin = 1.30;

        /**
         * maximum curvature of the cluster's contour next to a finger tip
         */
        double fingerCurveNearMax = 2.80;

        /**
         * minimum curvature of the cluster's contour at a significant distance from the finger tip
         */
        double fingerCurveFarMin = 0.05;

        /**
         * maximum curvature of the cluster's contour at a significant distance from the finger tip
         */
        double fingerCurveFarMax = 1.10;

        /*
         * minimum finger length used when only one finger is detected
         */
        double singleFingerLenMin = 0.03;

        /**
         * maximum finger length used when only one finger is detected
         */
        double singleFingerLenMax = 0.11;

        /**
         * minimum angle formed by finger tip and neighboring defects
         */
        double singleFingerAngleThresh = 0.08;

        /**
         * maximum angle formed by the start, far, and end points of a defect
         */
        double defectMaxAngle = 0.70 * PI;

        /**
         * minimum distance from the end point of the previous defect to consider the
         * start point of the current defect as a finger candidate
         */
        double defectMinDist = 0.02;

        /**
         * minimum distance from a defect's far point to the center (m)
         */
        double defectFarCenterMinDist = 0.01;

        /**
         * minimum distance from a defect's far point to the center (m)
         */
        double defectFarCenterMaxDist = 0.105;

        /**
         * minimum distance between the start and end points of a defect (m)
         */
        double defectStartEndMinDist = 0.01;

        /**
         * maximum y-coord of a defect below the y-coord of the center point
         */
        int defectMaxYFromCenter = 30;

        /**
         * minimum angle between centroid, defect, and finger
         */
        double centroidDefectFingerAngleMin = 0.40 * PI;
    };
}
