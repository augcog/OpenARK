#pragma once

#include "stdafx.h" 

// OpenARK Libraries
#include "Hand.h"
#include "Plane.h"

class Object3D
{
public:
    /**
    * Whether the object is attached to the right edge of the frame.
    * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
    */
    bool rightEdgeConnected;

    /**
    * Whether the object is attached to the left edge of the frame.
    * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
    */
    bool leftEdgeConnected;

    /**
    * Constructs a empty instance of a Object3D.
    */
    Object3D();

    /**
    * Constructs a instance of Object3D based on a point cloud.
    * @param cluster point cloud representation of the object
    */
    explicit Object3D(cv::Mat cluster);

    /**
    * Deconstructs a Object3D instance.
    */
    ~Object3D();

    /**
    * Whether the object contains a hand.
    */
    bool hasHand;

    /**
    * Whether the object contains a plane.
    */
    bool hasPlane;

    /**
    * Whether the object contains a shape.
    * A shape is defined by anything that is not a plane or a hand
    */
    bool hasShape;

    /**
    * Gets instance of hand object if a hand is found.
    * @return instance of hand object
    */
    Hand getHand() const;

    /**
    * Gets instance of plane object is plane is found.
    * @return instance of plane object
    */
    Plane getPlane() const;

    /**
    * Gets instance of shape object.
    * @return instance of shape object
    */
    cv::Mat getShape() const;

    /**
    * Gets most complex (largest) contour in object
    * @return the largest contour in this 3D object
    */
    std::vector<cv::Point > getComplexContour() const;

    /**
    * Gets the 2D convex hull of this object
    * @return the 2D convex hull of this object, stored as a vector of points
    */
    std::vector<cv::Point > getConvexHull() const;

private:
    /**
    * Pointer to the hand instance.
    */
    Hand *hand = nullptr;

    /**
    * Pointer to the plane instance.
    */
    Plane *plane = nullptr;

    /**
    * The shape instance.
    */
    cv::Mat shape;

    /**
    * Stores channels of the input point cloud
    */
    cv::Mat channel[3];

    /** 
    * Largest contour in object
    */
    std::vector<cv::Point> complexContour;

    /**
    * Convex hull of object
    */
    std::vector<cv::Point> convexHull;

    /* 
    * Convex hull of this object, * with points stored as indices of the contour rather than cv::Point
    */
    std::vector<int> indexHull;

    /**
    * Determine whether the object is connected to an edge.
    * @param cluster point cloud of the object
    */
    void checkEdgeConnected(cv::Mat cluster);

    /**
    * Check whether the object is a hand
    * @param cluster the input point cloud
    * @param angle_thresh minimum angle formed by finger tip and neighboring defects
    * @param cluster_thresh maximum distance between two finger tips
    * @param finger_len_min minimum finger length
    * @param finger_len_max maximum finger length
    * @param max_defect_angle maximum angle (ij) at each defect
    * @param finger_defect_slope_min minimum value of (finger_y - defect_y)/abs(finger_x - defect_x) for any finger.
                        Used to filter out low fingers. Used to filter out fingers pointing straight down.
    * @param finger_centroid_slope_min minimum value of (finger_y - center_y)/abs(finger_x - center_x) for any finger.
    * @param finger_dist_min minimum distance between fingers
    * @param centroid_defect_finger_angle_min minimum angle between centroid, defect, and finger
    */
    Hand * checkForHand(cv::Mat cluster, double angle_thresh = 5, double cluster_thresh = 20,
        double finger_len_min = 0.01, double finger_len_max = 0.01, double max_defect_angle = 0.9 * PI,
        double finger_defect_slope_min = -1.3, double finger_centroid_slope_min = -0.5,
        double finger_dist_min = 0.020, double centroid_defect_finger_angle_min = 1.100);

    /**
    * Subroutine for computing the largest contour, convex hull, etc. for this 3D object.
    * @param cluster the point cloud representing the 3D object
    */
    void computeObjectFeatures(cv::Mat cluster);


    /**
    * Find the contour with the maximum number of points
    * @param contours vector of all contours
    * @returns contour with max number of points
    */
    static inline std::vector<cv::Point> findComplexContour(std::vector< std::vector<cv::Point> > contours);

    /**
    * Find the center of mass of an object
    * @param contour contour of the object
    * @returns point representing center of mass
    */
    static inline cv::Point Object3D::findCenter(std::vector<cv::Point> contour);

    /**
    * Simplify a convex hull by clustering points within distance 'threshold'
    * @param threshold maximum distance between points in a cluster
    * @returns simplified convex hull
    */
    static std::vector<cv::Point> Object3D::clusterConvexHull(std::vector<cv::Point> convexHull, int threshold);
};
