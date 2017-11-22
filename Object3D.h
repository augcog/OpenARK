#pragma once

#include "stdafx.h" 

// OpenARK Libraries
#include "version.h"
#include "Hand.h"
#include "Plane.h"

#ifdef USE_SVM
#include "HandClassifier.h"
#endif

class Object3D
{
public:
    /**
    * Whether the object is attached to the right edge of the frame.
    * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
    */
    bool rightEdgeConnected = false;

    /**
    * Whether the object is attached to the left edge of the frame.
    * Edge connected implies that object is likely connected to the user's body (hand, arm, etc)
    */
    bool leftEdgeConnected = false;

    /**
    * Constructs a empty instance of a Object3D.
    */
    Object3D();

    /**
    * Constructs an instance of Object3D based on an isolated point cloud containing points in the object.
    * Points not on the cluster must have 0 z-coordinate in the point cloud.
    * Complexity: O(n) in relation to the total number of points in the point cloud.
    * @param clusterDepthMap point cloud containing the object
    * @param min_size optionally, the minimum surface area of an object on which hand detection should be performed
    * @param max_size optionally, the maximum surface area of an object on which hand detection should be performed
    */
    explicit Object3D::Object3D(cv::Mat clusterDepthMap, double min_size = 0.04, double max_size = 0.29);

    /**
    * Construct an instance of Object3D from a vector of points belonging to the object
    * along with a reference point cloud containing all points in the scene.
    * Complexity: O(nlogn) in the size of the 'points' vector. O(n) if already sorted (specify sorted=false)
    * @param [in] points vector of all points (in screen coordinates) belonging to the object
    * @param [in] depthMap the reference point cloud. (CAN contain points outside this object)
    * @param depthMap the reference point cloud. (CAN contain points outside this object)
    * @param sorted if true, assumes that 'points' is already ordered and skips sorting to save time.
    * @param min_size optionally, the minimum surface area of an object on which hand detection should be performed
    * @param max_size optionally, the maximum surface area of an object on which hand detection should be performed
    */   
    Object3D::Object3D(std::vector<cv::Point> & points, cv::Mat & depthMap,
                       bool sorted = false, double min_size = 0.008, double max_size = 0.055);

    /**
    * Deconstructs a Object3D instance.
    */
    ~Object3D();

    /**
    * Whether the object contains a hand.
    */
    bool hasHand = false;

    /**
    * Whether the object contains a plane.
    */
    bool hasPlane = false;

    /**
    * Whether the object contains a shape.
    * A shape is defined by anything that is not a plane or a hand
    */
    bool hasShape = false;

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
    * Gets approximate center of mass of object in screen coordinates
    * @return center of object in screen coordinates
    */
    cv::Point getCenterIj();

    /**
    * Gets approximate center of mass of object in real coordinates
    * @return center of object in real (xyz) coordinates
    */
    cv::Vec3f getCenter();

    /**
    * Gets bounding box of object in screen coordinates
    * @return bounding box of object in screen coordinates
    */
    cv::Rect getBoundingBox() const;

    /**
    * Gets the average depth of the object
    * @returns average z-coordinate of the object in meters
    */
    float getDepth();
    
    /**
    * Gets the visible surface area of the object
    * @returns surface area in meters squared
    */
    double getSurfArea();

    /**
     * Get the depth map of the visible portion of this object
     * @returns depth map of visible object
     */
    const cv::Mat & getDepthMap();

    /**
     * Find the largest 2D contour within this cluster
     * @returns vector of points representing the largest 2D contour within this cluster
     */
    std::vector<cv::Point> getContour(); 

    /**
    * Gets the 2D convex hull of this object
    * @return the 2D convex hull of this object, stored as a vector of points
    */
    std::vector<cv::Point> getConvexHull();

private:
    /**
     * Stores points in this cluster
     */
    std::vector<cv::Point> points;

    /**
     * Top left point of bounding box of cluster
     */
    cv::Point topLeftPt;

    /**
     * (Partial) XYZ map of cluster (CV_32FC3).
     */
    cv::Mat xyzMap;

    /**
     * Full XYZ map of cluster (CV_32FC3).
     */
    cv::Mat fullXyzMap = cv::Mat();

    /**
     * Grayscale image containing normalized depth (z) information from the regular xyzMap (CV_8U)
     * Note: 2x the size of xyzMap
     */
    cv::Mat grayMap;

    /** 
     * Stores size of full xyz map
     */
    cv::Size fullMapSize;

    /** 
     * Surface area in meters squared
     */
    double surfaceArea = -1;

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

    /*
     * Center of the object in screen coordinates.
     */
    cv::Point centerIj = cv::Point(INT_MAX, 0);

    /*
     * Center of the object in real coordinates.
     */
    cv::Vec3f centerXyz = cv::Vec3f(FLT_MAX, 0.0f, 0.0f);

    /*
     * Average depth of object
     */
    double avgDepth = -1.0;

    /*
     * Maximum depth value of object
     */
    double maxVal = -FLT_MAX;

    /*
     * Screen Position of maximum depth value of object
     */
    cv::Point maxLoc = cv::Point(-1, -1);
    
    /**
    * Determine whether the object is connected to an edge.
    * @param cluster point cloud of the object
    */
    void checkEdgeConnected(cv::Mat & cluster);

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
    Hand * checkForHand(const cv::Mat & cluster, double angle_thresh = 0.08, double cluster_thresh = 10,
        double finger_len_min = 0.005, double finger_len_max = 0.17, 
        double max_defect_angle = 0.60 * PI,
        double finger_defect_slope_min = -1.0, double finger_centroid_slope_min = -0.45,
        double finger_dist_min = 0.005, double centroid_defect_finger_angle_min = 0.800);

    /**
    * Subroutine for computing the largest contour, convex hull, etc. for this 3D object.
    * @param min_size optionally, the minimum surface area of an object on which hand detection should be performed
    * @param max_size optionally, the maximum surface area of an object on which hand detection should be performed
    */
    void initializeObject(double min_size, double max_size);

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

   /**
     * Perform erode-dilate morphological operations on the cluster's depth image
     * @param erodeAmt size of erode kernel
     * @param dilateAmt size of dilate kernel (by default, takes same value as erodeAmt)
     * @param dilateFirst if true, performs dilate before erode
     * @param grayMap if true, performs operations on the gray map instead of the xyz map
     */
    void morph(int erodeAmt, int dilateAmt = -1, bool dilateFirst = false, bool grayMap = false);

    /**
     * Compute the grayscale z-coordinate image of this cluster from the normal xyz map
     * @param thresh the minimum z-coordinate of a point on the depth image below which the pixel will be zeroed out
     */
    void computeGrayMap(int thresh = 0);

#ifdef USE_SVM
    /**
    * SVM Hand classifier Instance
    */
    static classifier::SVMHandClassifier handClassifier;
#endif
};
