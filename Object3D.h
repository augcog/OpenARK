#pragma once
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
	Hand getHand();
	Plane getPlane();
	cv::Mat getShape();


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
	* Returns all points within a radius of a centroid
	* @param cluster the input point cloud
	* @param distance the radius (meters)
	* @return the percentage of points in the cluster that is "distance" away from the centroid
	*/
	double centroidCircleSweep(cv::Mat cluster, double distance) const;


private:
	/**
	* Reference to the hand instance.
	*/
	Hand hand;

	/**
	* Reference to the plane instance.
	*/
	Plane *plane;

	/**
	* Reference to the shape instance.
	*/
	cv::Mat shape;

	/**
	* Determine whether the object is connected to an edge.
	* @param cluster point cloud of the object
	*/
	void checkEdgeConnected(cv::Mat cluster);

	/**
	* Check whether the object is a hand
	* @param cluster the input point cloud
	* @param min_coverage the smallest allowable coverage for the index finger
	* @param max_coverage the maximum allowable coverage for the index finger
	* @param pointer_finger_distance the length of the pointer finger (meters)
	* @return whether the cluster is a hand
	*/
	bool checkForHand(cv::Mat cluster, double min_coverage, double max_coverage, double pointer_finger_distance = 0.08); //original
	//bool checkForHand(cv::Mat cluster, double min_coverage, double max_coverage, double pointer_finger_distance = 0.04);



};