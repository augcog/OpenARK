#pragma once

#include <vector>
#include "Core.h"

#ifdef UnityPlugin_EXPORTS
#define UnityPlugin_API __declspec(dllexport)   
#else  
#define UnityPlugin_API __declspec(dllimport)   
#endif  

extern "C" {
    /*** CAMERA ***/
    /** Connect to and begin capturing from the depth camera */
    UnityPlugin_API void beginCapture();

    /** Stop capturing from the camera. You may call beginCapture() again afterwards. */
    UnityPlugin_API void endCapture();

    /*** DETECTION ***/

    /** Update the detector. */
    UnityPlugin_API void update();

    /** Returns the number of hands found in the current frame. */
    UnityPlugin_API int numHands();

    /** Returns the number of planes found in the current frame. */
    UnityPlugin_API int numPlanes();

    /*** HAND ***/
    /** Get the central coordinates of hand 'hand_id' (meters)
      * @param axis 0:x 1:y 2:z coordinates
      */
    UnityPlugin_API float handPos(int hand_id, int axis);

    /** Get the hand's average depth (meters) */
    UnityPlugin_API float handAvgDepth(int hand_id);

    /** Get the hand's dominant direction vector
      * @param axis 0:x 1:y
      * @return 2D unit vector in the hand's dominant direction
      */
    UnityPlugin_API float handDirection(int hand_id, int axis);

    /** Get the number of fingers on hand 'hand_id' */
    UnityPlugin_API int handNumFingers(int hand_id);

    /*** WRIST ***/
    /** Get the coordinates of the sides of the hand's wrist
     * @param hand_id id of hand
     * @param index 0: left side of wrist  1: right side of rist
     * @param axis 0:x 1:y 2:z coordinates
     */
    UnityPlugin_API float handWristPos(int hand_id, int index, int axis);

    /*** FINGERS ***/
    /** Get the coordinates of 'index' th finger on hand 'hand_id'
      * @param axis 0:x 1:y 2:z coordinates
      */
    UnityPlugin_API float handFingerPos(int hand_id, int index, int axis);

    /** Get the coordinates of 'index'th finger on hand 'hand_id' 
      * @param axis 0:x 1:y 2:z coordinates
      */
    UnityPlugin_API float handDefectPos(int hand_id, int index, int axis);

    /*** PLANE ***/
    /** Get the coordinates at the center of mass of plane 'plane_id'
      * @param axis 0:x 1:y 2:z coordinates
      */
    UnityPlugin_API float planePos(int plane_id, int axis);

    /** Get the equation of plane 'plane_id': ax + by - z + c = 0
      * @param term 0:a 1:b 2:c
      */
    UnityPlugin_API float planeEquation(int plane_id, int term);

    /** Get the euclidean distance from a plane to a point */
    UnityPlugin_API float planePointDist(int plane_id, float x, float y, float z);

    /** Get the L2 norm from a plane to a point */
    UnityPlugin_API float planePointNorm(int plane_id, float x, float y, float z);

    /** Compute contact points between a hand and a plane
      * @see numTouches
      * @see touchIndex
      * @see touchPos
      * @param thresh max L2 norm between two points to consider them to be touching (m^2)
      * @return number of contact points found
      */
    UnityPlugin_API int computeTouches(int hand_id, int plane_id, float thresh);

    /** Get the number of finger-plane contact points found by computeTouches.
      * @see computeTouches
      */
    UnityPlugin_API int numTouches();

    /** Get the finger index of the 'touch_id' th finger-plane contact point found by computeTouches.
      * @see computeTouches
      * @see handFingerPos
      */
    UnityPlugin_API int touchFingerIndex(int touch_id);

    /** Get the 3D coordinates of the 'touch_id' th finger-plane contact point found by computeTouches.
      * @see computeTouches
      * @see handFingerPos
      * @param axis 0:x 1:y 2:z coordinates
      */
    UnityPlugin_API float touchPos(int touch_id, int axis);

    /** set whether the hand detector should use the SVM to eliminate false positives */
    UnityPlugin_API void handUseSVM(bool value);

    /** set whether the hand detector should eliminate hands not connected to the edge of the screen */
    UnityPlugin_API void handRequireEdgeConnected(bool value);
}
