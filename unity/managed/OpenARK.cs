using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using UnityEngine;

/** OpenARK Unity Interface */
namespace OpenARK
{

    /** Represents a detector */
    public class Detector
    {
        /** create a new, empty detector instance */
        public Detector(bool beginCaptureOnInit = false) {
            if (beginCaptureOnInit)
            {
                beginCapture();
            }
        }

        /** destructor */
        ~Detector()
        {
            Internal.endCapture();
        }

        /** whether the SVM is used for hand detection */
        public bool handUseSVM
        {
            get
            {
                return _useSVM;
            }
            set
            {
                _useSVM = value;
                Internal.handUseSVM(_useSVM);
            }
        }
        private bool _useSVM = true;

        /** whether hands must be connected to the edge of the screen */
        public bool handRequireEdgeConnected
        {
            get
            {
                return _reqEdgeConnected;
            }
            set
            {
                _reqEdgeConnected = value;
                Internal.handRequireEdgeConnected(_reqEdgeConnected);
            }
        }
        private bool _reqEdgeConnected = false;

        /** update this detector with the newest information */
        public void update()
        {
            Internal.update();
        }

        /** begin capturing from the camera and detecting objects */
        public void beginCapture()
        {
            Internal.beginCapture();
        }

        /** stop capturing from the camera and detecting objects */
        public void endCapture()
        {
            Internal.endCapture();
        }

        /** get a list of hands in the current frame */
        public List<Hand> getHands()
        {
            int nHands = Internal.numHands();
            List<Hand> result = new List<Hand>();

            for (int i = 0; i < nHands; ++i)
            {
                Vector3 center = Internal.readVector3(Internal.handPos, i);
                float depth = 0.0f;// Internal.handAvgDepth(i);
                Vector2 direction = new Vector2(1, 1);//= Internal.readVector2(Internal.handDirection, i);

                int nFingers = Internal.handNumFingers(i);
                Vector3[] fingers = Internal.readArray(Internal.handFingerPos, i, nFingers);
                Vector3[] defects = Internal.readArray(Internal.handDefectPos, i, nFingers);
                Vector3[] wrist = Internal.readArray(Internal.handWristPos, i, 2);

                Hand hand = new Hand(i, center, depth, direction, fingers, defects, wrist);
                result.Add(hand);
            }

            return result;
        }

        /** get a list of planes in the current frame */
        public List<Plane> getPlanes()
        {
            int nPlanes = Internal.numPlanes();
            List<Plane> result = new List<Plane>();

            for (int i = 0; i < nPlanes; ++i)
            {
                Vector3 center = Internal.readVector3(Internal.planePos, i);
                Vector3 equation = Internal.readVector3(Internal.planeEquation, i);
                Plane plane = new Plane(i, center, equation);
                result.Add(plane);
            }

            return result;
        }
    }

    /** Represents a hand */
    public class Hand
    {
        /** the 3D coordinates at the center of the hand's palm */
        public Vector3 center;

        /** the hand object's average depth (z-coordinate) */
        public float depth;

        /** the hand's dominant direction (direction with largest radius) */
        public Vector2 direction;

        /** the 3D coordinates of the hand's fingertips */
        public Vector3[] fingers;

        /** the 3D coordinates of the hand's defects (bases of fingers) */
        public Vector3[] defects;

        /** the 3D coordinates of the sides of the hand's wrist ([0] is left side, [1] is right) */
        public Vector3[] wrist;

        /** get the indices of the fingers that are touching the given plane
          * @param threshold the square of the 'thickness' of the plane,
          *        i.e. max L2 norm between a finger and the plane
          */
        public int[] touchingPlane(Plane plane, float threshold = 0.0002f)
        {
            Internal.computeTouches(id, plane.id, threshold);
            return Internal.readArray(Internal.touchFingerIndex, Internal.numTouches());
        }

        /** construct a new hand instance */
        public Hand(int id, Vector3 center, float depth, Vector2 direction, Vector3[] fingers,
                    Vector3[] defects, Vector3[] wrist)
        {
            this.id = id;
            this.center = center;
            this.depth = depth;
            this.direction = direction;
            this.fingers = fingers;
            this.defects = defects;
            this.wrist = wrist;
        }

        /** the Hand's unique ID */
        internal int id;
    }

    /** Represents a plane */
    public class Plane
    {
        /** the 3D coordinates of the plane's center of mass */
        public Vector3 center;

        /** the plane's equation (a,b,c): ax + by - z + c = 0 */
        public Vector3 equation;

        /** construct a new plane */
        public Plane(int id, Vector3 center, Vector3 equation)
        {
            this.id = id;
            this.center = center;
            this.equation = equation;
        }

        /** compute distance from this plane to a point */
        public float DistanceToPoint(Vector3 point)
        {
            return Internal.planePointDist(id, point.x, point.y, point.z);
        }

        /** compute L2 norm from this plane to a point */
        public float NormToPoint(Vector3 point)
        {
            return Internal.planePointNorm(id, point.x, point.y, point.z);
        }

        /** the Plane's unique ID */
        internal int id;
    }

    internal static class Internal
    {
        /** name of OpenARK DLL */
        const string OPENARK_DLL = "openark_unity_0_9_3_native";

        /*** CAMERA ***/
        /** Connect to and begin capturing from the depth camera */
        [DllImport(OPENARK_DLL)]
        public static extern void beginCapture();

        /** Stop capturing from the camera. You may call beginCapture() again afterwards. */
        [DllImport(OPENARK_DLL)]
        public static extern void endCapture();

        /*** DETECTION ***/
        /** Update the detector */
        [DllImport(OPENARK_DLL)]
        public static extern void update();

        /** Returns the number of hands found in the current frame. */
        [DllImport(OPENARK_DLL)]
        public static extern int numHands();

        /** Returns the number of planes found in the current frame. */
        [DllImport(OPENARK_DLL)]
        public static extern int numPlanes();

        /*** HAND ***/
        /** Get the central coordinates of hand 'hand_id' (meters)
          * @param axis 0:x 1:y 2:z coordinates
          */
        [DllImport(OPENARK_DLL)]
        public static extern float handPos(int hand_id, int axis);

        /** Get the hand's average depth (meters) */
        [DllImport(OPENARK_DLL)]
        public static extern float handAvgDepth(int hand_id);

        /** Get the hand's dominant direction vector
          * @param axis 0:x 1:y
          * @return 2D unit vector in the hand's dominant direction
          */
        [DllImport(OPENARK_DLL)]
        public static extern float handDirection(int hand_id, int axis);

        /** Get the number of fingers on hand 'hand_id' */
        [DllImport(OPENARK_DLL)]
        public static extern int handNumFingers(int hand_id);

        /*** WRIST ***/
        /** Get the coordinates of the sides of the hand's wrist
         * @param hand_id id of hand
         * @param index 0: left side of wrist  1: right side of rist
         * @param axis 0:x 1:y 2:z coordinates
         */
        [DllImport(OPENARK_DLL)]
        public static extern float handWristPos(int hand_id, int index, int axis);

        /*** FINGERS ***/
        /** Get the coordinates of 'index' th finger on hand 'hand_id'
          * @param axis 0:x 1:y 2:z coordinates
          */
        [DllImport(OPENARK_DLL)]
        public static extern float handFingerPos(int hand_id, int index, int axis);

        /** Get the coordinates of 'index'th finger on hand 'hand_id' 
          * @param axis 0:x 1:y 2:z coordinates
          */
        [DllImport(OPENARK_DLL)]
        public static extern float handDefectPos(int hand_id, int index, int axis);

        /*** PLANE ***/
        /** Get the coordinates at the center of mass of plane 'plane_id'
          * @param axis 0:x 1:y 2:z coordinates
          */
        [DllImport(OPENARK_DLL)]
        public static extern float planePos(int plane_id, int axis);

        /** Get the equation of plane 'plane_id': ax + by - z + c = 0
          * @param term 0:a 1:b 2:c
          */
        [DllImport(OPENARK_DLL)]
        public static extern float planeEquation(int plane_id, int term);

        /** Get the euclidean distance from a plane to a point */
        [DllImport(OPENARK_DLL)]
        public static extern float planePointDist(int plane_id, float x, float y, float z);

        /** Get the L2 norm from a plane to a point */
        [DllImport(OPENARK_DLL)]
        public static extern float planePointNorm(int plane_id, float x, float y, float z);

        /** Compute contact points between a hand and a plane
          * @see numTouches
          * @see touchIndex
          * @see touchPos
          * @param thresh max L2 norm between two points to consider them to be touching (m^2)
          * @return number of contact points found
          */
        [DllImport(OPENARK_DLL)]
        public static extern int computeTouches(int hand_id, int plane_id, float thresh);

        /** Get the number of finger-plane contact points found by computeTouches.
          * @see computeTouches
          */
        [DllImport(OPENARK_DLL)]
        public static extern int numTouches();

        /** Get the finger index of the 'touch_id' th finger-plane contact point found by computeTouches.
          * @see computeTouches
          * @see handFingerPos
          */
        [DllImport(OPENARK_DLL)]
        public static extern int touchFingerIndex(int touch_id);

        /** Get the 3D coordinates of the 'touch_id' th finger-plane contact point found by computeTouches.
          * @see computeTouches
          * @see handFingerPos
          * @param axis 0:x 1:y 2:z coordinates
          */
        [DllImport(OPENARK_DLL)]
        public static extern float touchPos(int touch_id, int axis);

        /** set whether the hand detector should use the SVM to eliminate false positives */
        [DllImport(OPENARK_DLL)]
        public static extern void handUseSVM(bool value);

        /** set whether the hand detector should eliminate hands not connected to the edge of the screen */
        [DllImport(OPENARK_DLL)]
        public static extern void handRequireEdgeConnected(bool value);

        /** read a Vector3 from the given function */
        public static Vector3 readVector3(Func<int, int, float> fn, int a)
        {
            return new Vector3(fn(a, 0), fn(a, 1), fn(a, 2));
        }

        /** read a Vector3 from the given function */
        public static Vector3 readVector3(Func<int, int, int, float> fn, int a, int b)
        {
            return new Vector3(fn(a, b, 0), fn(a, b, 1), fn(a, b, 2));
        }

        /** read a Vector2 from the given function */
        public static Vector2 readVector2(Func<int, int, float> fn, int a)
        {
            return new Vector2(fn(a, 0), fn(a, 1));
        }

        /** read a list of Vector3 from the given function */
        public static Vector3[] readArray(Func<int, int, int, float> fn, int a, int len)
        {
            Vector3[] result = new Vector3[len];
            for (int i = 0; i < len; ++i)
            {
                result[i] = readVector3(fn, a, i);
            }
            return result;
        }

        /** read an array of Vector3 from the given function */
        public static Vector3[] readArray(Func<int, int, float> fn, int len)
        {
            Vector3[] result = new Vector3[len];
            for (int i = 0; i < len; ++i)
            {
                result[i] = readVector3(fn, i);
            }
            return result;
        }

        /** read an array of int from the given function */
        public static int[] readArray(Func<int, int> fn, int len)
        {
            int[] result = new int[len];
            for (int i = 0; i < len; ++i)
            {
                result[i] = fn(i);
            }
            return result;
        }
    }
}
