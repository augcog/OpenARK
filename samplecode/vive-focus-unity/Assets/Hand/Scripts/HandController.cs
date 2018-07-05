using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WaveVR_Log;

public class HandController : MonoBehaviour {
    private static string LOG_TAG = "HandController";
    public List<Transform> FingerTransforms;

    internal StreamingAverager Center;
    internal StreamingAverager WristL, WristR;
    internal List<StreamingAverager> Fingers;
    internal int numFingers = 0;
    internal float speed = 0.0f;

    private StreamingAverager numFingersAvg;
    private int invalidCount = 0;

    // if more than this many invalid frames are received, hides hand
    private const int MAX_INVALID_COUNT = 400;

    private Rigidbody rigidBody;
    private Vector3 prevPos;

    public enum HandPose
    {
        None = -1, Open = 0, Pointer, Grabbing
    }

    private HandPose _pose = HandPose.None;
    public HandPose Pose {
        get
        {
            return _pose;
        }
        set
        {
            if (_pose == value) return;
            _pose = value;
            switch(value)
            {
                case HandPose.Open:
                    FingerTransforms[0].localRotation = new Quaternion(0.35544130f, 0.92423210f, -0.02134409f, 0.13784370f);
                    FingerTransforms[1].localRotation = new Quaternion(0.07915623f, 0.72873750f, 0.67304250f, 0.09843646f);
                    FingerTransforms[2].localRotation = new Quaternion(0.01273032f, 0.64875730f, 0.76042840f, 0.02646718f);
                    FingerTransforms[3].localRotation = new Quaternion(0.04904802f, -0.57624870f, -0.81461720f, 0.04393831f);
                    FingerTransforms[4].localRotation = new Quaternion(-0.47228790f, -0.06413012f, 0.12191700f, 0.87061350f);
                    break;
                case HandPose.Pointer:
                    FingerTransforms[0].localRotation = new Quaternion(0.36726420f, 0.66298450f, -0.53157420f, 0.37815000f);
                    FingerTransforms[1].localRotation = new Quaternion(0.12994600f, 0.74990270f, 0.64814320f, 0.02589393f);
                    FingerTransforms[2].localRotation = new Quaternion(-0.40399800f, 0.52294150f, 0.64058800f, 0.39110730f);
                    FingerTransforms[3].localRotation = new Quaternion(0.43244750f, -0.47840080f, -0.70148040f, -0.30339270f);
                    FingerTransforms[4].localRotation = new Quaternion(-0.14327840f, -0.34867640f, 0.03077803f, 0.92571530f);
                    break;
                case HandPose.Grabbing:
                    FingerTransforms[0].localRotation = new Quaternion(0.33354100f, 0.41231350f, -0.75541910f, 0.38482470f);
                    FingerTransforms[1].localRotation = new Quaternion(-0.33879600f, 0.61754500f, 0.61759480f, 0.34990310f);
                    FingerTransforms[2].localRotation = new Quaternion(-0.35274300f, 0.57745930f, 0.63239190f, 0.37708570f);
                    FingerTransforms[3].localRotation = new Quaternion(0.37326990f, -0.48506610f, -0.73083210f, -0.30210080f);
                    FingerTransforms[4].localRotation = new Quaternion(-0.53906130f, -0.25284260f, -0.15787560f, 0.78775550F);
                    break;
            }
        }
    }
    public HandPose StartPose;

    private GameObject childHand;

	void Start () {
        rigidBody = GetComponent<Rigidbody>();
        childHand = transform.GetChild(0).gameObject;

        // initialize streaming averagers
        numFingersAvg = new StreamingAverager(new Vector3(2.0f, 0.0f, 0.0f), 11);
        Center = new StreamingAverager(new Vector3(0, -20.0f, 140f));
        WristL = new StreamingAverager(new Vector3(-0.5f, -25.3f, 135.3f));
        WristR = new StreamingAverager(new Vector3(0.5f, -25.3f, 135.3f));
        Fingers = new List<StreamingAverager>(5);
        for (int i = 0; i < 5; ++i)
        {
            Fingers.Add(new StreamingAverager(new Vector3(i - 2.5f, 0, 150.3f)));
        }
        Pose = StartPose;
        childHand.SetActive(false);
    }

    void FixedUpdate () {
        speed = (transform.position - prevPos).magnitude; // compute speed
        prevPos = transform.position;
        if (Center.Count == 0) return;

        Log.e(LOG_TAG, "Updating... Center num points: " + Center.Count.ToString() + " Avg:" + Center.Point.ToString());
        numFingersAvg.addPoint(new Vector3(numFingers, 0, 0));
        if (Pose == HandPose.Open && numFingersAvg.Point.x < 1.05f)
        {
            Pose = HandPose.Pointer;
        }
        else if (Pose == HandPose.Pointer && numFingersAvg.Point.x > 2.8f)
        {
            Pose = HandPose.Open;
        }

        // rotate and translate to real hand location
        Vector3 targetPos = Center.Point;
        rigidBody.MovePosition(targetPos);

        Vector3 wristRPt = WristR.Point, wristLPt = WristL.Point;
        Vector3 wristCen = (wristLPt + wristRPt) * 0.5f;

        Vector3 rightDir = wristRPt - wristLPt, upDir = Center.Point - wristCen;
        Vector3 forwardDir = -Vector3.Cross(upDir, rightDir);

        Quaternion targetRot = Quaternion.LookRotation(forwardDir, upDir);
        rigidBody.MoveRotation(targetRot);
    }

    void Update()
    {
        if (Center.Count > 0)
        {
            invalidCount = 0;
            childHand.SetActive(true);
        }
        else
        {
            ++invalidCount;
            if (invalidCount > MAX_INVALID_COUNT)
            {
                childHand.SetActive(false);
            }
        }
    }
}
