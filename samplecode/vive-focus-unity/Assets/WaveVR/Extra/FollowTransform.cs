// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowTransform : MonoBehaviour {
    public enum DOF {
        TRACK_6DOF,
        TRACK_3DOF,
    }
    public GameObject target;
    public DOF howTargetTrackHMD = DOF.TRACK_3DOF;
    public bool followPostion = true;
    public bool followRotation = false;

    private WaveVR_DevicePoseTracker inverter;
	
    void OnEnable()
    {
        if (target == null)
        {
            Debug.LogError("FollowTransform havn't set a target");
            enabled = false;
            return;
        }
        var rb = target.GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("FollowTransform's target didn't have a Rigidbody");
            enabled = false;
            return;
        }
        WaveVR_Utils.Event.Listen(WaveVR_Utils.Event.AFTER_NEW_POSES, OnAfterNewPoses);
        Expand();
    }

    void OnDisable()
    {
        WaveVR_Utils.Event.Remove(WaveVR_Utils.Event.AFTER_NEW_POSES, OnAfterNewPoses);
    }

    void Expand()
    {
        // TODO Check if a PoseTracker between this and target.
        inverter = target.GetComponentInChildren<WaveVR_DevicePoseTracker>();
        if (inverter == null)
        {
            var obj = new GameObject("OriginPredictor");
            inverter = obj.AddComponent<WaveVR_DevicePoseTracker>();
            obj.transform.SetParent(target.transform, false);
        }

        inverter.type = wvr.WVR_DeviceType.WVR_DeviceType_HMD;

        if (howTargetTrackHMD == DOF.TRACK_3DOF)
        {
            inverter.trackPosition = false;
        }
        else
        {
            inverter.trackPosition = true;
        }
        inverter.inversePosition = true;
        inverter.inverseRotation = true;
    }

    void OnAfterNewPoses(params object[] args) {
        if (inverter == null)
            return;
        if (followPostion)
        {
            transform.localPosition = inverter.transform.position;
            target.transform.localPosition = Vector3.zero;
        }
        if (followRotation)
        {
            transform.localRotation = inverter.transform.rotation;
            target.transform.localRotation = Quaternion.identity;
        }
    }
}
