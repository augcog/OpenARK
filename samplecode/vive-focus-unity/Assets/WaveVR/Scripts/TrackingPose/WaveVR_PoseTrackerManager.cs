using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;
using WaveVR_Log;

#if UNITY_EDITOR
using UnityEditor;

[CustomEditor(typeof(WaveVR_PoseTrackerManager))]
public class WaveVR_PoseTrackerManagerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        WaveVR_PoseTrackerManager myScript = target as WaveVR_PoseTrackerManager;

        myScript.Type = (WVR_DeviceType)EditorGUILayout.EnumPopup ("Type", myScript.Type);
        myScript.TrackPosition = EditorGUILayout.Toggle ("Track Position", myScript.TrackPosition);
        if (true == myScript.TrackPosition)
        {
            myScript.SimulationOption = (WVR_SimulationOption)EditorGUILayout.EnumPopup ("    Simulate Position", myScript.SimulationOption);
            if (myScript.SimulationOption == WVR_SimulationOption.ForceSimulation || myScript.SimulationOption == WVR_SimulationOption.WhenNoPosition)
            {
                myScript.FollowHead = (bool)EditorGUILayout.Toggle ("        Follow Head", myScript.FollowHead);
            }
        }

        myScript.TrackRotation = EditorGUILayout.Toggle ("Track Rotation", myScript.TrackRotation);
        myScript.TrackTiming = (WVR_TrackTiming)EditorGUILayout.EnumPopup ("Track Timing", myScript.TrackTiming);

        if (GUI.changed)
            EditorUtility.SetDirty ((WaveVR_PoseTrackerManager)target);
    }
}
#endif

public enum WVR_TrackTiming {
    WhenUpdate,  // Pose will delay one frame.
    WhenNewPoses
};

public enum WVR_SimulationOption
{
    WhenNoPosition,
    ForceSimulation,
    NoSimulation
};

public class WaveVR_PoseTrackerManager : MonoBehaviour
{
    private const string LOG_TAG = "WaveVR_PoseTrackerManager";
    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + ": Type: " + Type + ", " + msg);
        #endif
        Log.d (LOG_TAG, "Type: " + Type + ", " + msg);
    }

    public WVR_DeviceType Type;
    public bool TrackPosition = true;
    public WVR_SimulationOption SimulationOption = WVR_SimulationOption.WhenNoPosition;
    public bool FollowHead = false;
    public bool TrackRotation = true;
    public WVR_TrackTiming TrackTiming = WVR_TrackTiming.WhenNewPoses;

    private GameObject[] IncludedObjects;
    private bool[] IncludedStates;

    private bool connected = false;
    private bool mFocusCapturedBySystem = false;

    private bool ptmEnabled = false;
    void OnEnable()
    {
        if (!ptmEnabled)
        {
            int _children_count = transform.childCount;
            IncludedObjects = new GameObject[_children_count];
            IncludedStates = new bool[_children_count];
            for (int i = 0; i < _children_count; i++)
            {
                IncludedObjects [i] = transform.GetChild (i).gameObject;
                IncludedStates [i] = transform.GetChild (i).gameObject.activeSelf;
                PrintDebugLog ("OnEnable() " + gameObject.name + " has child: " + IncludedObjects [i].name + ", active? " + IncludedStates [i]);
            }

            #if UNITY_EDITOR
            if (Application.isEditor)
                this.connected = true; // WaveVR_Controller.Input (Type).connected;
            else
            #endif
            {
                // If pose is invalid, considering as disconnected and not show controller.
                WaveVR.Device _device = WaveVR.Instance.getDeviceByType (Type);
                this.connected = _device.connected;
            }
            PrintDebugLog ("OnEnable() is " + (this.connected ? "connected" : "disconnected") + ", mFocusCapturedBySystem: " + mFocusCapturedBySystem);
            ActivateTargetObjects (this.connected && (!mFocusCapturedBySystem));

            WaveVR_Utils.Event.Listen (WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);

            ptmEnabled = true;
        }
    }

    void Awake()
    {
        if (TrackPosition == false)
        {
            SimulationOption = WVR_SimulationOption.NoSimulation;
            FollowHead = false;
        }

        gameObject.SetActive (false);
        PrintDebugLog ("Awake() TrackPosition: " + TrackPosition + ", SimulationOption=" + SimulationOption +
            ", FollowHead: " + FollowHead + ", TrackRotation: " + TrackRotation + ", TrackTiming=" + TrackTiming);

        WaveVR_PointerCameraTracker pcTracker = gameObject.GetComponent<WaveVR_PointerCameraTracker>();
        if (SimulationOption == WVR_SimulationOption.ForceSimulation || SimulationOption == WVR_SimulationOption.WhenNoPosition)
        {
            if (pcTracker == null)
            {
                PrintDebugLog ("Awake() load WaveVR_ControllerPoseTracker.");
                WaveVR_ControllerPoseTracker _cpt = (WaveVR_ControllerPoseTracker)gameObject.AddComponent<WaveVR_ControllerPoseTracker> ();
                if (null != _cpt)
                {
                    _cpt.Type = Type;
                    _cpt.TrackPosition = TrackPosition;
                    _cpt.SimulationOption = SimulationOption;
                    _cpt.FollowHead = FollowHead;
                    _cpt.TrackRotation = TrackRotation;
                    _cpt.TrackTiming = TrackTiming;
                }
            }
        } else
        {
            if (pcTracker == null)
            {
                PrintDebugLog ("Awake() load WaveVR_DevicePoseTracker.");
                WaveVR_DevicePoseTracker _dpt = (WaveVR_DevicePoseTracker)gameObject.AddComponent<WaveVR_DevicePoseTracker> ();
                if (null != _dpt)
                {
                    _dpt.type = Type;
                    _dpt.trackPosition = TrackPosition;
                    _dpt.trackRotation = TrackRotation;
                    _dpt.timing = TrackTiming;
                }
            }
        }
        gameObject.SetActive (true);
    }

    void Update()
    {
        if (!this.connected)
            return;

        bool _focus = false;
        if (!Application.isEditor)
        {
            _focus = Interop.WVR_IsInputFocusCapturedBySystem ();
        }

        if (mFocusCapturedBySystem != _focus)
        {
            // InputFocus changed!
            mFocusCapturedBySystem = _focus;
            PrintDebugLog ("Focus is " + (mFocusCapturedBySystem == true ? "captured by system" : "not captured"));
            ActivateTargetObjects(!mFocusCapturedBySystem);
        }
    }

    void OnDisable()
    {
        // Consider a situation: no pose is updated and WaveVR_PoseTrackerManager is enabled <-> disabled multiple times.
        // At this situation, IncludedStates will be set to false forever since thay are deactivated at 1st time OnEnable()
        // and the deactivated state will be updated to IncludedStates in 2nd time OnEnable().
        // To prevent this situation, activate IncludedObjects in OnDisable to restore the state Children GameObjects.
        PrintDebugLog("OnDisable() restore children objects.");
        ActivateTargetObjects (true);

        WaveVR_Utils.Event.Remove (WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
        ptmEnabled = false;
    }

    private void ActivateTargetObjects(bool active)
    {
        if (IncludedObjects == null)
            return;

        for (int i = 0; i < IncludedObjects.Length; i++)
        {
            if (IncludedObjects [i] == null)
                continue;

            if (IncludedStates [i])
            {
                PrintDebugLog ("ActivateTargetObjects() " + (active ? "activate" : "deactivate") + " " + IncludedObjects [i].name);
                IncludedObjects [i].SetActive (active);
            }
        }
    }

    private void onDeviceConnected(params object[] args)
    {
        if (!ptmEnabled)
        {
            PrintDebugLog ("onDeviceConnected() do NOTHING when disabled.");
            return;
        }

        bool _connected = false;
        WVR_DeviceType _type = this.Type;

        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            _connected = WaveVR_Controller.Input (this.Type).connected;
            _type = WaveVR_Controller.Input(this.Type).DeviceType;
        }
        else
        #endif
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.Type);
            _connected = _device.connected;
            _type = _device.type;
        }

        PrintDebugLog ("onDeviceConnected() " + _type + " is " + (_connected ? "connected" : "disconnected") + ", left-handed? " + WaveVR_Controller.IsLeftHanded);

        if (this.connected != _connected)
        {
            this.connected = _connected;
            ActivateTargetObjects (this.connected && (!mFocusCapturedBySystem));
        }
    }
}
