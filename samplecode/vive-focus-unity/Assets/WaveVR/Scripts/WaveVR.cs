// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using UnityEngine;
using UnityEngine.Assertions;
using wvr;
using WaveVR_Log;
using System;

public class WaveVR : System.IDisposable
{
    [SerializeField]
    public bool editor = false;
    private static string LOG_TAG = "WVR_WaveVR";

    public static WaveVR Instance
    {
        get
        {
            if (instance == null)
                instance = Application.isEditor ? null : new WaveVR();
            return instance;
        }
    }
    private static WaveVR instance = null;

    public class Device
    {
        public Device(WVR_DeviceType type)
        {
            this.type = type;
            for (int i = 0; i < DeviceTypes.Length; i++)
            {
                if (DeviceTypes[i] == type)
                {
                    index = i;
                    break;
                }
            }
        }
        public WVR_DeviceType type { get; private set; }
        public int index { get; private set; }
        public bool connected { get { return instance.connected[index]; } }
        public WVR_DevicePosePair_t pose { get { return instance.poses[instance.deviceIndexMap[index]]; } }
        public WaveVR_Utils.RigidTransform rigidTransform { get { return instance.rtPoses[instance.deviceIndexMap[index]]; } }
    }

    public Device hmd { get; private set; }
    public Device controllerLeft { get; private set; }
    public Device controllerRight { get; private set; }

    public Device getDeviceByType(WVR_DeviceType type)
    {
        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_HMD:
            return hmd;
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            return WaveVR_Controller.IsLeftHanded ? controllerLeft : controllerRight;
        case WVR_DeviceType.WVR_DeviceType_Controller_Left:
            return WaveVR_Controller.IsLeftHanded ? controllerRight : controllerLeft;
        default:
            Assert.raiseExceptions = true;
            return hmd;  // Should not happen
        }
    }

    private static void ReportError(WVR_InitError error)
    {
        switch (error)
        {
            case WVR_InitError.WVR_InitError_None:
                break;
            case WVR_InitError.WVR_InitError_NotInitialized:
                Log.e(LOG_TAG, "WaveVR: Not initialized");
                Application.Quit();
                break;
            case WVR_InitError.WVR_InitError_Unknown:
                Log.e(LOG_TAG, "WaveVR: Unknown error during initializing");
                break;
            default:
                //TODO Log.e(LOG_TAG, Interop.WVR_GetErrorString(error));
                break;
        }
    }

    [System.Obsolete("Please check WaveVR.Instance directly")]
    public static bool Hmd
    {
        get
        {
            return Instance != null;
        }
    }

    public static WVR_DeviceType[] DeviceTypes = new WVR_DeviceType[]{
        WVR_DeviceType.WVR_DeviceType_HMD,
        WVR_DeviceType.WVR_DeviceType_Controller_Right,
        WVR_DeviceType.WVR_DeviceType_Controller_Left
    };

    public bool[] connected = new bool[DeviceTypes.Length];
    public uint[] deviceIndexMap = new uint[DeviceTypes.Length];  // Mapping from DeviceTypes's index to poses's index

    private WVR_DevicePosePair_t[] poses = new WVR_DevicePosePair_t[DeviceTypes.Length];  // HMD, R, L controllers.
    private WaveVR_Utils.RigidTransform[] rtPoses = new WaveVR_Utils.RigidTransform[DeviceTypes.Length];

    private WaveVR()
    {
        Log.d(LOG_TAG, "WaveVR()+");

        WVR_InitError error = Interop.WVR_Init(WVR_AppType.WVR_AppType_VRContent);
        if (error != WVR_InitError.WVR_InitError_None)
        {
            ReportError(error);
            Interop.WVR_Quit();
            Debug.Log("WVR_Quit");
            return;
        }
        WaveVR_Utils.notifyActivityUnityStarted();

        for (int i = 0; i < 3; i++)
        {
            poses[i] = new WVR_DevicePosePair_t();
            connected[i] = false; // force update connection status to all listener.
            deviceIndexMap[i] = 0;  // use hmd's id as default.
        }

        hmd = new Device(WVR_DeviceType.WVR_DeviceType_HMD);
        controllerLeft = new Device(WVR_DeviceType.WVR_DeviceType_Controller_Left);
        controllerRight = new Device(WVR_DeviceType.WVR_DeviceType_Controller_Right);

        Log.d(LOG_TAG, "WaveVR()-");
    }

    ~WaveVR()
    {
        Dispose();
    }

    public void onLoadLevel()
    {
        Log.i (LOG_TAG, "onLoadLevel() reset all connection");
        for (int i = 0; i < DeviceTypes.Length; i++)
        {
            poses[i] = new WVR_DevicePosePair_t();
            connected[i] = false; // force update connection status to all listener.
        }
    }

    public void OnControllerRoleChange()
    {
        int _left = Array.IndexOf (DeviceTypes, WVR_DeviceType.WVR_DeviceType_Controller_Left);
        Log.i (LOG_TAG, "OnControllerRoleChange() device " + DeviceTypes [_left] + " is " + (connected [_left] ? "connected" : "disconnected"));
        WaveVR_Utils.Event.Send (WaveVR_Utils.Event.DEVICE_CONNECTED, DeviceTypes [_left], connected [_left]);

        int _right = Array.IndexOf (DeviceTypes, WVR_DeviceType.WVR_DeviceType_Controller_Right);
        Log.i (LOG_TAG, "OnControllerRoleChange() device " + DeviceTypes [_right] + " is " + (connected [_right] ? "connected" : "disconnected"));
        WaveVR_Utils.Event.Send (WaveVR_Utils.Event.DEVICE_CONNECTED, DeviceTypes [_right], connected [_right]);
    }

    public void Dispose()
    {
        Interop.WVR_Quit();
        Debug.Log("WVR_Quit");
        instance = null;
        System.GC.SuppressFinalize(this);
    }

    // Use this interface to avoid accidentally creating the instance 
    // in the process of attempting to dispose of it.
    public static void SafeDispose()
    {
        if (instance != null)
            instance.Dispose();
    }

    // Use this interface to check what kind of dof is running
    public int is6DoFTracking()
    {
        WVR_NumDoF dof = Interop.WVR_GetDegreeOfFreedom(WVR_DeviceType.WVR_DeviceType_HMD);

        if (dof == WVR_NumDoF.WVR_NumDoF_6DoF)
            return 6;  // 6 DoF
        else if (dof == WVR_NumDoF.WVR_NumDoF_3DoF)
            return 3;  // 3 DoF
        else
            return 0;  // abnormal case
    }

    public void UpdatePoses(WVR_PoseOriginModel origin)
    {
        Log.gpl.d(LOG_TAG, "UpdatePoses");
        Interop.WVR_GetSyncPose(origin, poses, (uint)poses.Length);

        for (uint i = 0; i < DeviceTypes.Length; i++)
        {
            bool _hasType = false;

            for (uint j = 0; j < poses.Length; j++)
            {
                WVR_DevicePosePair_t _pose = poses[j];

                if (_pose.type == DeviceTypes [i])
                {
                    _hasType = true;
                    deviceIndexMap[i] = j;

                    if (connected [i] != _pose.pose.IsValidPose)
                    {
                        connected [i] = _pose.pose.IsValidPose;
                        Log.i (LOG_TAG, "device " + DeviceTypes [i] + " is " + (connected [i] ? "connected" : "disconnected"));
                        WaveVR_Utils.Event.Send(WaveVR_Utils.Event.DEVICE_CONNECTED, DeviceTypes [i], connected[i]);
                    }

                    if (connected [i])
                    {
                        rtPoses[j].update(_pose.pose.PoseMatrix);
                    }

                    break;
                }
            }

            // no such type
            if (!_hasType)
            {
                if (connected [i] == true)
                {
                    connected [i] = false;
                    Log.i (LOG_TAG, "device " + DeviceTypes [i] + " is disconnected.");
                    WaveVR_Utils.Event.Send(WaveVR_Utils.Event.DEVICE_CONNECTED, DeviceTypes [i], connected[i]);
                }
            }
        }

        for (int i = 0; i < poses.Length; i++)
        {
            WVR_DeviceType _type = poses [i].type;
            bool _connected = Interop.WVR_IsDeviceConnected (_type);
            bool _posevalid = poses [i].pose.IsValidPose;

            Log.gpl.d (LOG_TAG, "Device " + _type + " is " + (_connected ? "connected" : "disconnected")
                + ", pose is " + (_posevalid ? "valid" : "invalid")
                + ", pos: {" + rtPoses [i].pos.x + ", " + rtPoses [i].pos.y + ", " + rtPoses [i].pos.z + "}"
                + ", rot: {" + rtPoses [i].rot.x + ", " + rtPoses [i].rot.y + ", " + rtPoses [i].rot.z + ", " + rtPoses [i].rot.w + "}");
        }

        try
        {
            WaveVR_Utils.Event.Send(WaveVR_Utils.Event.NEW_POSES, poses, rtPoses);
        }
        catch (Exception ex)
        {
            Log.e(LOG_TAG, "Send NEW_POSES Event Exception : " + ex);
        }
        Log.gpl.d(LOG_TAG, "after new poses");
        try
        {
            WaveVR_Utils.Event.Send(WaveVR_Utils.Event.AFTER_NEW_POSES);
        }
        catch (Exception ex)
        {
            Log.e(LOG_TAG, "Send AFTER_NEW_POSES Event Exception : " + ex);
        }
    }

    public int SetQualityLevel(int level, bool applyExpensiveChanges = true)
    {
        return WaveVR_Render.Instance.SetQualityLevel(level, applyExpensiveChanges);
    }
}

