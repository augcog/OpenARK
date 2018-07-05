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
using wvr;
using WaveVR_Log;

public class ControllerConnectionStateReactor : MonoBehaviour
{
    private static string LOG_TAG = "WaveVRConnReactor";
    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " : " + this.type + ", " + msg);
        #endif
        Log.d (LOG_TAG, this.type + ", " + msg);
    }

    public WVR_DeviceType type;
    private bool connected = false;
    public List<GameObject> targetGameObjects = new List<GameObject>();
    private bool mFocusCapturedBySystem = false;

    void OnEnable()
    {
        if (!Application.isEditor)
        {
            WaveVR_Utils.Event.Listen (WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
            if (checkConnection () != connected)
                connected = !connected;
            setActive (connected && (!mFocusCapturedBySystem));
        }
    }

    void OnDisable()
    {
        if (!Application.isEditor)
        {
            WaveVR_Utils.Event.Remove (WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
        }
    }

    void Update ()
    {
        if (!connected) return;

        bool focusCaptured = Interop.WVR_IsInputFocusCapturedBySystem();

        if (focusCaptured != mFocusCapturedBySystem)
        {
            // InputFocus changed!
            mFocusCapturedBySystem = focusCaptured;

            PrintDebugLog ("Focus is " + (mFocusCapturedBySystem == true ? " captured by system" : " not captured"));

            setActive(!mFocusCapturedBySystem);
        }
    }

    private bool checkConnection()
    {
        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            return false;
        } else
        #endif
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.type);
            return _device.connected;
        }
    }

    private void setActive(bool active)
    {
        foreach (var go in targetGameObjects)
        {
            if (go != null) go.SetActive(active);
        }
    }

    private void onDeviceConnected(params object[] args)
    {
        bool _connected = false;
        WVR_DeviceType _type = this.type;

        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            _connected = WaveVR_Controller.Input (this.type).connected;
            _type = WaveVR_Controller.Input(this.type).DeviceType;
        }
        else
        #endif
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.type);
            _connected = _device.connected;
            _type = _device.type;
        }

        PrintDebugLog ("onDeviceConnected() " + _type + " is " + (_connected ? "connected" : "disconnected") + ", left-handed? " + WaveVR_Controller.IsLeftHanded);

        if (connected != _connected)
        {
            connected = _connected;
            setActive (connected && (!mFocusCapturedBySystem));
        }
    }
}
