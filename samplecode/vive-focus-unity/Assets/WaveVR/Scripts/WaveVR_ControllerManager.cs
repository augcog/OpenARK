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

public class WaveVR_ControllerManager : MonoBehaviour
{
    private static string LOG_TAG = "WaveVR_ControllerManager";
    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " " + msg);
        #endif
        Log.d (LOG_TAG, msg);
    }

    public GameObject right, left;

    public enum CIndex
    {
        invalid = -1,
        right = 0,
        left = 1
    }
    private GameObject[] ControllerObjects; // populate with objects you want to assign to additional controllers
    private bool[] ControllerConnected = new bool[2]{false, false};

    #region Override functions
    void Awake()
    {
        var objects = new GameObject[2];
        objects [(uint)CIndex.right] = right;
        objects [(uint)CIndex.left] = left;

        this.ControllerObjects = objects;
    }

    void OnEnable()
    {
        for (int i = 0; i < ControllerObjects.Length; i++)
        {
            var obj = ControllerObjects[i];
            if (obj != null)
            {
                PrintDebugLog ("OnEnable() disable controller " + i);
                obj.SetActive (false);
            }
        }

        for (int i = 0; i < WaveVR.DeviceTypes.Length; i++)
        {
            #if UNITY_EDITOR
            if (WaveVR_Controller.Input(WaveVR.DeviceTypes[i]).connected)
            #else
            if (WaveVR.Instance.connected [i])
            #endif
            {
                PrintDebugLog ("OnEnable() device " + WaveVR.DeviceTypes [i] + " is connected.");
                onDeviceConnected (WaveVR.DeviceTypes [i], true);
            }
        }

        WaveVR_Utils.Event.Listen(WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
    }

    void OnDisable()
    {
        WaveVR_Utils.Event.Remove(WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
    }
    #endregion

    private void BroadcastToObjects(CIndex _index)
    {
        WVR_DeviceType deviceIndex = WVR_DeviceType.WVR_DeviceType_Controller_Right;

        var obj = ControllerObjects [(uint)_index];
        if (obj != null)
        {
            if (ControllerConnected [(uint)_index] == false)
            {
                PrintDebugLog ("BroadcastToObjects() disable controller " + (int)_index);
                obj.SetActive (false);
            } else
            {
                PrintDebugLog ("BroadcastToObjects() enable controller " + (int)_index);
                // means object with _index is not null and connected.
                obj.SetActive(true);
                deviceIndex = _index == CIndex.right ?
                    WVR_DeviceType.WVR_DeviceType_Controller_Right :
                    WVR_DeviceType.WVR_DeviceType_Controller_Left; 

                obj.BroadcastMessage("SetDeviceIndex", deviceIndex, SendMessageOptions.DontRequireReceiver);
            }
        }
    }

    private void onDeviceConnected(params object[] args)
    {
        var device = (WVR_DeviceType)args[0];
        var connected = (bool)args[1];
        PrintDebugLog ("onDeviceConnected() device " + device + " is " + (connected == true ? "connected" : "disconnected")
            + ", left-handed? " + WaveVR_Controller.IsLeftHanded);

        if (false == WaveVR_Controller.IsLeftHanded)
        {
            if (device == WVR_DeviceType.WVR_DeviceType_Controller_Right)
            {
                if (ControllerConnected [(uint)CIndex.right] != connected)
                {   // Connection status has been changed.
                    ControllerConnected [(uint)CIndex.right] = connected;
                    BroadcastToObjects (CIndex.right);
                }
            } else if (device == WVR_DeviceType.WVR_DeviceType_Controller_Left)
            {
                if (ControllerConnected [(uint)CIndex.left] != connected)
                {   // Connection status has been changed.
                    ControllerConnected [(uint)CIndex.left] = connected;
                    BroadcastToObjects (CIndex.left);
                }
            }
        } else
        {
            if (device == WVR_DeviceType.WVR_DeviceType_Controller_Left)
            {
                if (ControllerConnected [(uint)CIndex.right] != connected)
                {   // Connection status has been changed.
                    ControllerConnected [(uint)CIndex.right] = connected;
                    BroadcastToObjects (CIndex.right);
                }
            } else if (device == WVR_DeviceType.WVR_DeviceType_Controller_Right)
            {
                if (ControllerConnected [(uint)CIndex.left] != connected)
                {   // Connection status has been changed.
                    ControllerConnected [(uint)CIndex.left] = connected;
                    BroadcastToObjects (CIndex.left);
                }
            }
        }
    }

}
