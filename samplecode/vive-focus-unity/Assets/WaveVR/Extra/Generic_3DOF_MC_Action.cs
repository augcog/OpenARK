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

public class Generic_3DOF_MC_Action : MonoBehaviour
{
    private static string LOG_TAG = "Generic_3DOF_MC_Action";

    public WVR_DeviceType device = WVR_DeviceType.WVR_DeviceType_Controller_Right;
    private GameObject VolumeUpButtonPress_Effect = null;
    private GameObject VolumeDownButtonPress_Effect = null;
    public GameObject MenuButtonPress_Effect = null;
    public GameObject Touch_Effect = null;
    private Vector3 originPosition;

    // Use this for initialization
    void Start()
    {
        if (VolumeUpButtonPress_Effect != null)
        {
            VolumeUpButtonPress_Effect.SetActive(false);
        }
        if (VolumeDownButtonPress_Effect != null)
        {
            VolumeDownButtonPress_Effect.SetActive(false);
        }
        if (MenuButtonPress_Effect != null)
        {
            MenuButtonPress_Effect.SetActive(false);
        }
        if (Touch_Effect != null)
        {
            originPosition = Touch_Effect.transform.localPosition;
            Touch_Effect.SetActive(false);
        }
    }

    // Update is called once per frame
    void Update()
    {
        //WVR_InputId_Alias1_Menu
        if (WaveVR_Controller.Input(device).GetPressDown(WVR_InputId.WVR_InputId_Alias1_Menu))
        {
            if (MenuButtonPress_Effect != null)
            {
                MenuButtonPress_Effect.SetActive(true);
            }
        }
        // button pressed
        if (WaveVR_Controller.Input(device).GetPress(WVR_InputId.WVR_InputId_Alias1_Menu))
        {
            if (MenuButtonPress_Effect != null)
            {
                MenuButtonPress_Effect.SetActive(true);
            }
        }
        if (WaveVR_Controller.Input(device).GetPressUp(WVR_InputId.WVR_InputId_Alias1_Menu))
        {
            if (MenuButtonPress_Effect != null)
            {
                MenuButtonPress_Effect.SetActive(false);
            }
        }

        //WVR_InputId_Alias1_Touchpad
        if (WaveVR_Controller.Input(device).GetPressDown(WVR_InputId.WVR_InputId_Alias1_Touchpad))
        {
            if (Touch_Effect != null)
            {
                Touch_Effect.SetActive(true);
            }
        }

        if (WaveVR_Controller.Input(device).GetPress(WVR_InputId.WVR_InputId_Alias1_Touchpad))
        {
            if (Touch_Effect != null)
            {
                Touch_Effect.SetActive(true);
            }
        }

        if (WaveVR_Controller.Input(device).GetPressUp(WVR_InputId.WVR_InputId_Alias1_Touchpad))
        {
            if (Touch_Effect != null)
            {
                Touch_Effect.SetActive(false);
            }
        }
        // button touch down
        if (WaveVR_Controller.Input(device).GetTouchDown(WVR_InputId.WVR_InputId_Alias1_Touchpad))
        {
            if (Touch_Effect != null)
            {
                Touch_Effect.SetActive(true);
            }
        }

        // button touch up
        if (WaveVR_Controller.Input(device).GetTouchUp(WVR_InputId.WVR_InputId_Alias1_Touchpad))
        {
            if (Touch_Effect != null)
            {
                Touch_Effect.SetActive(false);
            }
        }
        // button touched
        if (WaveVR_Controller.Input(device).GetTouch(WVR_InputId.WVR_InputId_Alias1_Touchpad))
        {
            var axis = WaveVR_Controller.Input(device).GetAxis(WVR_InputId.WVR_InputId_Alias1_Touchpad);

            float xangle = axis.x / 100, yangle = axis.y / 100;
            //Log.d(LOG_TAG, "WVR_InputId_Alias1_Touchpad axis xangle: " + xangle + ", yangle: " + yangle);
            if (xangle > 0.006f) xangle = 0.006f;
            if (xangle < -0.009f) xangle = -0.009f;

            if (yangle > 0.006f) yangle = 0.006f;
            if (yangle < -0.009f) yangle = -0.009f;

            if (Touch_Effect != null)
            {
                var translateVec = new Vector3(xangle, yangle, 0);
                Touch_Effect.transform.localPosition = originPosition + translateVec;
            }
        }

        //WVR_InputId_Alias1_Grip
        if (WaveVR_Controller.Input(device).GetPressDown(WVR_InputId.WVR_InputId_Alias1_Grip))
        {
            Log.d(LOG_TAG, "WVR_InputId.WVR_InputId_Alias1_Grip press down.");
        }
        // button pressed
        if (WaveVR_Controller.Input(device).GetPress(WVR_InputId.WVR_InputId_Alias1_Grip))
        {
            Log.d(LOG_TAG, "WVR_InputId.WVR_InputId_Alias1_Grip press down.");
        }
        if (WaveVR_Controller.Input(device).GetPressUp(WVR_InputId.WVR_InputId_Alias1_Grip))
        {
            Log.d(LOG_TAG, "WVR_InputId.WVR_InputId_Alias1_Grip press down.");
        }

        //WVR_InputId_Alias1_Trigger
        if (WaveVR_Controller.Input(device).GetPressDown(WVR_InputId.WVR_InputId_Alias1_Trigger))
        {
            Log.d(LOG_TAG, "WVR_InputId.WVR_InputId_Alias1_Trigger press down.");
        }
        // button pressed
        if (WaveVR_Controller.Input(device).GetPress(WVR_InputId.WVR_InputId_Alias1_Trigger))
        {
            Log.d(LOG_TAG, "WVR_InputId.WVR_InputId_Alias1_Trigger press down.");
        }
        if (WaveVR_Controller.Input(device).GetPressUp(WVR_InputId.WVR_InputId_Alias1_Trigger))
        {
            Log.d(LOG_TAG, "WVR_InputId.WVR_InputId_Alias1_Trigger press down.");
        }
        // button touch down
        if (WaveVR_Controller.Input(device).GetTouchDown(WVR_InputId.WVR_InputId_Alias1_Trigger))
        {
            // do nothing
        }

        // button touch up
        if (WaveVR_Controller.Input(device).GetTouchUp(WVR_InputId.WVR_InputId_Alias1_Trigger))
        {
            // do nothing
        }
        // button touched
        if (WaveVR_Controller.Input(device).GetTouch(WVR_InputId.WVR_InputId_Alias1_Trigger))
        {
            // do nothing
        }
    }
}
