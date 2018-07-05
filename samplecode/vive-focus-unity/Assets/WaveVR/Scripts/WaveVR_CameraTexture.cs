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
using WaveVR_Log;
using System;
using wvr;

public class WaveVR_CameraTexture
{
    private static string LOG_TAG = "WVR_CameraTexture";

    private WVR_CameraInfo_t camerainfo;
    private bool mStarted = false;
    private uint nativeTextureId = 0;
    private float spentTime = Time.time;
    public bool isStarted
    {
        get
        {
            return mStarted;
        }
    }

    public delegate void UpdateCameraCompleted(uint nativeTextureId);
    public static event UpdateCameraCompleted UpdateCameraCompletedDelegate = null;

    public delegate void StartCameraCompleted(bool result);
    public static event StartCameraCompleted StartCameraCompletedDelegate = null;

    private static WaveVR_CameraTexture mInstance = null;

    public static WaveVR_CameraTexture instance
    {
        get
        {
            if (mInstance == null)
            {
                mInstance = new WaveVR_CameraTexture();
            }

            return mInstance;
        }
    }

    private void OnStartCameraCompleted(params object[] args)
    {
        mStarted = (bool)args[0];
        if (StartCameraCompletedDelegate != null) StartCameraCompletedDelegate(mStarted);
        if (!mStarted) return ;
        camerainfo = (WVR_CameraInfo_t)args[1];

        Log.d(LOG_TAG, "OnStartCameraCompleted, result = " + mStarted + " type = " + camerainfo.imgType + " width = " + camerainfo.width + " height = " + camerainfo.height);
    }

    private void OnUpdateCameraCompleted(params object[] args)
    {
        bool texUpdated = (bool)args[0];
        Log.d(LOG_TAG, "OnUpdateCameraCompleted, result = " + texUpdated + ", refresh rate = " + (1 / (System.DateTime.Now.Millisecond - spentTime))*1000 + "/sec");

        if (UpdateCameraCompletedDelegate != null)  UpdateCameraCompletedDelegate(nativeTextureId);
    }

    public uint getNativeTextureId()
    {
        if (!mStarted) return 0;
        return nativeTextureId;
    }

    public bool startCamera()
    {
        WaveVR_Utils.Event.Listen("StartCameraCompleted", OnStartCameraCompleted);
        WaveVR_Utils.Event.Listen("UpdateCameraCompleted", OnUpdateCameraCompleted);

        WaveVR_Utils.SendRenderEvent(WaveVR_Utils.RENDEREVENTID_StartCamera);
        return true;
    }

    public WVR_CameraImageType GetCameraImageType()
    {
        return camerainfo.imgType;
    }

    public uint GetCameraImageWidth()
    {
        if (!mStarted) return 0;
        return camerainfo.width;
    }

    public uint GetCameraImageHeight()
    {
        if (!mStarted) return 0;
        return camerainfo.width;
    }

    public void stopCamera()
    {
        if (!mStarted) return ;
        WaveVR_Utils.Event.Remove("StartCameraCompleted", OnStartCameraCompleted);
        WaveVR_Utils.Event.Remove("UpdateCameraCompleted", OnUpdateCameraCompleted);
        WaveVR_Utils.SendRenderEvent(WaveVR_Utils.RENDEREVENTID_StopCamera);
        mStarted = false;
    }

    public void updateTexture(uint textureId)
    {
        nativeTextureId = textureId;
        if (!mStarted)
        {
            Log.d(LOG_TAG, "camera not started yet");
            return;
        }
        spentTime = System.DateTime.Now.Millisecond;
        WaveVR_Utils.SendRenderEvent(WaveVR_Utils.RENDEREVENTID_UpdateCamera);
    }
}
