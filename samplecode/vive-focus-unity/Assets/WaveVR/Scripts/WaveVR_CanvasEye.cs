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
using System.Collections;
[RequireComponent(typeof(Canvas))]
public class WaveVR_CanvasEye : MonoBehaviour {
    private Canvas canvas;
    private delegate void OnChangeEyeCallback(Camera eyeCamera);
    private static OnChangeEyeCallback listeners;
    // Use this for initialization
    void Start () {
        canvas = GetComponent<Canvas>();
    }

    void OnEnable()
    {
        listeners += OnChangeEye;
    }

    void OnDisable()
    {
        listeners -= OnChangeEye;
    }

    public static void changeEye(Camera eyeCamera)
    {
        if (listeners == null)
            return;
        listeners(eyeCamera);
    }

    private void OnChangeEye(Camera eyeCamera)
    {
        if (eyeCamera == null)
            return;
        canvas.renderMode = RenderMode.ScreenSpaceCamera;
        canvas.worldCamera = eyeCamera;
        //canvas.planeDistance = eyeCamera.nearClipPlane + 0.1f;
	}
}
