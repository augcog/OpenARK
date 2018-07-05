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
using System.Runtime.InteropServices;
using UnityEngine;
using wvr;

[RequireComponent(typeof(Camera))]
public class WaveVR_Camera : MonoBehaviour
{
    public WVR_Eye eye;

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
    }

    public Camera getCamera()
    {
        return cam;
    }

    void OnPreRender()
    {
        if (Camera.current == cam && eye == WVR_Eye.WVR_Eye_Left)
        {
            WaveVR_Utils.Event.Send(WaveVR_Utils.Event.PRE_RENDER_OBJECT_LEFT);
        }
        else if (Camera.current == cam && eye == WVR_Eye.WVR_Eye_Right)
        {
            WaveVR_Utils.Event.Send(WaveVR_Utils.Event.PRE_RENDER_OBJECT_RIGHT);
        }
    }

    void OnRenderObject()
	{
		if (Camera.current == cam && eye == WVR_Eye.WVR_Eye_Left) {
			WaveVR_Utils.Event.Send(WaveVR_Utils.Event.RENDER_OBJECT_LEFT);
		} else if (Camera.current == cam && eye == WVR_Eye.WVR_Eye_Right) {
			WaveVR_Utils.Event.Send(WaveVR_Utils.Event.RENDER_OBJECT_RIGHT);
		}
	}
}
