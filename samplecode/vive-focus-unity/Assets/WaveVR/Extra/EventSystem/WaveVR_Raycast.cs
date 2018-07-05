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
using wvr;

#if UNITY_EDITOR
using UnityEditor;

[CustomEditor(typeof(WaveVR_Raycast))]
public class WaveVR_RaycastEditor : Editor
{
    override public void OnInspectorGUI()
    {
        var myScript = target as WaveVR_Raycast;

        myScript.index = (WVR_DeviceType)EditorGUILayout.EnumPopup ("Device Index", myScript.index);
        myScript.ListenToDevice = EditorGUILayout.Toggle ("Listen to device", myScript.ListenToDevice);
        myScript.AddLineRenderer = EditorGUILayout.Toggle ("Add LineRenderer", myScript.AddLineRenderer);
    }
}
#endif

/// <summary>
/// This class mainly draws a ray cast of associated tracked object.
/// </summary>
public class WaveVR_Raycast : MonoBehaviour {
    private static string LOG_TAG = "WaveVR_Raycast";
    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " : " + this.index + ", " + msg);
        #endif
        Log.d (LOG_TAG, this.index + ", " + msg);
    }

    #region public variables specified by developer
    /**
     * @brief index of controller device.
     **/
    public WVR_DeviceType index = WVR_DeviceType.WVR_DeviceType_Controller_Right;
    public bool ListenToDevice = true;
    public bool AddLineRenderer = true;
    #endregion

    #region public variables access by other GameObject
    public GameObject raycastObject = null;
    public float distance = 0.0f;
    #endregion

    private LineRenderer lr;

    private void initLineRenderer()
    {
        lr = gameObject.AddComponent<LineRenderer>();
        #if UNITY_5_5_OR_NEWER
        lr.startWidth = 0.02f;
        lr.endWidth = 0.01f;
        #else
        lr.SetWidth (0.02f, 0.01f);
        #endif

        lr.material = new Material (Shader.Find("Particles/Additive")); // in "Always Included Shaders"

        lr.useWorldSpace = true;
        lr.enabled = !ListenToDevice;   // if not listen to device, default enable ray.
    }

    private void shotRaycast()
    {
        Vector3 pos = transform.position;
        Vector3 forward = transform.TransformDirection(new Vector3(0, 0, 1));
        float rayLength = 10.0f;

        RaycastHit hit;
        if (Physics.Raycast (pos, forward, out hit))
        {
            raycastObject = hit.collider.gameObject;
            distance = hit.distance;
        } else
        {
            RaycastHit2D hit2D = Physics2D.Raycast (pos, forward, rayLength);
            if (hit2D.collider)
            {
                raycastObject = hit2D.collider.gameObject;
                distance = hit2D.distance;
            }
            else
                raycastObject = null;
        }
    }

    private void updateLineRenderer()
    {
        Vector3 pos = transform.position;
        Vector3 forward = transform.TransformDirection(new Vector3(0, 0, 1));
        float rayLength = 10.0f;

        Vector3 vertex0 = pos;
        Vector3 vertex1 = forward * rayLength;

        lr.SetPosition(0, vertex0);
        lr.SetPosition(1, vertex1);

        #if UNITY_5_5_OR_NEWER
        lr.startColor = Color.yellow;
        lr.endColor = Color.yellow;
        #else
        lr.SetColors(Color.yellow, Color.yellow);
        #endif
    }

    #region device and RenderModel status
    private void onDeviceConnected(params object[] args)
    {
        bool _connected = false;
        WVR_DeviceType _type = this.index;

        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            _connected = WaveVR_Controller.Input (this.index).connected;
            _type = WaveVR_Controller.Input(this.index).DeviceType;
        }
        else
        #endif
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.index);
            _connected = _device.connected;
            _type = _device.type;
        }

        PrintDebugLog ("onDeviceConnected() " + _type + " is " + (_connected ? "connected" : "disconnected") + ", left-handed? " + WaveVR_Controller.IsLeftHanded);

        if (lr != null)
        {
            lr.enabled = _connected;
        }
    }

    private void OnRenderModelLoaded(params object[] args)
    {
        var i = (int)args[2];
        if (i != (int)index)
            return;

        var success = (bool)args[1];

        #if UNITY_EDITOR
        Debug.Log("set lr.enabled = " + success);
        #endif
        Log.i (LOG_TAG, "set lr.enabled = " + success);

        if (lr != null)
        {
            lr.enabled = success;
        }
    }
    #endregion

    #region override functions
    // Use this for initialization
    void Start () {
        if (AddLineRenderer == true)
        {
            initLineRenderer ();
        }
    }

    // Update is called once per frame
    void Update () {
        shotRaycast ();
        if (AddLineRenderer == true)
        {
            if (lr.enabled == true)
                updateLineRenderer ();
        }
    }

    void onDestroy() {
        Destroy(this.GetComponent<Renderer>().material);
    }

    void OnEnable()
    {
        #if UNITY_EDITOR
        ListenToDevice = false;
        #endif
        if (ListenToDevice)
        {
            for (int i = 0; i < WaveVR.Instance.connected.Length; i++)
            {
                if (WaveVR.Instance.connected [i])
                {
                    Log.i (LOG_TAG, "OnEnable, device " + WaveVR.DeviceTypes[i] + " is connected.");
                    onDeviceConnected (WaveVR.DeviceTypes [i], true);
                }
            }

            WaveVR_Utils.Event.Listen (WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
            WaveVR_Utils.Event.Listen ("render_model_loaded", OnRenderModelLoaded);
        }
    }

    void OnDisable()
    {
        if (ListenToDevice)
        {
            WaveVR_Utils.Event.Remove (WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
            WaveVR_Utils.Event.Remove ("render_model_loaded", OnRenderModelLoaded);
        }
    }
    #endregion
}
