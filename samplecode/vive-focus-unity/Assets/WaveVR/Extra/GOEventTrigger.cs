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
using UnityEngine.EventSystems;
using System.Collections;
using wvr;
using WaveVR_Log;
using System.Collections.Generic;

public class GOEventTrigger : MonoBehaviour
{
    private static string LOG_TAG = "WaveVR_GOEventTrigger";
    private Vector3 startPosition;
    private Color defaultColor = Color.gray;
    private Color changedColor = Color.red;

    private WaveVR_PermissionManager pmInstance = null;

    void Start ()
    {
        startPosition = transform.localPosition;

        #if UNITY_EDITOR
        if (Application.isEditor) return;
        #endif
        Log.d(LOG_TAG, "Start() get instance of WaveVR_PermissionManager");
        pmInstance = WaveVR_PermissionManager.instance;
    }

    // --------------- Event Handling begins --------------
    public void OnEnter()
    {
        Log.d (LOG_TAG, "OnEnter");
        ChangeColor (true);
    }

    public void OnTrigger()
    {
        Log.d (LOG_TAG, "OnTrigger");
        TeleportRandomly ();
    }

    public void OnExit()
    {
        Log.d (LOG_TAG, "OnExit");
        ChangeColor (false);
    }

    public void OnGazeReset ()
    {
        transform.localPosition = startPosition;
        ChangeColor (false);
    }

    public void OnShowButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnShowButton");
        #endif
        transform.gameObject.SetActive (true);
    }

    public void OnHideButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnHideButton");
        #endif
        transform.gameObject.SetActive (false);
    }

    public void OnBeamButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnBeamButton");
        #endif
        if (EventSystem.current != null && EventSystem.current.GetComponent<WaveVR_ControllerInputModule>() != null)
            EventSystem.current.GetComponent<WaveVR_ControllerInputModule>().RaycastMode = ERaycastMode.Beam;
    }

    public void OnFixedButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnFixedButton");
        #endif
        if (EventSystem.current != null && EventSystem.current.GetComponent<WaveVR_ControllerInputModule>() != null)
            EventSystem.current.GetComponent<WaveVR_ControllerInputModule>().RaycastMode = ERaycastMode.Fixed;
    }

    public void OnMouseButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnMouseButton");
        #endif
        if (EventSystem.current != null && EventSystem.current.GetComponent<WaveVR_ControllerInputModule>() != null)
            EventSystem.current.GetComponent<WaveVR_ControllerInputModule>().RaycastMode = ERaycastMode.Mouse;
    }

    public void OnSelectCtrlrButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnSelectCtrlrButton");
        #endif
        if (WaveVR_InputModuleManager.Instance != null && WaveVR_InputModuleManager.Instance.Controller != null)
        {
            WaveVR_InputModuleManager.Instance.Controller.EnableController = true;
        }
        if (WaveVR_InputModuleManager.Instance != null && WaveVR_InputModuleManager.Instance.Gaze != null)
        {
            WaveVR_InputModuleManager.Instance.Gaze.EnableGaze = false;
        }
    }

    public void OnSelectGazeButton()
    {
        #if UNITY_EDITOR
        Debug.Log ("OnSelectGazeButton");
        #endif
        if (WaveVR_InputModuleManager.Instance != null && WaveVR_InputModuleManager.Instance.Controller != null)
        {
            WaveVR_InputModuleManager.Instance.Controller.EnableController = false;
        }
        if (WaveVR_InputModuleManager.Instance != null && WaveVR_InputModuleManager.Instance.Gaze != null)
        {
            WaveVR_InputModuleManager.Instance.Gaze.EnableGaze = true;
        }
    }

    private const string CONTENT_PROVIDER_CLASSNAME = "com.htc.vr.unity.ContentProvider";
    private AndroidJavaObject contentProvider = null;
    /*
    public void OnChangeHand()
    {
        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            WaveVR_Controller.SetLeftHandedMode(WaveVR_Controller.IsLeftHanded ? false : true);
        } else
        #endif
        {
            if (pmInstance != null)
            {
                Log.d (LOG_TAG, "isPermissionGranted(com.htc.vr.core.server.VRDataWrite) = " + pmInstance.isPermissionGranted ("com.htc.vr.core.server.VRDataWrite"));
                Log.d (LOG_TAG, "isPermissionGranted(com.htc.vr.core.server.VRDataRead) = " + pmInstance.isPermissionGranted ("com.htc.vr.core.server.VRDataRead"));
                Log.d (LOG_TAG, "isPermissionGranted(com.htc.vr.core.server.VRDataProvider) = " + pmInstance.isPermissionGranted ("com.htc.vr.core.server.VRDataProvider"));
            }

            AndroidJavaClass ajc = new AndroidJavaClass(CONTENT_PROVIDER_CLASSNAME);
            if (ajc == null)
            {
                Log.e(LOG_TAG, "OnChangeHand() " + CONTENT_PROVIDER_CLASSNAME + " is null");
                return;
            }
            // Get the PermissionManager object
            contentProvider = ajc.CallStatic<AndroidJavaObject>("getInstance");
            if (contentProvider != null)
            {
                string _role = WaveVR_Controller.IsLeftHanded ? "2" : "1";
                Log.d (LOG_TAG, "OnChangeHand() got instance of " + CONTENT_PROVIDER_CLASSNAME + ", change role to " + _role);
                contentProvider.Call ("writeControllerRoleValue", _role);
            } else
            {
                Log.e (LOG_TAG, "OnChangeHand() could NOT get instance of " + CONTENT_PROVIDER_CLASSNAME);
            }
        }
    }
*/
    // --------------- Event Handling ends --------------

    public void ChangeColor(string color)
    {
        if (color.Equals("blue"))
            GetComponent<Renderer>().material.color = Color.blue;
        else if (color.Equals("cyan"))
            GetComponent<Renderer>().material.color = Color.cyan;
    }

    private void ChangeColor(bool change)
    {
        GetComponent<Renderer>().material.color = change ? changedColor : defaultColor;
    }

    private void TeleportRandomly () {
        Vector3 direction = UnityEngine.Random.onUnitSphere;
        direction.y = Mathf.Clamp (direction.y, 0.5f, 1f);
        direction.z = Mathf.Clamp (direction.z, 3f, 10f);
        float distance = 2 * UnityEngine.Random.value + 1.5f;
        transform.localPosition = direction * distance;
    }
}
