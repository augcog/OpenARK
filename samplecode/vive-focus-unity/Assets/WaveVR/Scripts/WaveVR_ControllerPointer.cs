// "WaveVR SDK
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

#pragma warning disable 0219
#pragma warning disable 0414

using UnityEngine;
using wvr;
using System;
using WaveVR_Log;

/// <summary>
/// Draws a pointer in front of any object that the controller point at.
/// The circle dilates if the object is clickable.
/// </summary>
[RequireComponent(typeof(Renderer))]
public class WaveVR_ControllerPointer : MonoBehaviour {
    private const string LOG_TAG = "WaveVR_ControllerPointer";

    public bool ListenToDevice = false;
    public WVR_DeviceType device;
    /// <summary>
    /// Growth speed multiplier for the pointer.
    /// </summary>
    private float pointerGrowthSpeed = 8.0f;
    /// <summary>
    /// Color of the pointer.
    /// </summary>
    // private Color pointerColor = Color.white;

    private Material materialComp;
    private Renderer rend;
    private float pointerDistanceInMeters = 10.0f;          // Current distance of the pointer (in meters).
    private const float kpointerDistanceMin = 0.2f;         // Minimum distance of the pointer (in meters).
    private const float kpointerDistanceMax = 10.0f;        // Maximum distance of the pointer (in meters).
    private float pointerOuterAngle = 1.2f;                 // Current outer angle of the pointer (in degrees).
    private const float kpointerMinOuterAngle = 1.5f;       // Minimum outer angle of the pointer (in degrees).
    private const float kpointerGrowthAngle = 90f;          // Angle at which to expand the pointer when intersecting with an object (in degrees).
    private float pointerOuterDiameter = 0.0f;              // Current outer diameters of the pointer, before distance multiplication.
    private float pointerOuterDiameterMin = 0.25f;          // Current outer diameters of the pointer, before distance multiplication.
    private bool enabledpointer = true;                     // true: show pointer, false: remove pointer
    private bool meshIsCreated = false;                     // true: the mesh of reticle is created, false: the mesh of reticle is not ready
    private bool stay = false;
    private ERaycastMode RaycastMode = ERaycastMode.Mouse;

    void Start () {
         if (enabledpointer) {
              if (!meshIsCreated) {
                   initialPointer();
              }
         } else {
              if (meshIsCreated) {
                   removePointer();
              }
         }
    }

    void Update() {
        if (ListenToDevice)
        {
            #if UNITY_EDITOR
            if (Application.isEditor)
                enabledpointer = WaveVR_Controller.Input (this.device).connected ? true : false;
            else
            #endif
            {
                WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.device);
                enabledpointer = _device.connected;
            }
        }

        if (enabledpointer) {
            if (!meshIsCreated) {
                initialPointer();
            }
        } else {
            if (meshIsCreated) {
                removePointer();
            }
            return;
        }
        if (RaycastMode == ERaycastMode.Fixed)
            pointerDistanceInMeters = kpointerDistanceMax;
        else if (RaycastMode == ERaycastMode.Mouse)
           pointerDistanceInMeters = kpointerDistanceMin;
        else
            pointerDistanceInMeters = Mathf.Clamp(pointerDistanceInMeters, kpointerDistanceMin, kpointerDistanceMax);

        if (pointerOuterAngle < kpointerMinOuterAngle)
            pointerOuterAngle = kpointerMinOuterAngle;
        float outerHalfAngelRadians = Mathf.Deg2Rad * pointerOuterAngle * 0.5f;
        float outerDiameter = 2.0f * Mathf.Tan(outerHalfAngelRadians);

        if (RaycastMode == ERaycastMode.Fixed)
            pointerOuterDiameter = 0.2f;
        else if (RaycastMode == ERaycastMode.Mouse)
            pointerOuterDiameter = pointerOuterDiameterMin;
        else
            pointerOuterDiameter = Mathf.Lerp(pointerOuterDiameter, outerDiameter, Time.deltaTime * pointerGrowthSpeed);

        if (RaycastMode == ERaycastMode.Fixed)
        {
            materialComp.renderQueue = 1000;
        }
        else
        {
            materialComp.renderQueue = 5000;
        }

        materialComp.SetFloat("_OuterDiameter", (RaycastMode != ERaycastMode.Fixed)? 0.03f + (pointerDistanceInMeters/kpointerGrowthAngle) : pointerOuterDiameter * pointerDistanceInMeters);
        materialComp.SetFloat("_DistanceInMeters", pointerDistanceInMeters);
    }

    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " " + msg);
        #endif
        Log.d (LOG_TAG, msg);
    }

    private void initialPointer() {
        rend = GetComponent<Renderer>();
        rend.enabled = true;
        materialComp = gameObject.GetComponent<Renderer>().material;
        meshIsCreated = true;
    }

    public void showPointer() {
        initialPointer();
    }

    public void removePointer() {
        rend = GetComponent<Renderer>();
        rend.enabled = false;
        meshIsCreated = false;
    }

    public void OnPointerEnter (Camera camera, GameObject target, Vector3 intersectionPosition, bool isInteractive) {
        SetPointerTarget(intersectionPosition, isInteractive);
    }

    public void OnPointerStay (Camera camera, GameObject target, Vector3 intersectionPosition, bool isInteractive) {
        SetPointerTarget(intersectionPosition, isInteractive);
    }

    public void OnPointerExit (Camera camera, GameObject target) {
        stay = false;
        pointerDistanceInMeters = kpointerDistanceMax;
        pointerOuterAngle = kpointerMinOuterAngle;
    }

    public void setRaycastMode(ERaycastMode mode) {
        RaycastMode = mode;
    }

    public float getPointerCurrentDistance() {
         return pointerDistanceInMeters;
    }

    private void SetPointerTarget (Vector3 target, bool interactive) {
        Vector3 targetLocalPosition = transform.InverseTransformPoint(target);
        pointerDistanceInMeters = Mathf.Clamp(targetLocalPosition.z, kpointerDistanceMin, kpointerDistanceMax);
        if (interactive) {
            pointerOuterAngle = kpointerMinOuterAngle + Mathf.Clamp(Mathf.Abs(targetLocalPosition.z) / kpointerDistanceMax, 0.1f, kpointerDistanceMax) * kpointerGrowthAngle;
        } else {
            pointerOuterAngle = kpointerMinOuterAngle;
        }
    }

    public void SetPointerColor(Color pointer_color)
    {
        // pointerColor = pointer_color;
    }
}
