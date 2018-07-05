using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WaveVR_Log;
using wvr;

public class WaveVR_PointerCameraTracker : MonoBehaviour
{
    private const string LOG_TAG = "WaveVR_PointerCameraTracker";

    public GameObject reticleObject = null;
    public WVR_DeviceType type;
    private WaveVR_ControllerPointer reticle;
    private Vector3 pointer_location;

    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
            Debug.Log(LOG_TAG + " " + msg);
        #endif
        Log.d (LOG_TAG, msg);
    }

    // Use this for initialization
    void Start()
    {
        if (reticleObject != null)
		{
            reticle = reticleObject.GetComponent<WaveVR_ControllerPointer>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (reticleObject != null && reticle == null) {
            reticle = reticleObject.GetComponent<WaveVR_ControllerPointer>();
        }

        if (reticleObject != null && reticle != null)
        {
            pointer_location = reticleObject.transform.position + reticleObject.transform.forward * reticle.getPointerCurrentDistance();
            transform.rotation = Quaternion.LookRotation(pointer_location - transform.position);
        }
    }

    public void setDeviceType(WVR_DeviceType value)
    {
        type =  value;
    }
}

