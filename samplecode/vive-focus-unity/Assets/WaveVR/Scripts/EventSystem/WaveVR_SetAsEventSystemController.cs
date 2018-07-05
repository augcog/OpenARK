using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;

public class WaveVR_SetAsEventSystemController : MonoBehaviour
{
    private bool added = false;
    private WVR_DeviceType Type;

    void OnEnable()
    {
        WaveVR_PoseTrackerManager _ptm = (WaveVR_PoseTrackerManager)gameObject.GetComponent (typeof(WaveVR_PoseTrackerManager));
        if (_ptm != null)
        {
            Type = _ptm.Type;
            WaveVR_EventSystemControllerProvider.Instance.SetControllerModel (Type, gameObject);
            added = true;
        }
    }

    void OnDisable()
    {
        if (added)
        {
            WaveVR_EventSystemControllerProvider.Instance.SetControllerModel (Type, null);
            added = false;
        }
    }
}
