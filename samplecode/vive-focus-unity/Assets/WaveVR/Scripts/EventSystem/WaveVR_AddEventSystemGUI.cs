using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaveVR_AddEventSystemGUI : MonoBehaviour
{
    private bool added = false;
    void OnEnable()
    {
        Canvas _canvas = (Canvas)gameObject.GetComponent (typeof(Canvas));
        if (_canvas != null)
        {
            WaveVR_EventSystemGUIProvider.AddEventGUI (gameObject);
            added = true;
        }
    }

    void OnDisable()
    {
        if (added)
        {
            WaveVR_EventSystemGUIProvider.RemoveEventGUI (gameObject);
            added = false;
        }
    }
}
