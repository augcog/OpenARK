using System.Collections;
using System.Collections.Generic;
using UnityEngine;

static class WaveVR_EventSystemGUIProvider
{
    static List<GameObject> EventGUIs = new List<GameObject> ();

    public static void AddEventGUI(GameObject go)
    {
        EventGUIs.Add (go);
    }

    public static void RemoveEventGUI(GameObject go)
    {
        EventGUIs.Remove (go);
    }

    public static GameObject[] GetEventGUIs()
    {
        if (EventGUIs.Count == 0)
            return null;

        return EventGUIs.ToArray ();
    }
}
