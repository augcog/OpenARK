using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using wvr;
using WaveVR_Log;
using UnityEngine.SceneManagement;

#if UNITY_EDITOR
using UnityEditor;

[CustomEditor(typeof(WaveVR_InputModuleManager))]
public class WaveVR_InputModuleManagerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        EditorGUILayout.PropertyField(serializedObject.FindProperty("Gaze"), true);
        serializedObject.ApplyModifiedProperties();
        WaveVR_InputModuleManager myScript = target as WaveVR_InputModuleManager;

        if (myScript != null && myScript.Gaze != null)
        {
            if (myScript.Gaze.EnableGaze)
            {
                myScript.Gaze.BtnControl = EditorGUILayout.Toggle ("    BtnControl", myScript.Gaze.BtnControl);
                if (myScript.Gaze.BtnControl)
                {
                    myScript.Gaze.GazeDevice = (EGazeTriggerDevice) EditorGUILayout.EnumPopup ("        Gaze Trigger Device", myScript.Gaze.GazeDevice);
                    myScript.Gaze.ButtonToTrigger = (EGazeTriggerButton) EditorGUILayout.EnumPopup ("        Button To Trigger", myScript.Gaze.ButtonToTrigger);
                    myScript.Gaze.WithTimeGaze = EditorGUILayout.Toggle ("        With Time Gaze", myScript.Gaze.WithTimeGaze);
                }
            }
            if (EventSystem.current != null)
            {
                GazeInputModule gim = EventSystem.current.GetComponent<GazeInputModule> ();
                if (gim != null)
                {
                    gim.BtnControl = myScript.Gaze.BtnControl;
                    gim.GazeDevice = myScript.Gaze.GazeDevice;
                    gim.ButtonToTrigger = myScript.Gaze.ButtonToTrigger;
                    gim.WithTimeGaze = myScript.Gaze.WithTimeGaze;
                }
            }
        }
        serializedObject.Update();
        EditorGUILayout.PropertyField(serializedObject.FindProperty("Controller"), true);
        serializedObject.ApplyModifiedProperties();

        if (GUI.changed)
            EditorUtility.SetDirty ((WaveVR_InputModuleManager)target);
    }
}
#endif

public class WaveVR_InputModuleManager : MonoBehaviour
{
    private const string LOG_TAG = "WaveVR_InputModuleManager";

    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " " + msg);
        #endif
        Log.d (LOG_TAG, msg);
    }

    #region Gaze parameters
    [System.Serializable]
    public class CGazeInputModule
    {
        public bool EnableGaze = false;
        public bool progressRate = true;  // The switch to show how many percent to click by TimeToGaze
        public float RateTextZPosition = 0.5f;
        public bool progressCounter = true;  // The switch to show how long to click by TimeToGaze
        public float CounterTextZPosition = 0.5f;
        public float TimeToGaze = 2.0f;
        public EGazeInputEvent InputEvent = EGazeInputEvent.PointerSubmit;
        public GameObject Head = null;
        [HideInInspector]
        public bool BtnControl = false;
        [HideInInspector]
        public EGazeTriggerDevice GazeDevice = EGazeTriggerDevice.HMD;
        [HideInInspector]
        public EGazeTriggerButton ButtonToTrigger = EGazeTriggerButton.Trigger;
        [HideInInspector]
        public bool WithTimeGaze = false;
    }

    public CGazeInputModule Gaze;
    #endregion

    #region Controller Input Module parameters
    [System.Serializable]
    public class CControllerInputModule
    {
        public bool EnableController = true;
        public GameObject RightController;
        public LayerMask RightRaycastMask = ~0;
        public GameObject LeftController;
        public LayerMask LeftRaycastMask = ~0;
        public EControllerButtons ButtonToTrigger = EControllerButtons.Touchpad;
        public ERaycastMode RaycastMode = ERaycastMode.Mouse;
        public ERaycastStartPoint RaycastStartPoint = ERaycastStartPoint.CenterOfEyes;
        [Tooltip("Will be obsoleted soon!")]
        public string CanvasTag = "EventCanvas";
    }

    public CControllerInputModule Controller;
    #endregion

    private static WaveVR_InputModuleManager instance = null;
    public static WaveVR_InputModuleManager Instance {
        get
        {
            return instance;
        }
    }

    private GameObject Head = null;
    private GameObject eventSystem = null;
    private GazeInputModule gazeInputModule = null;
    private WaveVR_Reticle gazePointer = null;
    private WaveVR_ControllerInputModule controllerInputModule = null;

    void Awake()
    {
        if (instance == null)
            instance = this;
    }

    void Start()
    {
        if (EventSystem.current == null)
        {
            EventSystem _es = FindObjectOfType<EventSystem> ();
            if (_es != null)
            {
                eventSystem = _es.gameObject;
                PrintDebugLog ("Start() find current EventSystem: " + eventSystem.name);
            }

            if (eventSystem == null)
            {
                PrintDebugLog ("Start() could not find EventSystem, create new one.");
                eventSystem = new GameObject ("EventSystem", typeof(EventSystem));
                eventSystem.AddComponent<GazeInputModule> ();
            }
        } else
        {
            eventSystem = EventSystem.current.gameObject;
        }

        // Standalone Input Module
        StandaloneInputModule _sim = eventSystem.GetComponent<StandaloneInputModule> ();
        if (_sim != null)
            _sim.enabled = false;

        // Gaze Input Module
        gazeInputModule = eventSystem.GetComponent<GazeInputModule> ();
        if (Gaze.EnableGaze)
        {
            if (gazeInputModule == null)
                CreateGazeInputModule ();
            else
                SetGazeInputModuleParameters ();
        } else
        {
            // Deactivate gaze pointer to prevent showing pointer in scene.
            ActivateGazePointer (false);
        }

        // Controller Input Module
        controllerInputModule = eventSystem.GetComponent<WaveVR_ControllerInputModule> ();
        if (Controller.EnableController)
        {
            if (controllerInputModule == null)
                CreateControllerInputModule ();
            else
                SetControllerInputModuleParameters ();
        }
    }

    private void ActivateGazePointer(bool active)
    {
        if (gazePointer == null)
            gazePointer = Gaze.Head.GetComponentInChildren<WaveVR_Reticle> ();
        if (gazePointer != null)
            gazePointer.gameObject.SetActive (active);
    }

    private void CreateGazeInputModule()
    {
        if (gazeInputModule == null)
        {
            // Before initializing variables of input modules, disable EventSystem to prevent the OnEnable() of input modules being executed.
            eventSystem.SetActive (false);

            gazeInputModule = eventSystem.AddComponent<GazeInputModule> ();
            SetGazeInputModuleParameters ();

            // Enable EventSystem after initializing input modules.
            eventSystem.SetActive (true);
        }
    }

    private void SetGazeInputModuleParameters()
    {
        if (gazeInputModule != null)
        {
            ActivateGazePointer (true);

            gazeInputModule.enabled = false;
            gazeInputModule.progressRate = Gaze.progressRate;
            gazeInputModule.RateTextZPosition = Gaze.RateTextZPosition;
            gazeInputModule.progressCounter = Gaze.progressCounter;
            gazeInputModule.CounterTextZPosition = Gaze.CounterTextZPosition;
            gazeInputModule.TimeToGaze = Gaze.TimeToGaze;
            gazeInputModule.InputEvent = Gaze.InputEvent;
            gazeInputModule.Head = Gaze.Head;
            gazeInputModule.BtnControl = Gaze.BtnControl;
            gazeInputModule.GazeDevice = Gaze.GazeDevice;
            gazeInputModule.ButtonToTrigger = Gaze.ButtonToTrigger;
            gazeInputModule.WithTimeGaze = Gaze.WithTimeGaze;
            gazeInputModule.enabled = true;
        }
    }

    private void CreateControllerInputModule()
    {
        if (controllerInputModule == null)
        {
            // Before initializing variables of input modules, disable EventSystem to prevent the OnEnable() of input modules being executed.
            eventSystem.SetActive (false);

            controllerInputModule = eventSystem.AddComponent<WaveVR_ControllerInputModule> ();
            SetControllerInputModuleParameters ();

            // Enable EventSystem after initializing input modules.
            eventSystem.SetActive (true);
        }
    }

    private void SetControllerInputModuleParameters()
    {
        if (controllerInputModule != null)
        {
            controllerInputModule.enabled = false;
            controllerInputModule.RightController = Controller.RightController;
            controllerInputModule.RightRaycastMask = Controller.RightRaycastMask;
            controllerInputModule.LeftController = Controller.LeftController;
            controllerInputModule.LeftRaycastMask = Controller.LeftRaycastMask;
            controllerInputModule.ButtonToTrigger = Controller.ButtonToTrigger;
            controllerInputModule.RaycastMode = Controller.RaycastMode;
            controllerInputModule.RaycastStartPoint = Controller.RaycastStartPoint;
            controllerInputModule.CanvasTag = Controller.CanvasTag;
            controllerInputModule.enabled = true;
        }
    }

    private void SetActiveGaze(bool value)
    {
        if (gazeInputModule != null)
            gazeInputModule.enabled = value;
        else
        {
            if (value)
                CreateGazeInputModule ();
        }
    }

    private void SetActiveController(bool value)
    {
        if (controllerInputModule != null)
            controllerInputModule.enabled = value;
        else
        {
            if (value)
                CreateControllerInputModule ();
        }
    }

    private bool IsAnyControllerConnected()
    {
        bool _result = false;

        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            if (_dt == WVR_DeviceType.WVR_DeviceType_HMD)
                continue;

            if (WaveVR_Controller.Input (_dt).connected)
            {
                _result = true;
                break;
            }
        }

        return _result;
    }

    private void renderControllerPointer(WVR_DeviceType type, bool result)
    {
        if (WaveVR_EventSystemControllerProvider.Instance != null)
        {
            GameObject ctrlr = WaveVR_EventSystemControllerProvider.Instance.GetControllerModel (type);
            if (ctrlr != null)
            {
                WaveVR_ControllerPointer cp = ctrlr.GetComponentInChildren<WaveVR_ControllerPointer>();
                MeshRenderer cpmr = null;

                if (cp != null)
                    cpmr = cp.gameObject.GetComponentInChildren<MeshRenderer>();
                if (cpmr != null)
                    cpmr.enabled = result;
            }
        }
    }

    private void renderControllerBeam(WVR_DeviceType type, bool result)
    {
        if (WaveVR_EventSystemControllerProvider.Instance != null)
        {
            GameObject ctrlr = WaveVR_EventSystemControllerProvider.Instance.GetControllerModel (type);
            if (ctrlr != null)
            {
                WaveVR_Beam beam = ctrlr.GetComponentInChildren<WaveVR_Beam>();
                MeshRenderer bmr = null;

                if (beam != null)
                    bmr = beam.gameObject.GetComponentInChildren<MeshRenderer>();
                if (bmr != null)
                    bmr.enabled = result;
            }
        }
    }

    public void Update()
    {
        if (WaveVR_Render.Instance != null)
            Head = WaveVR_Render.Instance.gameObject;

        if (Head != null)
        {
            gameObject.transform.localPosition = Head.transform.localPosition;
            gameObject.transform.localRotation = Head.transform.localRotation;
        }

        if (Gaze.EnableGaze && Controller.EnableController)
        {
            if (IsAnyControllerConnected ())
            {
                // One or more controller connected, using controller input module, disable gaze input module.
                SetActiveGaze (false);
                SetActiveController (true);
                renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Left, true);
                renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Right, true);
                renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Left, true);
                renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Right, true);
            } else
            {
                // No controller connected, using gaze input module.
                SetActiveGaze (true);
                SetActiveController (false);
                renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Left, false);
                renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Right, false);
                renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Left, false);
                renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Right, false);
            }
        } else if (Gaze.EnableGaze)
        {
            // Only using gaze input module
            SetActiveGaze (true);
            SetActiveController (false);
            renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Left, false);
            renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Right, false);
            renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Left, false);
            renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Right, false);
        } else if (Controller.EnableController)
        {
            // Only using controller input module
            SetActiveGaze (false);
            SetActiveController (true);
            renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Left, true);
            renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Right, true);
            renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Left, true);
            renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Right, true);
        } else
        {
            SetActiveGaze (false);
            SetActiveController (false);
            renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Left, false);
            renderControllerPointer(WVR_DeviceType.WVR_DeviceType_Controller_Right, false);
            renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Left, false);
            renderControllerBeam(WVR_DeviceType.WVR_DeviceType_Controller_Right, false);
        }
    }

    public ERaycastMode GetRaycastMode()
    {
         if (Controller != null)
             return Controller.RaycastMode;
         else if (controllerInputModule != null)
             return controllerInputModule.RaycastMode;
         else
             return ERaycastMode.Beam;
    }
}
