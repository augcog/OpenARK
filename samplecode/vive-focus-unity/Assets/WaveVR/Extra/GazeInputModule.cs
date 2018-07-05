#pragma warning disable 0414 // private field assigned but not used.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using WaveVR_Log;
using UnityEngine.UI;
using wvr;

#if UNITY_EDITOR
using UnityEditor;

[CustomEditor(typeof(GazeInputModule))]
public class GazeInputModuleEditor : Editor
{
    public override void OnInspectorGUI()
    {
        GazeInputModule myScript = target as GazeInputModule;
        if (myScript != null)
        {
            myScript.progressRate = EditorGUILayout.Toggle ("Progress Rate", myScript.progressRate);
            myScript.RateTextZPosition = EditorGUILayout.FloatField("Rate Text Z Position", myScript.RateTextZPosition);
            myScript.progressCounter = EditorGUILayout.Toggle ("Progress Counter", myScript.progressCounter);
            myScript.CounterTextZPosition = EditorGUILayout.FloatField("Counter Text Z Position", myScript.CounterTextZPosition);
            myScript.TimeToGaze = EditorGUILayout.FloatField("Time To Gaze", myScript.TimeToGaze);
            myScript.InputEvent = (EGazeInputEvent) EditorGUILayout.EnumPopup ("Input Event", myScript.InputEvent);
            myScript.Head = (GameObject) EditorGUILayout.ObjectField("Head", myScript.Head, typeof(GameObject), true);
            if (myScript.enabled)
            {
                myScript.BtnControl = EditorGUILayout.Toggle ("BtnControl", myScript.BtnControl);
                if (myScript.BtnControl)
                {
                    myScript.GazeDevice = (EGazeTriggerDevice) EditorGUILayout.EnumPopup ("        Gaze Trigger Device", myScript.GazeDevice);
                    myScript.ButtonToTrigger = (EGazeTriggerButton) EditorGUILayout.EnumPopup ("        Button To Trigger", myScript.ButtonToTrigger);
                    myScript.WithTimeGaze = EditorGUILayout.Toggle ("        With Time Gaze", myScript.WithTimeGaze);
                }
            }
        }

        if (GUI.changed)
            EditorUtility.SetDirty ((GazeInputModule)target);
    }
}
#endif

public enum EGazeTriggerMouseKey
{
    LeftClick,
    RightClick,
    MiddleClick
}

public enum EGazeTriggerButton
{
    System = WVR_InputId.WVR_InputId_Alias1_System,
    Menu = WVR_InputId.WVR_InputId_Alias1_Menu,
    Grip = WVR_InputId.WVR_InputId_Alias1_Grip,
    DPad_Left = WVR_InputId.WVR_InputId_Alias1_DPad_Left,
    DPad_Up = WVR_InputId.WVR_InputId_Alias1_DPad_Up,
    DPad_Right = WVR_InputId.WVR_InputId_Alias1_DPad_Right,
    DPad_Down = WVR_InputId.WVR_InputId_Alias1_DPad_Down,
    Volume_Up = WVR_InputId.WVR_InputId_Alias1_Volume_Up,
    Volume_Down = WVR_InputId.WVR_InputId_Alias1_Volume_Down,
    Bumper = WVR_InputId.WVR_InputId_Alias1_Bumper,
    Touchpad = WVR_InputId.WVR_InputId_Alias1_Touchpad,
    Trigger = WVR_InputId.WVR_InputId_Alias1_Trigger
}

public enum EGazeTriggerDevice
{
    HMD,
    LeftController,
    RightController,
    HMDWithLeftController,
    HMDWithRightController,
    HMDWithTwoControllers
}

public enum EGazeInputEvent
{
    PointerDown,
    PointerClick,
    PointerSubmit
}

public class GazeInputModule : PointerInputModule
{
    private static string LOG_TAG = "GazeInputModule";
    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " " + msg);
        #endif
        Log.d (LOG_TAG, msg);
    }

    public bool progressRate = true;  // The switch to show how many percent to click by TimeToGaze
    public float RateTextZPosition = 0.5f;
    public bool progressCounter = true;  // The switch to show how long to click by TimeToGaze
    public float CounterTextZPosition = 0.5f;
    public float TimeToGaze = 2.0f;
    public EGazeInputEvent InputEvent = EGazeInputEvent.PointerSubmit;
    public GameObject Head = null;
    public bool BtnControl = false;
    [HideInInspector]
    public EGazeTriggerDevice GazeDevice = EGazeTriggerDevice.HMD;
    [HideInInspector]
    public EGazeTriggerButton ButtonToTrigger = EGazeTriggerButton.Trigger;
    [HideInInspector]
    public bool WithTimeGaze = false;
    private bool btnPressDown = false;
    private bool btnPressed = false;
    private bool btnPressUp = false;
    /**
    * @brief get intersection position in world space
    **/
    private Vector3 GetIntersectionPosition(Camera cam, RaycastResult raycastResult)
    {
        // Check for camera
        if (cam == null) {
            return Vector3.zero;
        }

        float intersectionDistance = raycastResult.distance + cam.nearClipPlane;
        Vector3 intersectionPosition = cam.transform.position + cam.transform.forward * intersectionDistance;
        return intersectionPosition;
    }

    private PointerEventData pointerData;

    private void CastToCenterOfScreen()
    {
        if (pointerData == null)
            pointerData = new PointerEventData (eventSystem);

        pointerData.Reset();
        pointerData.position = new Vector2 (0.5f * Screen.width, 0.5f * Screen.height);  // center of screen

        if (Head != null)
        {
            Camera _event_camera = Head.GetComponent<Camera> ();
            GraphicRaycast (_event_camera);

            if (pointerData.pointerCurrentRaycast.gameObject == null)
            {
                PhysicsRaycaster _raycaster = Head.GetComponent<PhysicsRaycaster> ();
                PhysicRaycast (_raycaster);
            }
        }
    }

    private void GraphicRaycast(Camera event_camera)
    {
        List<RaycastResult> _raycast_results = new List<RaycastResult>();

        // Reset pointerCurrentRaycast even no GUI.
        RaycastResult _firstResult = new RaycastResult ();
        pointerData.pointerCurrentRaycast = _firstResult;

        foreach (Canvas _canvas in sceneCanvases)
        {
            GraphicRaycaster _gr = _canvas.GetComponent<GraphicRaycaster> ();
            if (_gr == null)
                continue;

            // 1. Change event camera.
            _canvas.worldCamera = event_camera;

            // 2.
            _gr.Raycast (pointerData, _raycast_results);

            _firstResult = FindFirstRaycast (_raycast_results);
            pointerData.pointerCurrentRaycast = _firstResult;
            _raycast_results.Clear ();

            #if UNITY_EDITOR
            if (_firstResult.module != null)
            {
                //Debug.Log ("GraphicRaycast() device: " + event_controller.device + ", camera: " + _firstResult.module.eventCamera + ", first result = " + _firstResult);
            }
            #endif

            // Found graphic raycasted object!
            if (_firstResult.gameObject != null)
            {
                if (_firstResult.worldPosition == Vector3.zero)
                {
                    _firstResult.worldPosition = GetIntersectionPosition (
                        _firstResult.module.eventCamera,
                        //_eventController.event_data.enterEventCamera,
                        _firstResult
                    );
                    pointerData.pointerCurrentRaycast = _firstResult;
                }

                pointerData.position = _firstResult.screenPosition;
                break;
            }
        }
    }

    private void PhysicRaycast(PhysicsRaycaster raycaster)
    {
        if (raycaster == null)
            return;

        List<RaycastResult> _raycast_results = new List<RaycastResult>();
        raycaster.Raycast (pointerData, _raycast_results);

        RaycastResult _firstResult = FindFirstRaycast (_raycast_results);
        pointerData.pointerCurrentRaycast = _firstResult;

        #if UNITY_EDITOR
        //PrintDebugLog ("PhysicRaycast() first result = " + _firstResult);
        #endif

        if (_firstResult.gameObject != null)
        {
            if (_firstResult.worldPosition == Vector3.zero)
            {
                _firstResult.worldPosition = GetIntersectionPosition (
                    _firstResult.module.eventCamera,
                    //_eventController.event_data.enterEventCamera,
                    _firstResult
                );
                pointerData.pointerCurrentRaycast = _firstResult;
            }

            pointerData.position = _firstResult.screenPosition;
        }
    }

    private float gazeTime = 0.0f;
    // { ------- Reticle --------
    private Text progressText = null;
    private Text counterText = null;
    private WaveVR_Reticle gazePointer = null;
    private GameObject percentCanvas = null, counterCanvas = null;
    private bool progressflag = true;
    private float countingTime = 0f;

    private GameObject GetCurrentGameObject(PointerEventData pointerData) {
        if (pointerData != null && pointerData.enterEventCamera != null)
            return pointerData.pointerCurrentRaycast.gameObject;

        return null;
    }

    private Vector3 GetIntersectionPosition(PointerEventData pointerData) {
        if (null == pointerData.enterEventCamera)
            return Vector3.zero;

        float intersectionDistance = pointerData.pointerCurrentRaycast.distance + pointerData.enterEventCamera.nearClipPlane;
        Vector3 intersectionPosition = pointerData.enterEventCamera.transform.position + pointerData.enterEventCamera.transform.forward * intersectionDistance;
        return intersectionPosition;
    }

    private void UpdateProgressDistance(PointerEventData pointerEvent) {
        Vector3 intersectionPosition = GetIntersectionPosition(pointerEvent);
        if (gazePointer == null)
            return;

        if (percentCanvas != null) {
            Vector3 tmpVec = new Vector3(percentCanvas.transform.localPosition.x, percentCanvas.transform.localPosition.y, intersectionPosition.z - (RateTextZPosition >= 0 ? RateTextZPosition : 0));
            percentCanvas.transform.localPosition = tmpVec;
        }

        if (counterCanvas != null) {
            Vector3 tmpVec = new Vector3(counterCanvas.transform.localPosition.x, counterCanvas.transform.localPosition.y, intersectionPosition.z - (CounterTextZPosition >= 0 ? CounterTextZPosition : 0));
            counterCanvas.transform.localPosition = tmpVec;
        }
    }

    private void UpdateReticle (GameObject preGazedObject, PointerEventData pointerEvent) {
        if (gazePointer == null)
            return;

        GameObject curGazeObject = GetCurrentGameObject(pointerEvent);
        Vector3 intersectionPosition = GetIntersectionPosition(pointerEvent);
        bool isInteractive = pointerEvent.pointerPress != null || ExecuteEvents.GetEventHandler<IPointerClickHandler>(curGazeObject) != null;

        if (curGazeObject == preGazedObject) {
            if (curGazeObject != null) {
                gazePointer.OnGazeStay(pointerEvent.enterEventCamera, curGazeObject, intersectionPosition, isInteractive);
            } else {
                gazePointer.OnGazeExit(pointerEvent.enterEventCamera, preGazedObject);
                return;
            }
        } else {
            if (preGazedObject != null) {
                gazePointer.OnGazeExit(pointerEvent.enterEventCamera, preGazedObject);
            }
            if (curGazeObject != null) {
                gazePointer.OnGazeEnter(pointerEvent.enterEventCamera, curGazeObject, intersectionPosition, isInteractive);
            }
        }
        UpdateProgressDistance(pointerEvent);
    }
    // --------- Reticle -------- }

    private void UpdateBtnState()
    {
        btnPressDown = Input.GetMouseButtonDown ((int) EGazeTriggerMouseKey.LeftClick);
        btnPressed = Input.GetMouseButton ((int) EGazeTriggerMouseKey.LeftClick);
        btnPressUp = Input.GetMouseButtonUp ((int) EGazeTriggerMouseKey.LeftClick);
        if (GazeDevice == EGazeTriggerDevice.HMD ||
            GazeDevice == EGazeTriggerDevice.HMDWithLeftController ||
            GazeDevice == EGazeTriggerDevice.HMDWithRightController ||
            GazeDevice == EGazeTriggerDevice.HMDWithTwoControllers)
        {
                btnPressDown |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_HMD).GetPressDown ((WVR_InputId)ButtonToTrigger);
                btnPressed |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_HMD).GetPress ((WVR_InputId)ButtonToTrigger);
                btnPressUp |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_HMD).GetPressUp ((WVR_InputId)ButtonToTrigger);
        }

        if (GazeDevice == EGazeTriggerDevice.LeftController ||
            GazeDevice == EGazeTriggerDevice.HMDWithLeftController ||
            GazeDevice == EGazeTriggerDevice.HMDWithTwoControllers)
        {
                btnPressDown |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).GetPressDown ((WVR_InputId)ButtonToTrigger);
                btnPressed |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).GetPress ((WVR_InputId)ButtonToTrigger);
                btnPressUp |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).GetPressUp ((WVR_InputId)ButtonToTrigger);
        }

        if (GazeDevice == EGazeTriggerDevice.RightController ||
            GazeDevice == EGazeTriggerDevice.HMDWithRightController ||
            GazeDevice == EGazeTriggerDevice.HMDWithTwoControllers)
        {
                btnPressDown |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).GetPressDown ((WVR_InputId)ButtonToTrigger);
                btnPressed |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).GetPress ((WVR_InputId)ButtonToTrigger);
                btnPressUp |= WaveVR_Controller.Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).GetPressUp ((WVR_InputId)ButtonToTrigger);
        }
    }

    private void OnTriggeGaze()
    {
        bool sendEvent = false;
        // The gameobject to which raycast positions
        var currentOverGO = pointerData.pointerCurrentRaycast.gameObject;

        // { ------- Reticle --------
        if (progressText == null) {
            GameObject pt = GameObject.Find("ProgressText");
            if (pt != null) {
                progressText = pt.GetComponent<Text>();
            }
        }
        if (counterText == null) {
            GameObject ct = GameObject.Find("CounterText");
            if (ct != null) {
                counterText = ct.GetComponent<Text>();
            }
        }

        if (pointerData.pointerEnter == null && currentOverGO == null) {
            UpdateReticle(currentOverGO, pointerData);
            progressflag = true;
            if (progressText != null) {
                progressText.text = "";
            }
            if (counterText != null) {
                counterText.text = "";
            }
            if (gazePointer != null) {
                gazePointer.triggerProgressBar(false);
            }
            return;
        }

        if (!progressRate && !progressCounter) {  //  if no counting, reset trigger flag
            progressflag = true;
        }

        if (!progressRate || !progressCounter) {  //  clear counting content
            if (progressText != null && progressText.text != "") {
                progressText.text = "";
            }
            if (counterText != null && counterText.text != "") {
                counterText.text = "";
            }
        }
        // --------- Reticle -------- }

        if (pointerData.pointerEnter != currentOverGO)
        {
            #if UNITY_EDITOR
            PrintDebugLog ("OnTriggeGaze() pointerEnter: " + pointerData.pointerEnter + ", currentOverGO: " + currentOverGO);
            #endif
            HandlePointerExitAndEnter (pointerData, currentOverGO);

            gazeTime = Time.unscaledTime;

            // { ------- Reticle --------
            countingTime = Time.unscaledTime;
            UpdateReticle(currentOverGO, pointerData);
            // --------- Reticle -------- }
        }
        else
        {
            // { ------- Reticle --------
            if (progressflag) {   // begin to count, do initialization
                if (gazePointer != null) {
                    gazePointer.triggerProgressBar(true);
                }
                if (progressRate && progressText != null) {
                    progressText.text = "0%";
                }
                if (progressCounter && counterText != null) {
                    counterText.text = TimeToGaze.ToString();
                }
                countingTime = Time.unscaledTime;
                progressflag = false;  // counting the rate of waiting for clicking event
            }
            // --------- Reticle -------- }

            float elapsedTime = Time.unscaledTime;
            if (BtnControl && !WithTimeGaze)
            {
                UpdateBtnState();

                if (btnPressDown)
                {
                    sendEvent = true;
                }
                if (progressRate) {
                    if (progressText != null) {
                        progressText.text = "";
                    }
                }
                if (progressCounter) {
                    if (counterText != null) {
                        counterText.text = "";
                    }
                }
                if (gazePointer != null) {
                    gazePointer.triggerProgressBar(false);
                }
                progressflag = false;
            } else {
                if (BtnControl && WithTimeGaze)
                {
                    UpdateBtnState();

                    if (btnPressDown)
                    {
                        gazeTime = Time.unscaledTime - TimeToGaze;
                    }
                }
                if (elapsedTime - gazeTime > TimeToGaze)
                {
                    #if UNITY_EDITOR
                    //PrintDebugLog ("OnTriggeGaze() Selected: {" + currentOverGO.name + "} over " + TimeToGaze + " seconds.");
                    #endif
                    sendEvent = true;
                    gazeTime = Time.unscaledTime;

                    // { ------- Reticle --------
                    if (progressRate) {
                        if (progressText != null) {
                            progressText.text = "";
                        }
                    }
                    if (progressCounter) {
                        if (counterText != null) {
                            counterText.text = "";
                        }
                    }
                    if (gazePointer != null) {
                        gazePointer.triggerProgressBar(false);
                    }
                    progressflag = true;   // reset trigger flag after each counting is done
                } else {
                    float rate = ((Time.unscaledTime - gazeTime) / TimeToGaze) * 100;
                    if (gazePointer != null) {
                        gazePointer.setProgressBarTime(rate);
                    }
                    if (progressRate) {
                        if (progressText != null) {
                            progressText.text = Mathf.Floor(rate) + "%";
                        }
                    }
                    if (progressCounter) {
                        if (counterText != null) {
                            counterText.text = System.Math.Round(TimeToGaze - (Time.unscaledTime - countingTime), 2).ToString();
                        }
                    }
                    // --------- Reticle -------- }
                }
            }
        }

        // Standalone Input Module information
        pointerData.delta = Vector2.zero;
        pointerData.dragging = false;

        DeselectIfSelectionChanged (currentOverGO, pointerData);

        if (sendEvent)
        {
            if (InputEvent == EGazeInputEvent.PointerClick)
            {
                ExecuteEvents.ExecuteHierarchy (currentOverGO, pointerData, ExecuteEvents.pointerClickHandler);
                pointerData.clickTime = Time.unscaledTime;
            } else if (InputEvent == EGazeInputEvent.PointerDown)
            {
                // like "mouse" action, press->release soon, do NOT keep the pointerPressRaycast cause do NOT need to controll "down" object while not gazing.
                pointerData.pressPosition = pointerData.position;
                pointerData.pointerPressRaycast = pointerData.pointerCurrentRaycast;

                var _pointerDownGO = ExecuteEvents.ExecuteHierarchy (currentOverGO, pointerData, ExecuteEvents.pointerDownHandler);
                ExecuteEvents.ExecuteHierarchy (_pointerDownGO, pointerData, ExecuteEvents.pointerUpHandler);
            } else if (InputEvent == EGazeInputEvent.PointerSubmit)
            {
                ExecuteEvents.ExecuteHierarchy (currentOverGO, pointerData, ExecuteEvents.submitHandler);
            }
        }
    }

    private void GazeControl()
    {
        CastToCenterOfScreen ();
        OnTriggeGaze();
    }

    private bool EnableGaze = false;
    private Canvas[] sceneCanvases = null;
    protected override void OnEnable()
    {
        base.OnEnable ();

        EnableGaze = true;

        if (gazePointer == null)
        {
            // Set gazePointer only when null, or it will got null when WaveVR_Reticle gameObject is SetActive(false).
            if (Head == null)
                Head = WaveVR_Render.Instance.gameObject;
            if (Head != null)
                gazePointer = Head.GetComponentInChildren<WaveVR_Reticle> ();
        }

        if (gazePointer != null)
        {
            gazePointer.gameObject.SetActive (true);
            percentCanvas = gazePointer.transform.Find ("PercentCanvas").gameObject;
            counterCanvas = gazePointer.transform.Find ("CounterCanvas").gameObject;
        }

        sceneCanvases = GameObject.FindObjectsOfType<Canvas> ();
    }

    protected override void OnDisable()
    {
        base.OnDisable ();

        EnableGaze = false;
        if (gazePointer != null)
            gazePointer.gameObject.SetActive (false);

        if (pointerData != null)
            HandlePointerExitAndEnter (pointerData, null);
    }

    public override void Process()
    {
        if (EnableGaze)
            GazeControl ();
    }
}
