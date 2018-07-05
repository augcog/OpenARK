using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using wvr;
using WaveVR_Log;
using System;

public enum EControllerButtons
{
    Menu = WVR_InputId.WVR_InputId_Alias1_Menu,
    Touchpad = WVR_InputId.WVR_InputId_Alias1_Touchpad,
    Trigger = WVR_InputId.WVR_InputId_Alias1_Trigger
}

public enum ERaycastMode
{
    Beam,
    Fixed,
    Mouse
}

public enum ERaycastStartPoint
{
    CenterOfEyes,
    LeftEye,
    RightEye
}

public class WaveVR_ControllerInputModule : BaseInputModule
{
    private const string LOG_TAG = "WaveVR_ControllerInputModule";

    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + " " + msg);
        #endif
        Log.d (LOG_TAG, msg);
    }

    #region Developer specified parameters
    public GameObject RightController;
    public LayerMask RightRaycastMask = ~0;
    public GameObject LeftController;
    public LayerMask LeftRaycastMask = ~0;
    public EControllerButtons ButtonToTrigger = EControllerButtons.Touchpad;
    [HideInInspector]
    public ERaycastMode RaycastMode = ERaycastMode.Mouse;
    [HideInInspector]
    public ERaycastStartPoint RaycastStartPoint = ERaycastStartPoint.CenterOfEyes;

    [Tooltip("Will be obsoleted soon!")]
    public string CanvasTag = "EventCanvas";
    #endregion

    // Do NOT allow event DOWN being sent multiple times during CLICK_TIME.
    // Since UI element of Unity needs time to perform transitions.
    private const float CLICK_TIME = 0.1f;

    private const float raycastStartPointOffset = 0.0315f;

    private GameObject pointCameraL = null;
    private GameObject pointCameraR = null;

    #region basic declaration
    [SerializeField]
    private bool mForceModuleActive = true;

    public bool ForceModuleActive
    {
        get { return mForceModuleActive; }
        set { mForceModuleActive = value; }
    }

    public override bool IsModuleSupported()
    {
        return mForceModuleActive;
    }

    public override bool ShouldActivateModule()
    {
        if (!base.ShouldActivateModule ())
            return false;

        if (mForceModuleActive)
            return true;

        return false;
    }

    public override void DeactivateModule() {
        base.DeactivateModule();

        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            EventController _event_controller = (EventController)EventControllers [_dt];
            if (_event_controller != null)
            {
                PointerEventData _ped = _event_controller.event_data;
                if (_ped != null)
                {
                    // OnTriggerUp();
                    PrintDebugLog ("DeactivateModule() exit " + _ped.pointerEnter);
                    HandlePointerExitAndEnter (_ped, null);
                }
            }
        }
    }
    #endregion


    public class EventController
    {
        public WVR_DeviceType device {
            get;
            set;
        }

        public GameObject controller {
            get;
            set;
        }

        public GameObject prevRaycastedObject {
            get;
            set;
        }

        public PointerEventData event_data {
            get;
            set;
        }

        public WaveVR_ControllerPointer pointer {
            get;
            set;
        }

        public WaveVR_Beam beam {
            get;
            set;
        }

        public bool eligibleForButtonClick {
            get;
            set;
        }

        public EventController(WVR_DeviceType type)
        {
            device = type;
            controller = null;
            prevRaycastedObject = null;
            event_data = null;
            eligibleForButtonClick = false;
        }
    }

    private Hashtable EventControllers = new Hashtable();

    private void SetupEventController(EventController event_controller, GameObject controller_model)
    {
        LayerMask _mask = ~0;

        if (event_controller.controller != null)
        {
            PhysicsRaycaster _raycaster = event_controller.controller.GetComponentInChildren<PhysicsRaycaster> ();
            if (_raycaster != null)
                _mask = _raycaster.eventMask;
        }

        SetupEventController (event_controller, controller_model, _mask);
    }

    private void SetupEventController(EventController eventController, GameObject controller_model, LayerMask mask)
    {
        // Diactivate old controller, replace with new controller, activate new controller
        if (eventController.controller != null)
        {
            PrintDebugLog ("SetupEventController() deactivate " + eventController.controller.name);
            eventController.controller.SetActive (false);
        }

        eventController.controller = controller_model;

        if (eventController.controller != null)
        {
            PrintDebugLog ("SetupEventController() activate " + eventController.controller.name);
            eventController.controller.SetActive (true);

            // Get PhysicsRaycaster of controller. If none, add new one.
            PhysicsRaycaster _raycaster = eventController.controller.GetComponentInChildren<PhysicsRaycaster> ();
            if (_raycaster == null)
            {
                _raycaster = eventController.controller.AddComponent<PhysicsRaycaster> ();
                _raycaster.eventMask = mask;
            }

            // Get pointer and beam of controller.
            eventController.pointer = eventController.controller.GetComponentInChildren<WaveVR_ControllerPointer> ();
            eventController.beam = eventController.controller.GetComponentInChildren<WaveVR_Beam> ();
            Camera eventCam = eventController.controller.GetComponent<Camera>();
            if (eventCam != null)
                eventCam.enabled = false;
        }
    }

    private int GetIndexOfPointerCamera(WVR_DeviceType type)
    {
        switch(type)
        {
            case WVR_DeviceType.WVR_DeviceType_Controller_Left:
                return 0;
            case WVR_DeviceType.WVR_DeviceType_Controller_Right:
                return 1;
            default:
                return 0;
        }
    }

    private void SetupPointerCamera(WVR_DeviceType type)
    {
        if (type == WVR_DeviceType.WVR_DeviceType_Controller_Right)
        {
                GameObject head = GameObject.Find("head");
                pointCameraR = new GameObject("PointerCameraR");
                pointCameraR.AddComponent<WaveVR_PointerCameraTracker>();
                pointCameraR.AddComponent<WaveVR_PoseTrackerManager>();
                pointCameraR.AddComponent<PhysicsRaycaster>();
                pointCameraR.transform.SetParent(head.transform, true);
                if (RaycastStartPoint == ERaycastStartPoint.LeftEye)
                {
                    pointCameraR.transform.localPosition = new Vector3(-raycastStartPointOffset, 0f, 0.15f);
                }
                else if (RaycastStartPoint == ERaycastStartPoint.RightEye)
                {
                    pointCameraR.transform.localPosition = new Vector3(raycastStartPointOffset, 0f, 0.15f);
                }
                else
                {
                    pointCameraR.transform.localPosition = new Vector3(0f, 0f, 0.15f);
                }
                Camera pc = pointCameraR.GetComponent<Camera>();
                if (pc != null)
                {
                    pc.enabled = false;
                    pc.fieldOfView = 1f;
                    pc.nearClipPlane = 0.01f;
                }
                WaveVR_PointerCameraTracker pcTracker = pointCameraR.GetComponent<WaveVR_PointerCameraTracker>();
                if (pcTracker != null)
                {
                    pcTracker.setDeviceType(type);
                }
                WaveVR_PoseTrackerManager poseTracker = pointCameraR.GetComponent<WaveVR_PoseTrackerManager>();
                if (poseTracker != null)
                {
                    poseTracker.Type = type;
                    poseTracker.TrackPosition = false;
                    poseTracker.TrackRotation = false;
				    poseTracker.enabled = false;
                }
        }
        else if (type == WVR_DeviceType.WVR_DeviceType_Controller_Left)
        {
                GameObject head = GameObject.Find("head");
                pointCameraL = new GameObject("PointerCameraL");
                pointCameraL.AddComponent<WaveVR_PointerCameraTracker>();
                pointCameraL.AddComponent<WaveVR_PoseTrackerManager>();
                pointCameraL.AddComponent<PhysicsRaycaster>();
                pointCameraL.transform.SetParent(head.transform, true);
                if (RaycastStartPoint == ERaycastStartPoint.LeftEye)
                {
                    pointCameraL.transform.localPosition = new Vector3(-raycastStartPointOffset, 0f, 0.15f);
                }
                else if (RaycastStartPoint == ERaycastStartPoint.RightEye)
                {
                    pointCameraL.transform.localPosition = new Vector3(raycastStartPointOffset, 0f, 0.15f);
                }
                else
                {
                    pointCameraL.transform.localPosition = new Vector3(0f, 0f, 0.15f);
                }
                Camera pc = pointCameraL.GetComponent<Camera>();
                if (pc != null)
                {
                    pc.enabled = false;
                    pc.fieldOfView = 1f;
                    pc.nearClipPlane = 0.01f;
                }
                WaveVR_PointerCameraTracker pcTracker = pointCameraL.GetComponent<WaveVR_PointerCameraTracker>();
                if (pcTracker != null)
                {
                    pcTracker.setDeviceType(type);
                }
                WaveVR_PoseTrackerManager poseTracker = pointCameraL.GetComponent<WaveVR_PoseTrackerManager>();
                if (poseTracker != null)
                {
                    poseTracker.Type = type;
                    poseTracker.TrackPosition = false;
                    poseTracker.TrackRotation = false;
				    poseTracker.enabled = false;
                }
        }
    }

    #region Override BaseInputModule
    private bool enableControllerInputModule = false;
    protected override void OnEnable()
    {
        base.OnEnable ();
        PrintDebugLog ("OnEnable()");

        enableControllerInputModule = true;
        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            EventControllers.Add (_dt, new EventController (_dt));
        }

        // Right controller
        if (RightController != null)
        {
            SetupEventController (
                (EventController)EventControllers [WVR_DeviceType.WVR_DeviceType_Controller_Right],
                RightController,
                RightRaycastMask
            );
        }

        // Left controller
        if (LeftController != null)
        {
            SetupEventController (
                (EventController)EventControllers [WVR_DeviceType.WVR_DeviceType_Controller_Left],
                LeftController,
                LeftRaycastMask
            );
        }
    }

    protected override void OnDisable()
    {
        base.OnDisable ();

        enableControllerInputModule = false;
        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            EventController _event_controller = (EventController)EventControllers [_dt];
            if (_event_controller != null)
            {
                PointerEventData _ped = _event_controller.event_data;
                if (_ped != null)
                {
                    PrintDebugLog ("OnDisable() exit " + _ped.pointerEnter);
                    HandlePointerExitAndEnter (_ped, null);
                }
            }
        }
        pointCameraL = null;
        pointCameraR = null;
        EventControllers.Clear ();
    }

    public override void Process()
    {
        if (!enableControllerInputModule)
            return;

        SetControllerModel ();
        SetPointerCameraTracker ();

        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            // HMD uses Gaze, not controller input module.
            if (_dt == WVR_DeviceType.WVR_DeviceType_HMD)
                continue;

            EventController _eventController = (EventController)EventControllers [_dt];
            if (_eventController == null)
                continue;

            GameObject _controller = _eventController.controller;
            if (_controller == null)
                continue;

            bool _connected = false;
            #if UNITY_EDITOR
            if (Application.isEditor)
            {
                // "connected" from WaveVR_Controller means the real connection status of controller.
                _connected = WaveVR_Controller.Input (_dt).connected;
            } else
            #endif
            {
                // "connected" from WaveVR means the "pose" is valid or not.
                WaveVR.Device _device = WaveVR.Instance.getDeviceByType (_dt);
                _connected = _device.connected;
            }
            if (!_connected)
                continue;

            if (_eventController.pointer == null)
            {
                _eventController.pointer = _eventController.controller.GetComponentInChildren<WaveVR_ControllerPointer> ();
            }

            if (_connected)
            {
                if ((_dt == WVR_DeviceType.WVR_DeviceType_Controller_Left && pointCameraL == null) || (_dt == WVR_DeviceType.WVR_DeviceType_Controller_Right && pointCameraR == null))
                    SetupPointerCamera(_dt);
            }

            Camera _event_camera;

            // Mouse mode: raycasting from HMD after direct raycasting from controller
            if (RaycastMode == ERaycastMode.Mouse)
            {
                _event_camera = _dt == WVR_DeviceType.WVR_DeviceType_Controller_Left ? pointCameraL.GetComponent<Camera>() : pointCameraR.GetComponent<Camera>();
                _eventController.prevRaycastedObject = GetRaycastedObject(_dt);
				if (_eventController.pointer != null) {
                    _eventController.pointer.setRaycastMode(RaycastMode);
                }
                // 1. Get graphic raycast object.
                ResetPointerEventData_Hybrid (_dt, _event_camera);
            }
            else
            {
                _event_camera = (Camera)_controller.GetComponent (typeof(Camera));
                if (_event_camera == null)
                    continue;

                if (_eventController.pointer != null) {
                    _eventController.pointer.setRaycastMode(RaycastMode);
                }

                _eventController.prevRaycastedObject = GetRaycastedObject (_dt);
                // 1. Get graphic raycast object.
                ResetPointerEventData (_dt);
            }
            GraphicRaycast (_eventController, _event_camera);

            if (GetRaycastedObject (_dt) == null)
            {
                // 2. Get physic raycast object.
                PhysicsRaycaster _raycaster = null;
                if (RaycastMode == ERaycastMode.Mouse)
                {
                    _raycaster = _event_camera.GetComponent<PhysicsRaycaster>();
                }
                else
                {
                    _raycaster = _controller.GetComponent<PhysicsRaycaster>();
                }
                if (_raycaster == null)
                    continue;

                if (RaycastMode == ERaycastMode.Mouse)
                    ResetPointerEventData_Hybrid (_dt, _event_camera);
                else
                    ResetPointerEventData (_dt);

                PhysicRaycast (_eventController, _raycaster);
            }

            // 3. Exit previous object, enter new object.
            OnTriggerEnterAndExit (_dt, _eventController.event_data);

            // 4. Hover object.
            GameObject _curRaycastedObject = GetRaycastedObject (_dt);
            if (_curRaycastedObject != null && _curRaycastedObject == _eventController.prevRaycastedObject)
            {
                OnTriggerHover (_dt, _eventController.event_data);
            }

            // 5. Get button state.
            bool btnPressDown = false, btnPressed = false, btnPressUp = false;
            if (_dt == WVR_DeviceType.WVR_DeviceType_Controller_Left || _dt == WVR_DeviceType.WVR_DeviceType_Controller_Right)
            {
                int _mousekey = _dt == WVR_DeviceType.WVR_DeviceType_Controller_Left ? 0 : 1;
                btnPressDown = Input.GetMouseButtonDown (_mousekey);
                btnPressed = Input.GetMouseButton (_mousekey);
                btnPressUp = Input.GetMouseButtonUp (_mousekey);
            }
            btnPressDown |= WaveVR_Controller.Input (_dt).GetPressDown ((WVR_InputId)ButtonToTrigger);
            btnPressed |= WaveVR_Controller.Input (_dt).GetPress ((WVR_InputId)ButtonToTrigger);
            btnPressUp |= WaveVR_Controller.Input (_dt).GetPressUp ((WVR_InputId)ButtonToTrigger);

            if (btnPressDown)
                _eventController.eligibleForButtonClick = true;
            // Pointer Click equals to Button.onClick, we sent Pointer Click in OnTriggerUp()
            //if (btnPressUp && _eventController.eligibleForButtonClick)
                //onButtonClick (_eventController);

            if (!btnPressDown && btnPressed)
            {
                // button hold means to drag.
                OnDrag (_dt, _eventController.event_data);
            } else if (Time.unscaledTime - _eventController.event_data.clickTime < CLICK_TIME)
            {
                // Delay new events until CLICK_TIME has passed.
            } else if (btnPressDown && !_eventController.event_data.eligibleForClick)
            {
                // 1. button not pressed -> pressed.
                // 2. no pending Click should be procced.
                OnTriggerDown (_dt, _eventController.event_data);
            } else if (!btnPressed)
            {
                // 1. If Down before, send Up event and clear Down state.
                // 2. If Dragging, send Drop & EndDrag event and clear Dragging state.
                // 3. If no Down or Dragging state, do NOTHING.
                OnTriggerUp (_dt, _eventController.event_data);
            }

            if (RaycastMode != ERaycastMode.Mouse)
                UpdateReticlePointer (_eventController);
        }
    }
    #endregion

    private void UpdateReticlePointer(EventController event_controller)
    {
        WVR_DeviceType _type                = event_controller.device;
        GameObject _prevObject              = event_controller.prevRaycastedObject;
        PointerEventData _event_data        = event_controller.event_data;
        WaveVR_ControllerPointer _pointer   = event_controller.pointer;
        WaveVR_Beam _beam                   = event_controller.beam;

        if (_pointer != null && _beam != null)
        {
            Vector3 _intersectionPosition = GetIntersectionPosition (_event_data.enterEventCamera, _event_data.pointerCurrentRaycast);
            GameObject _go = GetRaycastedObject (_type);

            if (_go != _prevObject)
            {
                if (_go != null)
                {
                    _pointer.SetPointerColor (new Color32 (11, 220, 249, 255));
                    _pointer.OnPointerEnter (_event_data.enterEventCamera, _go, _intersectionPosition, true);
                    _beam.SetEndOffset (_intersectionPosition, false);
                } else
                {
                    _pointer.SetPointerColor (Color.white);
                    _pointer.OnPointerExit(_event_data.enterEventCamera, _prevObject);
                    _beam.ResetEndOffset ();
                }
            }
        }
    }

    #region EventSystem
    private void OnTriggerDown(WVR_DeviceType type, PointerEventData event_data)
    {
        GameObject _go = GetRaycastedObject (type);
        if (_go == null)
            return;

        // Send Pointer Down. If not received, get handler of Pointer Click.
        event_data.pressPosition = event_data.position;
        event_data.pointerPressRaycast = event_data.pointerCurrentRaycast;
        event_data.pointerPress =
            ExecuteEvents.ExecuteHierarchy(_go, event_data, ExecuteEvents.pointerDownHandler)
            ?? ExecuteEvents.GetEventHandler<IPointerClickHandler>(_go);

        PrintDebugLog ("OnTriggerDown() device: " + type + " send Pointer Down to " + event_data.pointerPress + ", current GameObject is " + _go);

        // If Drag Handler exists, send initializePotentialDrag event.
        event_data.pointerDrag = ExecuteEvents.GetEventHandler<IDragHandler>(_go);
        if (event_data.pointerDrag != null)
        {
            PrintDebugLog ("OnTriggerDown() device: " + type + " send initializePotentialDrag to " + event_data.pointerDrag + ", current GameObject is " + _go);
            ExecuteEvents.Execute(event_data.pointerDrag, event_data, ExecuteEvents.initializePotentialDrag);
        }

        // press happened (even not handled) object.
        event_data.rawPointerPress = _go;
        // allow to send Pointer Click event
        event_data.eligibleForClick = true;
        // reset the screen position of press, can be used to estimate move distance
        event_data.delta = Vector2.zero;
        // current Down, reset drag state
        event_data.dragging = false;
        event_data.useDragThreshold = true;
        // record the count of Pointer Click should be processed, clean when Click event is sent.
        event_data.clickCount = 1;
        // set clickTime to current time of Pointer Down instead of Pointer Click.
        // since Down & Up event should not be sent too closely. (< CLICK_TIME)
        event_data.clickTime = Time.unscaledTime;
    }

    private void OnTriggerUp(WVR_DeviceType type, PointerEventData event_data)
    {
        if (!event_data.eligibleForClick && !event_data.dragging)
        {
            // 1. no pending click
            // 2. no dragging
            // Mean user has finished all actions and do NOTHING in current frame.
            return;
        }

        GameObject _go = GetRaycastedObject (type);
        // _go may be different with event_data.pointerDrag so we don't check null

        if (event_data.pointerPress != null)
        {
            // In the frame of button is pressed -> unpressed, send Pointer Up
            PrintDebugLog ("OnTriggerUp type: " + type + " send Pointer Up to " + event_data.pointerPress);
            ExecuteEvents.Execute (event_data.pointerPress, event_data, ExecuteEvents.pointerUpHandler);
        }
        if (event_data.eligibleForClick)
        {
            // In the frame of button from being pressed to unpressed, send Pointer Click if Click is pending.
            PrintDebugLog ("OnTriggerUp type: " + type + " send Pointer Click to " + event_data.pointerPress);
            ExecuteEvents.Execute(event_data.pointerPress, event_data, ExecuteEvents.pointerClickHandler);
        } else if (event_data.dragging)
        {
            // In next frame of button from being pressed to unpressed, send Drop and EndDrag if dragging.
            PrintDebugLog ("OnTriggerUp type: " + type + " send Pointer Drop to " + _go + ", EndDrag to " + event_data.pointerDrag);
            ExecuteEvents.ExecuteHierarchy(_go, event_data, ExecuteEvents.dropHandler);
            ExecuteEvents.Execute(event_data.pointerDrag, event_data, ExecuteEvents.endDragHandler);

            event_data.pointerDrag = null;
            event_data.dragging = false;
        }

        // Down of pending Click object.
        event_data.pointerPress = null;
        // press happened (even not handled) object.
        event_data.rawPointerPress = null;
        // clear pending state.
        event_data.eligibleForClick = false;
        // Click is processed, clearcount.
        event_data.clickCount = 0;
        // Up is processed thus clear the time limitation of Down event.
        event_data.clickTime = 0;
    }

    private void OnDrag(WVR_DeviceType type, PointerEventData event_data)
    {
        if (event_data.pointerDrag != null && !event_data.dragging)
        {
            PrintDebugLog ("OnDrag() device: " + type + " send BeginDrag to " + event_data.pointerDrag);
            ExecuteEvents.Execute(event_data.pointerDrag, event_data, ExecuteEvents.beginDragHandler);
            event_data.dragging = true;
        }

        // Drag notification
        if (event_data.dragging && event_data.pointerDrag != null)
        {
            // Before doing drag we should cancel any pointer down state
            if (event_data.pointerPress != event_data.pointerDrag)
            {
                PrintDebugLog ("OnDrag device: " + type + " send Pointer Up to " + event_data.pointerPress);
                ExecuteEvents.Execute(event_data.pointerPress, event_data, ExecuteEvents.pointerUpHandler);

                // since Down state is cleaned, no Click should be processed.
                event_data.eligibleForClick = false;
                event_data.pointerPress = null;
                event_data.rawPointerPress = null;
            }
            /*
            PrintDebugLog ("OnDrag() device: " + type + " send Pointer Drag to " + event_data.pointerDrag +
                "camera: " + event_data.enterEventCamera +
                " (" + event_data.enterEventCamera.ScreenToWorldPoint (
                    new Vector3 (
                        event_data.position.x,
                        event_data.position.y,
                        event_data.pointerDrag.transform.position.z
                    )) +
                ")");
            */
            ExecuteEvents.Execute(event_data.pointerDrag, event_data, ExecuteEvents.dragHandler);
        }
    }

    private void OnTriggerHover(WVR_DeviceType type, PointerEventData event_data)
    {
        GameObject _go = GetRaycastedObject (type);

        ExecuteEvents.ExecuteHierarchy(_go, event_data, WaveVR_ExecuteEvents.pointerHoverHandler);
    }

    private void OnTriggerEnterAndExit(WVR_DeviceType type, PointerEventData event_data)
    {
        GameObject _go = GetRaycastedObject (type);

        if (event_data.pointerEnter != _go)
        {
            PrintDebugLog ("OnTriggerEnterAndExit() enter: " + _go + ", exit: " + event_data.pointerEnter);

            HandlePointerExitAndEnter (event_data, _go);

            PrintDebugLog ("OnTriggerEnterAndExit() pointerEnter: " + event_data.pointerEnter + ", camera: " + event_data.enterEventCamera);
        }
    }
    #endregion

    private void onButtonClick(EventController event_controller)
    {
        GameObject _go = GetRaycastedObject (event_controller.device);
        event_controller.eligibleForButtonClick = false;

        if (_go == null)
            return;

        Button _btn = _go.GetComponent<Button> ();
        if (_btn != null)
        {
            PrintDebugLog ("onButtonClick() trigger Button.onClick to " + _btn + " from " + event_controller.device);
            _btn.onClick.Invoke ();
        } else
        {
            PrintDebugLog ("onButtonClick() " + event_controller.device + ", " + _go.name + " does NOT contain Button!");
        }
    }

    private void PhysicRaycast(EventController event_controller, PhysicsRaycaster raycaster)
    {
        List<RaycastResult> _raycast_results = new List<RaycastResult>();
        raycaster.Raycast (event_controller.event_data, _raycast_results);

        RaycastResult _firstResult = FindFirstRaycast (_raycast_results);
        event_controller.event_data.pointerCurrentRaycast = _firstResult;

        #if UNITY_EDITOR
        if (_firstResult.module != null)
        {
            //Debug.Log ("PhysicRaycast() device: " + event_controller.device + ", camera: " + _firstResult.module.eventCamera + ", first result = " + _firstResult);
        }
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
                event_controller.event_data.pointerCurrentRaycast = _firstResult;
            }

            event_controller.event_data.position = _firstResult.screenPosition;
        }
    }

    private void GraphicRaycast(EventController event_controller, Camera event_camera)
    {
        // --------------------- Find GUIs those can be raycasted begins. ---------------------
        // 1. find Canvas by TAG
        GameObject[] _tag_GUIs = GameObject.FindGameObjectsWithTag (CanvasTag);
        // 2. Get Canvas from Pointer Canvas Provider
        GameObject[] _event_GUIs = WaveVR_EventSystemGUIProvider.GetEventGUIs();

        GameObject[] _GUIs = MergeArray (_tag_GUIs, _event_GUIs);
        // --------------------- Find GUIs those can be raycasted ends. ---------------------

        List<RaycastResult> _raycast_results = new List<RaycastResult>();

        // Reset pointerCurrentRaycast even no GUI.
        RaycastResult _firstResult = new RaycastResult ();
        event_controller.event_data.pointerCurrentRaycast = _firstResult;

        foreach (GameObject _GUI in _GUIs)
        {
            Canvas _canvas = (Canvas)_GUI.GetComponent (typeof(Canvas));
            if (_canvas == null)
                continue;

            GraphicRaycaster _gr = _canvas.GetComponent<GraphicRaycaster> ();
            if (_gr == null)
                continue;

            // 1. Change event camera.
            _canvas.worldCamera = event_camera;

            // 2.
            _gr.Raycast (event_controller.event_data, _raycast_results);

            _firstResult = FindFirstRaycast (_raycast_results);
            event_controller.event_data.pointerCurrentRaycast = _firstResult;
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
                    event_controller.event_data.pointerCurrentRaycast = _firstResult;
                }

                event_controller.event_data.position = _firstResult.screenPosition;
                break;
            }
        }
    }

    private void ResetPointerEventData(WVR_DeviceType type)
    {
        EventController _eventController = (EventController)EventControllers [type];
        if (_eventController != null)
        {
            if (_eventController.event_data == null)
                _eventController.event_data = new PointerEventData (eventSystem);

            _eventController.event_data.Reset ();
            _eventController.event_data.position = new Vector2 (0.5f * Screen.width, 0.5f * Screen.height); // center of screen
        }
    }

    private void ResetPointerEventData_Hybrid(WVR_DeviceType type, Camera eventCam)
    {
        EventController _eventController = (EventController)EventControllers[type];
        if (_eventController != null && eventCam != null)
        {
            if (_eventController.event_data == null)
                _eventController.event_data = new PointerEventData(EventSystem.current);

            _eventController.event_data.Reset();
            _eventController.event_data.position = new Vector2(0.5f * eventCam.pixelWidth, 0.5f * eventCam.pixelHeight); // center of screen
        }
    }

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

    private GameObject GetRaycastedObject(WVR_DeviceType type)
    {
        PointerEventData _ped = ((EventController)EventControllers [type]).event_data;
        if (_ped != null)
            return _ped.pointerCurrentRaycast.gameObject;
        return null;
    }

    private void SetControllerModel()
    {
        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            // HMD uses Gaze, not controller input module.
            if (_dt == WVR_DeviceType.WVR_DeviceType_HMD)
                continue;

            if (EventControllers [_dt] == null)
                continue;

            GameObject _controller = (GameObject)((EventController)EventControllers [_dt]).controller;
            GameObject _model = WaveVR_EventSystemControllerProvider.Instance.GetControllerModel (_dt);

            if (_controller == null)
            {
                if (_model != null)
                {
                    // replace with new controller instance.
                    SetupEventController ((EventController)EventControllers [_dt], _model);
                }
            } else
            {
                if (_model == null)
                {
                    if (WaveVR_EventSystemControllerProvider.Instance.HasControllerLoader(_dt))
                    {
                        // clear controller instance.
                        SetupEventController ((EventController)EventControllers [_dt], null);
                    }
                } else
                {
                    if (!GameObject.ReferenceEquals (_controller, _model))
                    {
                        // replace with new controller instance.
                        SetupEventController ((EventController)EventControllers [_dt], _model);
                    }
                }
            }
        }
    }

    private void SetPointerCameraTracker()
    {
        foreach (WVR_DeviceType _dt in Enum.GetValues(typeof(WVR_DeviceType)))
        {
            // HMD uses Gaze, not controller input module.
            if (_dt == WVR_DeviceType.WVR_DeviceType_HMD)
                continue;

            if (EventControllers [_dt] == null)
                continue;

            if (_dt == WVR_DeviceType.WVR_DeviceType_Controller_Left && pointCameraL != null)
            {
                WaveVR_PointerCameraTracker pcTracker = pointCameraL.GetComponent<WaveVR_PointerCameraTracker>();
                if (pcTracker != null && pcTracker.reticleObject == null)
                {
                    EventController _eventController = (EventController) EventControllers[_dt];
                    bool isConnected = true;
                    #if UNITY_EDITOR
                    if (Application.isEditor)
                    {
                        isConnected = true;
                    } else
                    #endif
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (_dt);
                        isConnected = _device.connected;
                    }
                    if (_eventController != null && isConnected)
                    {
                        if (_eventController.pointer == null && _eventController.controller != null)
                            _eventController.pointer = _eventController.controller.GetComponentInChildren<WaveVR_ControllerPointer> ();
                        if (_eventController.pointer != null)
                        {
                            pcTracker.reticleObject = _eventController.pointer.gameObject;
                        }
                    }
                }
            }

            if (_dt == WVR_DeviceType.WVR_DeviceType_Controller_Right && pointCameraR != null)
            {
                WaveVR_PointerCameraTracker pcTracker = pointCameraR.GetComponent<WaveVR_PointerCameraTracker>();
                if (pcTracker != null && pcTracker.reticleObject == null)
                {
                    EventController _eventController = (EventController) EventControllers[_dt];
                    bool isConnected = true;

                    #if UNITY_EDITOR
                    if (Application.isEditor)
                    {
                        isConnected = true;
                    } else
                    #endif
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (_dt);
                        isConnected = _device.connected;
                    }
                    if (_eventController != null && isConnected)
                    {
                        if (_eventController.pointer == null && _eventController.controller != null)
                            _eventController.pointer = _eventController.controller.GetComponentInChildren<WaveVR_ControllerPointer>();
                        if (_eventController.pointer != null)
                        {
                            pcTracker.reticleObject = _eventController.pointer.gameObject;
                        }
                    }
                }
            }
        }
    }

    private GameObject[] MergeArray(GameObject[] start, GameObject[] end)
    {
        GameObject[] _merged = null;

        if (start == null)
        {
            if (end != null)
                _merged = end;
        } else
        {
            if (end == null)
            {
                _merged = start;
            } else
            {
                uint _duplicate = 0;
                for (int i = 0; i < start.Length; i++)
                {
                    for (int j = 0; j < end.Length; j++)
                    {
                        if (GameObject.ReferenceEquals (start [i], end [j]))
                        {
                            _duplicate++;
                            end [j] = null;
                        }
                    }
                }

                _merged = new GameObject[start.Length + end.Length - _duplicate];
                uint _merge_index = 0;

                for (int i = 0; i < start.Length; i++)
                    _merged [_merge_index++] = start [i];

                for (int j = 0; j < end.Length; j++)
                {
                    if (end [j] != null)
                        _merged [_merge_index++] = end [j];
                }

                //Array.Copy (start, _merged, start.Length);
                //Array.Copy (end, 0, _merged, start.Length, end.Length);
            }
        }

        return _merged;
    }
}