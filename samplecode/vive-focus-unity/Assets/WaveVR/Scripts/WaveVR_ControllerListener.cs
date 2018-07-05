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
using System;

public class WaveVR_ControllerListener : MonoBehaviour
{
    private static string LOG_TAG = "WaveVR_ControllerListener";

    private static WaveVR_ControllerListener instance = null;
    public static WaveVR_ControllerListener Instance
    {
        get {
            if (instance == null)
            {
                Log.i (LOG_TAG, "Instance, create WaveVR_ControllerListener GameObject");
                var gameObject = new GameObject("WaveVR_ControllerListener");
                instance = gameObject.AddComponent<WaveVR_ControllerListener>();
                // This object should survive all scene transitions.
                GameObject.DontDestroyOnLoad(instance);
            }
            return instance;
        }
    }

    private static Device[] devices;

    /// <summary>
    /// Get the controller by device index.
    /// </summary>
    /// <param name="deviceIndex">The index of the controller.</param>
    /// <returns></returns>
    public Device Input(WVR_DeviceType deviceIndex)
    {
        if (devices == null)
        {
            devices = new Device[Enum.GetNames (typeof(WVR_DeviceType)).Length];
            uint i = 0;
            devices [i++] = new Device (WVR_DeviceType.WVR_DeviceType_HMD);
            devices [i++] = new Device (WVR_DeviceType.WVR_DeviceType_Controller_Right);
            devices [i++] = new Device (WVR_DeviceType.WVR_DeviceType_Controller_Left);
        }

        for (uint i = 0; i < devices.Length; i++)
        {
            if (deviceIndex == devices [i].DeviceType)
            {
                return devices [i];
            }
        }

        return null;
    }

    public class Device
    {
        public static ulong Input_Mask_Menu        = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Menu;
        public static ulong Input_Mask_Grip        = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Grip;
        public static ulong Input_Mask_Touchpad    = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Touchpad;
        public static ulong Input_Mask_Trigger     = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Trigger;

        public IEnumerator ieConnection;
        public IEnumerator iePressMenu, iePressGrip, iePressTouchpad, iePressTrigger;
        public IEnumerator ieTouchTouchpad, ieTouchTrigger;

        public Device(WVR_DeviceType dt)
        {
            Log.i (LOG_TAG, "Initialize WaveVR_Controller Device: " + dt);
            DeviceType = dt;

            ieConnection    = PollingConnectionStatus();
            iePressMenu     = PollingPressState_Menu();
            iePressGrip     = PollingPressState_Grip();
            iePressTouchpad = PollingPressState_Touchpad();
            iePressTrigger  = PollingPressState_Trigger();
            ieTouchTouchpad = PollingTouchState_Touchpad();
            ieTouchTrigger  = PollingTouchState_Trigger();
        }

        public WVR_DeviceType DeviceType
        {
            get;
            private set;
        }

        internal WaveVR_Utils.RigidTransform rt = WaveVR_Utils.RigidTransform.identity;

        public WaveVR_Utils.RigidTransform transform
        {
            get
            {
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                    return WaveVR_Utils.RigidTransform.identity;
                #endif

                WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                return _device.rigidTransform;
            }
        }

        internal WaveVR_Utils.WVR_ButtonState_t state, pre_state;
        internal WVR_Axis_t axis;
        internal WVR_PoseState_t pose;

        #if UNITY_EDITOR || UNITY_STANDALONE
        private bool isEditorMode = true;
        //internal WVR_ButtonState_t emu_state;
        #endif

        public delegate void ButtonEventHandler();
        // Listeners of press
        public event ButtonEventHandler PressDownListenersMenu;
        public event ButtonEventHandler PressDownListenersGrip;
        public event ButtonEventHandler PressDownListenersTouchpad;
        public event ButtonEventHandler PressDownListenersTrigger;
        public event ButtonEventHandler PressUpListenersMenu;
        public event ButtonEventHandler PressUpListenersGrip;
        public event ButtonEventHandler PressUpListenersTouchpad;
        public event ButtonEventHandler PressUpListenersTrigger;
        // Listeners of touch
        public event ButtonEventHandler TouchDownListenersTouchpad;
        public event ButtonEventHandler TouchDownListenersTrigger;
        public event ButtonEventHandler TouchUpListenersTouchpad;
        public event ButtonEventHandler TouchUpListenersTrigger;

        private WaitForEndOfFrame waitForEndOfFrame = new WaitForEndOfFrame();

        #region Connection
        public bool connected
        {
            get;
            private set;
        }

        public delegate void ConnectionEventHandler(bool value);
        public event ConnectionEventHandler ConnectionStatusListeners;

        IEnumerator PollingConnectionStatus()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _c = WaveVR_Controller.Input (this.DeviceType).connected;
                if (connected != _c)
                {
                    connected = !connected;
                    if (ConnectionStatusListeners != null)
                        ConnectionStatusListeners (connected);
                }
            }
        }
        #endregion

        #region Button Press
        /// <summary>
        /// Check if button state is equivallent to specified state.
        /// </summary>
        /// <returns><c>true</c>, equal, <c>false</c> otherwise.</returns>
        /// <param name="_id">input button</param>
        public bool GetPress(WVR_InputId _id)
        {
            bool _state = false;

            switch (_id)
            {
            case WVR_InputId.WVR_InputId_Alias1_Menu:
                _state = (state.BtnPressed & Input_Mask_Menu) != 0 ? true : false;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Grip:
                _state = (state.BtnPressed & Input_Mask_Grip) != 0 ? true : false;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Touchpad:
                _state = (state.BtnPressed & Input_Mask_Touchpad) != 0 ? true : false;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Trigger:
                _state = (state.BtnPressed & Input_Mask_Trigger) != 0 ? true : false;
                break;
            default:
                break;
            }

            return _state;
        }

        IEnumerator PollingPressState_Menu()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _connected = false;
                WVR_DeviceType _type = this.DeviceType;
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                    _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                        _connected = _device.connected;
                        _type = _device.type;
                    }
                }

                bool _pressed = false;

                var _menuvalue = state.BtnPressed & Input_Mask_Menu;
                if (_menuvalue == 0)
                    pre_state.BtnPressed &= ~Input_Mask_Menu;
                else
                    pre_state.BtnPressed |= _menuvalue;

                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    var system = WaveVR_PoseSimulator.Instance;
                    _pressed = system.GetButtonPressState(_type, WVR_InputId.WVR_InputId_Alias1_Menu);
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        if (_connected)
                            _pressed = Interop.WVR_GetInputButtonState (_type, WVR_InputId.WVR_InputId_Alias1_Menu);
                    }
                }   // isEditorMode

                if (_pressed)
                {
                    state.BtnPressed |= Input_Mask_Menu;
                } else
                {
                    state.BtnPressed &= ~Input_Mask_Menu;
                }

                if ((pre_state.BtnPressed ^ state.BtnPressed) != 0) // pre not equal to cur
                {
                    if (((state.BtnPressed & Input_Mask_Menu) != 0) && ((pre_state.BtnPressed & Input_Mask_Menu) == 0))
                    {
                        // unpressed -> pressed
                        if (PressDownListenersMenu != null)
                            PressDownListenersMenu ();
                    }

                    if (((state.BtnPressed & Input_Mask_Menu) == 0) && ((pre_state.BtnPressed & Input_Mask_Menu) != 0))
                    {
                        // pressed -> unpressed
                        if (PressUpListenersMenu != null)
                            PressUpListenersMenu ();
                    }
                }
            }
        }

        IEnumerator PollingPressState_Grip()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _connected = false;
                WVR_DeviceType _type = this.DeviceType;
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                    _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                        _connected = _device.connected;
                        _type = _device.type;
                    }
                }

                bool _pressed = false;

                var _gripvalue = state.BtnPressed & Input_Mask_Grip;
                if (_gripvalue == 0)
                    pre_state.BtnPressed &= ~Input_Mask_Grip;
                else
                    pre_state.BtnPressed |= _gripvalue;

                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    var system = WaveVR_PoseSimulator.Instance;
                    _pressed = system.GetButtonPressState(_type, WVR_InputId.WVR_InputId_Alias1_Grip);
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        if (_connected)
                            _pressed = Interop.WVR_GetInputButtonState (_type, WVR_InputId.WVR_InputId_Alias1_Grip);
                    }
                }   // isEditorMode

                if (_pressed)
                {
                    state.BtnPressed |= Input_Mask_Grip;
                } else
                {
                    state.BtnPressed &= ~Input_Mask_Grip;
                }

                if ((pre_state.BtnPressed ^ state.BtnPressed) != 0) // pre not equal to cur
                {
                    if (((state.BtnPressed & Input_Mask_Grip) != 0) && ((pre_state.BtnPressed & Input_Mask_Grip) == 0))
                    {
                        // unpressed -> pressed
                        if (PressDownListenersGrip != null)
                            PressDownListenersGrip ();
                    }

                    if (((state.BtnPressed & Input_Mask_Grip) == 0) && ((pre_state.BtnPressed & Input_Mask_Grip) != 0))
                    {
                        // pressed -> unpressed
                        if (PressUpListenersGrip != null)
                            PressUpListenersGrip ();
                    }
                }
            }
        }

        IEnumerator PollingPressState_Touchpad()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _connected = false;
                WVR_DeviceType _type = this.DeviceType;
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                    _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                        _connected = _device.connected;
                        _type = _device.type;
                    }
                }

                bool _pressed = false;

                var _padvalue = state.BtnPressed & Input_Mask_Touchpad;
                if (_padvalue == 0)
                    pre_state.BtnPressed &= ~Input_Mask_Touchpad;
                else
                    pre_state.BtnPressed |= _padvalue;

                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    var system = WaveVR_PoseSimulator.Instance;
                    _pressed = system.GetButtonPressState(_type, WVR_InputId.WVR_InputId_Alias1_Touchpad);
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        if (_connected)
                            _pressed = Interop.WVR_GetInputButtonState (_type, WVR_InputId.WVR_InputId_Alias1_Touchpad);
                    }
                }   // isEditorMode

                if (_pressed)
                {
                    state.BtnPressed |= Input_Mask_Touchpad;
                } else
                {
                    state.BtnPressed &= ~Input_Mask_Touchpad;
                }

                if ((pre_state.BtnPressed ^ state.BtnPressed) != 0) // pre not equal to cur
                {
                    if (((state.BtnPressed & Input_Mask_Touchpad) != 0) && ((pre_state.BtnPressed & Input_Mask_Touchpad) == 0))
                    {
                        // unpressed -> pressed
                        if (PressDownListenersTouchpad != null)
                            PressDownListenersTouchpad ();
                    }

                    if (((state.BtnPressed & Input_Mask_Touchpad) == 0) && ((pre_state.BtnPressed & Input_Mask_Touchpad) != 0))
                    {
                        // pressed -> unpressed
                        if (PressUpListenersTouchpad != null)
                            PressUpListenersTouchpad ();
                    }
                }
            }
        }

        IEnumerator PollingPressState_Trigger()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _connected = false;
                WVR_DeviceType _type = this.DeviceType;
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                    _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                        _connected = _device.connected;
                        _type = _device.type;
                    }
                }

                bool _pressed = false;

                var _triggervalue = state.BtnPressed & Input_Mask_Trigger;
                if (_triggervalue == 0)
                    pre_state.BtnPressed &= ~Input_Mask_Trigger;
                else
                    pre_state.BtnPressed |= _triggervalue;

                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    var system = WaveVR_PoseSimulator.Instance;
                    _pressed = system.GetButtonPressState(_type, WVR_InputId.WVR_InputId_Alias1_Trigger);
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        if (_connected)
                            _pressed = Interop.WVR_GetInputButtonState (_type, WVR_InputId.WVR_InputId_Alias1_Trigger);
                    }
                }   // isEditorMode

                if (_pressed)
                {
                    state.BtnPressed |= Input_Mask_Trigger;
                } else
                {
                    state.BtnPressed &= ~Input_Mask_Trigger;
                }

                if ((pre_state.BtnPressed ^ state.BtnPressed) != 0) // pre not equal to cur
                {
                    if (((state.BtnPressed & Input_Mask_Trigger) != 0) && ((pre_state.BtnPressed & Input_Mask_Trigger) == 0))
                    {
                        // unpressed -> pressed
                        if (PressDownListenersTrigger != null)
                            PressDownListenersTrigger ();
                    }

                    if (((state.BtnPressed & Input_Mask_Trigger) == 0) && ((pre_state.BtnPressed & Input_Mask_Trigger) != 0))
                    {
                        // pressed -> unpressed
                        if (PressUpListenersTrigger != null)
                            PressUpListenersTrigger ();
                    }
                }
            }
        }
        #endregion

        #region Button Touch
        /// <summary>
        /// If true, button with _id is touched, else untouched..
        /// </summary>
        /// <returns><c>true</c>, if touch was gotten, <c>false</c> otherwise.</returns>
        /// <param name="_id">WVR_ButtonId, id of button</param>
        public bool GetTouch(WVR_InputId _id)
        {
            bool _state = false;

            switch (_id)
            {
            case WVR_InputId.WVR_InputId_Alias1_Touchpad:
                _state = (state.BtnTouched & Input_Mask_Touchpad) != 0 ? true : false;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Trigger:
                _state = (state.BtnTouched & Input_Mask_Trigger) != 0 ? true : false;
                break;
            default:
                break;
            }

            return _state;
        }

        IEnumerator PollingTouchState_Touchpad()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _connected = false;
                WVR_DeviceType _type = this.DeviceType;
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                    _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                        _connected = _device.connected;
                        _type = _device.type;
                    }
                }

                bool _touched = false;

                var _touchpadvalue = state.BtnTouched & Input_Mask_Touchpad;
                if (_touchpadvalue == 0)
                    pre_state.BtnTouched &= ~Input_Mask_Touchpad;
                else
                    pre_state.BtnTouched |= _touchpadvalue;

                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    var system = WaveVR_PoseSimulator.Instance;
                    _touched = system.GetButtonTouchState(_type, WVR_InputId.WVR_InputId_Alias1_Touchpad);
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        if (_connected)
                            _touched = Interop.WVR_GetInputTouchState (_type, WVR_InputId.WVR_InputId_Alias1_Touchpad);
                    }
                }   // isEditorMode

                if (_touched)
                {
                    state.BtnTouched |= Input_Mask_Touchpad;
                } else
                {
                    state.BtnTouched &= ~Input_Mask_Touchpad;
                }

                if ((pre_state.BtnTouched ^ state.BtnTouched) != 0) // pre not equal to cur
                {
                    if (((state.BtnTouched & Input_Mask_Touchpad) != 0) && ((pre_state.BtnTouched & Input_Mask_Touchpad) == 0))
                    {
                        // untouched -> touched
                        if (TouchDownListenersTouchpad != null)
                            TouchDownListenersTouchpad ();
                    }

                    if (((state.BtnTouched & Input_Mask_Touchpad) == 0) && ((pre_state.BtnTouched & Input_Mask_Touchpad) != 0))
                    {
                        // touched -> touched
                        if (TouchUpListenersTouchpad != null)
                            TouchUpListenersTouchpad ();
                    }
                }
            }
        }

        IEnumerator PollingTouchState_Trigger()
        {
            while (true) {
                yield return waitForEndOfFrame;

                bool _connected = false;
                WVR_DeviceType _type = this.DeviceType;
                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                    _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                        _connected = _device.connected;
                        _type = _device.type;
                    }
                }

                bool _touched = false;

                var _triggervalue = state.BtnTouched & Input_Mask_Trigger;
                if (_triggervalue == 0)
                    pre_state.BtnTouched &= ~Input_Mask_Trigger;
                else
                    pre_state.BtnTouched |= _triggervalue;

                #if UNITY_EDITOR || UNITY_STANDALONE
                if (isEditorMode)
                {
                    var system = WaveVR_PoseSimulator.Instance;
                    _touched = system.GetButtonTouchState(_type, WVR_InputId.WVR_InputId_Alias1_Trigger);
                } else
                #endif
                {
                    if (WaveVR.Instance != null)
                    {
                        if (_connected)
                            _touched = Interop.WVR_GetInputTouchState (_type, WVR_InputId.WVR_InputId_Alias1_Trigger);
                    }
                }   // isEditorMode

                if (_touched)
                {
                    state.BtnTouched |= Input_Mask_Trigger;
                } else
                {
                    state.BtnTouched &= ~Input_Mask_Trigger;
                }

                if ((pre_state.BtnTouched ^ state.BtnTouched) != 0) // pre not equal to cur
                {
                    if (((state.BtnTouched & Input_Mask_Trigger) != 0) && ((pre_state.BtnTouched & Input_Mask_Trigger) == 0))
                    {
                        // untouched -> touched
                        if (TouchDownListenersTrigger != null)
                            TouchDownListenersTrigger ();
                    }

                    if (((state.BtnTouched & Input_Mask_Trigger) == 0) && ((pre_state.BtnTouched & Input_Mask_Trigger) != 0))
                    {
                        // touched -> untouched
                        if (TouchUpListenersTrigger != null)
                            TouchUpListenersTrigger ();
                    }
                }
            }
        }
        #endregion

        #region Button Axis
        public Vector2 GetAxis(WVR_InputId _id)
        {
            if (_id != WVR_InputId.WVR_InputId_Alias1_Touchpad && _id != WVR_InputId.WVR_InputId_Alias1_Trigger)
            {
                Log.e (LOG_TAG, "GetAxis, button " + _id + " does NOT have axis!");
                return Vector2.zero;
            }

            bool _connected = false;
            WVR_DeviceType _type = this.DeviceType;
            #if UNITY_EDITOR || UNITY_STANDALONE
            if (isEditorMode)
            {
                _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
            } else
            #endif
            {
                if (WaveVR.Instance != null)
                {
                    WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                    _connected = _device.connected;
                    _type = _device.type;
                }
            }

            #if UNITY_EDITOR || UNITY_STANDALONE
            if (isEditorMode)
            {
                var system = WaveVR_PoseSimulator.Instance;
                axis = system.GetAxis(_type, WVR_InputId.WVR_InputId_Alias1_Trigger);
            } else
            #endif
            {
                if (_connected)
                    axis = Interop.WVR_GetInputAnalogAxis (_type, _id);
            }

            //Log.d (LOG_TAG, "GetAxis: {" + axis.x + ", " + axis.y + "}");
            return new Vector2 (axis.x, axis.y);
        }
        #endregion

        public void TriggerHapticPulse(
            ushort _durationMicroSec = 500, 
            WVR_InputId _id = WVR_InputId.WVR_InputId_Alias1_Touchpad
        )
        {
            bool _connected = false;
            WVR_DeviceType _type = this.DeviceType;
            #if UNITY_EDITOR || UNITY_STANDALONE
            if (isEditorMode)
            {
                _connected = WaveVR_Controller.Input(this.DeviceType).connected;
                _type = WaveVR_Controller.Input(this.DeviceType).DeviceType;
            } else
            #endif
            {
                if (WaveVR.Instance != null)
                {
                    WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.DeviceType);
                    _connected = _device.connected;
                    _type = _device.type;
                }
            }

            #if UNITY_EDITOR || UNITY_STANDALONE
            if (isEditorMode)
            {
                var system = WaveVR_PoseSimulator.Instance;
                system.TriggerHapticPulse (_type, _id, _durationMicroSec);
            } else
            #endif
            {
                if (_connected)
                    Interop.WVR_TriggerVibrator (DeviceType, _id, _durationMicroSec);
            }
        }
    } // Device


    // Use this for initialization
	void Start ()
    {
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).ieConnection);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).iePressMenu);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).iePressGrip);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).iePressTouchpad);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).iePressTrigger);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).ieTouchTouchpad);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Left).ieTouchTrigger);

        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).ieConnection);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).iePressMenu);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).iePressGrip);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).iePressTouchpad);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).iePressTrigger);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).ieTouchTouchpad);
        StartCoroutine (Input (WVR_DeviceType.WVR_DeviceType_Controller_Right).ieTouchTrigger);
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
