using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;

public class WaveVR_PoseSimulator : MonoBehaviour
{
    public static WaveVR_PoseSimulator Instance
    {
        get
        {
            if (instance == null)
            {
                var gameObject = new GameObject("SimulatedController");
                instance = gameObject.AddComponent<WaveVR_PoseSimulator>();
                // This object should survive all scene transitions.
                GameObject.DontDestroyOnLoad(instance);
            }
            return instance;
        }
    }
    private static WaveVR_PoseSimulator instance = null;

    private WVR_DevicePosePair_t pose_head = new WVR_DevicePosePair_t();
    private WVR_DevicePosePair_t pose_right = new WVR_DevicePosePair_t();
    private WaveVR_Utils.RigidTransform rtPose_head = WaveVR_Utils.RigidTransform.identity;
    private WaveVR_Utils.RigidTransform rtPose_right = WaveVR_Utils.RigidTransform.identity;
    private WaveVR_Utils.WVR_ButtonState_t btn_right, btn_left;
    private WVR_Axis_t axis_right, axis_left;

    void OnEnable()
    {
        Cursor.visible = false;

        pose_head.type = WVR_DeviceType.WVR_DeviceType_HMD;
        pose_right.type = WVR_DeviceType.WVR_DeviceType_Controller_Right;
        pose_right.pose.AngularVelocity.v0 = 0.1f;
        pose_right.pose.AngularVelocity.v1 = 0.1f;
        pose_right.pose.AngularVelocity.v2 = 0.1f;
        pose_right.pose.Velocity.v0 = 0.1f;
        pose_right.pose.Velocity.v1 = 0.0f;
        pose_right.pose.Velocity.v2 = 0.0f;
    }

    public WVR_Vector3f_t GetVelocity(WVR_DeviceType type)
    {
        WVR_Vector3f_t velocity = new WVR_Vector3f_t();
        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            velocity = pose_right.pose.Velocity;
            break;
        default:
            break;
        }
        return velocity;
    }

    public WVR_Vector3f_t GetAngularVelocity(WVR_DeviceType type)
    {
        WVR_Vector3f_t av = new WVR_Vector3f_t();
        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            av = pose_right.pose.AngularVelocity;
            break;
        default:
            break;
        }
        return av;
    }

    public WaveVR_Utils.RigidTransform GetRigidTransform(WVR_DeviceType type)
    {
        WaveVR_Utils.RigidTransform _rt = WaveVR_Utils.RigidTransform.identity;
        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_HMD:
            _rt = rtPose_head;
            break;
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            _rt = rtPose_right;
            break;
        default:
            break;
        }
        return _rt;
    }

    public void GetTransform(WVR_DeviceType type, ref WVR_DevicePosePair_t pose, ref WaveVR_Utils.RigidTransform rtPose)
    {
        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_HMD:
            pose = pose_head;
            rtPose = rtPose_head;
            break;
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            pose = pose_right;
            rtPose = rtPose_right;
            break;
        default:
            break;
        }
    }

    private const ulong Input_Mask_Menu        = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Menu;
    private const ulong Input_Mask_Grip        = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Grip;
    private const ulong Input_Mask_Touchpad    = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Touchpad;
    private const ulong Input_Mask_Trigger     = 1UL << (int)WVR_InputId.WVR_InputId_Alias1_Trigger;

    public bool GetButtonPressState(WVR_DeviceType type, WVR_InputId button) 
    {
        bool _ret = false;

        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            switch (button)
            {
            case WVR_InputId.WVR_InputId_Alias1_Menu:
                if ((btn_right.BtnPressed & Input_Mask_Menu) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Grip:
                if ((btn_right.BtnPressed & Input_Mask_Grip) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Touchpad:
                if ((btn_right.BtnPressed & Input_Mask_Touchpad) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Trigger:
                if ((btn_right.BtnPressed & Input_Mask_Trigger) != 0)
                    _ret = true;
                break;
            default:
                break;
            }
            break;
        case WVR_DeviceType.WVR_DeviceType_Controller_Left:
            switch (button)
            {
            case WVR_InputId.WVR_InputId_Alias1_Menu:
                if ((btn_left.BtnPressed & Input_Mask_Menu) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Grip:
                if ((btn_left.BtnPressed & Input_Mask_Grip) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Touchpad:
                if ((btn_left.BtnPressed & Input_Mask_Touchpad) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Trigger:
                if ((btn_left.BtnPressed & Input_Mask_Trigger) != 0)
                    _ret = true;
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }

        return _ret;
    }

    public bool GetButtonTouchState(WVR_DeviceType type, WVR_InputId button)
    {
        bool _ret = false;

        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            switch (button)
            {
            case WVR_InputId.WVR_InputId_Alias1_Menu:
                if ((btn_right.BtnTouched & Input_Mask_Menu) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Grip:
                if ((btn_right.BtnTouched & Input_Mask_Grip) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Touchpad:
                if ((btn_right.BtnTouched & Input_Mask_Touchpad) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Trigger:
                if ((btn_right.BtnTouched & Input_Mask_Trigger) != 0)
                    _ret = true;
                break;
            default:
                break;
            }
            break;
        case WVR_DeviceType.WVR_DeviceType_Controller_Left:
            switch (button)
            {
            case WVR_InputId.WVR_InputId_Alias1_Menu:
                if ((btn_left.BtnTouched & Input_Mask_Menu) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Grip:
                if ((btn_left.BtnTouched & Input_Mask_Grip) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Touchpad:
                if ((btn_left.BtnTouched & Input_Mask_Touchpad) != 0)
                    _ret = true;
                break;
            case WVR_InputId.WVR_InputId_Alias1_Trigger:
                if ((btn_left.BtnTouched & Input_Mask_Trigger) != 0)
                    _ret = true;
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }

        return _ret;
    }

    public WVR_Axis_t GetAxis(WVR_DeviceType type, WVR_InputId button)
    {
        WVR_Axis_t _axis;
        _axis.x = 0;
        _axis.y = 0;

        switch (type)
        {
        case WVR_DeviceType.WVR_DeviceType_Controller_Left:
            _axis = axis_left;
            break;
        case WVR_DeviceType.WVR_DeviceType_Controller_Right:
            _axis = axis_right;
            break;
        default:
            break;
        }

        return _axis;
    }

    public void TriggerHapticPulse(WVR_DeviceType device, WVR_InputId id, ushort durationMicroSec)
    {

    }

    private const string MOUSE_X = "Mouse X";
    private const string MOUSE_Y = "Mouse Y";
    private const string MOUSE_SCROLLWHEEL = "Mouse ScrollWheel";

    // position of head
    private float shiftX_head = 0, shiftY_head = 0, shiftZ_head = 0;
    // rotation of head
    private float angleX_head = 0, angleY_head = 0, angleZ_head = 0;
    // position of right controller
    private float shiftX_right = 0, shiftY_right = 0, shiftZ_right = 0;
    // rotation of right controller
    private float angleX_right = 0, angleY_right = 0, angleZ_right = 0;

    private static readonly KeyCode[] KeyCode_Head = { KeyCode.LeftAlt, KeyCode.LeftControl, KeyCode.LeftShift };
    private static readonly KeyCode[] KeyCode_Right = {
        KeyCode.RightAlt, KeyCode.RightControl, KeyCode.RightShift,
        KeyCode.UpArrow, KeyCode.DownArrow, KeyCode.LeftArrow, KeyCode.RightArrow
    };

    private void UpdateHeadPose(float axis_x, float axis_y, float axis_z)
    {
        Vector3 _headPos = Vector3.zero;
        Quaternion _headRot = Quaternion.identity;

        if (Input.GetKey (KeyCode_Head [0]))
        {
            angleX_head -= axis_y * 2.4f;
            angleX_head = Mathf.Clamp (angleX_head, -89, 89);
            angleY_head += axis_x * 5;
            if (angleY_head <= -180)
            {
                angleY_head += 360;
            } else if (angleY_head > 180)
            {
                angleY_head -= 360;
            }
        }
        if (Input.GetKey (KeyCode_Head [1]))
        {
            angleZ_head += axis_x * 5;
            angleZ_head = Mathf.Clamp (angleZ_head, -89, 89);
        }
        _headRot = Quaternion.Euler (angleX_head, angleY_head, angleZ_head);

        if (Input.GetKey (KeyCode_Head [2]))
        {
            // Assume mouse can move move 150 pixels in a-half second. 
            // So we map from 150 pixels to 0.3 meter.

            Vector3 shift = _headRot * new Vector3 (axis_x / 5, axis_y / 5, axis_z);
            shiftX_head += shift.x;
            shiftY_head += shift.y;
            shiftZ_head += shift.z;
        }
        _headPos = new Vector3 (shiftX_head, shiftY_head, shiftZ_head);
        rtPose_head.update (_headPos, _headRot);
    }

    private void UpdateRightHandPose(float axis_x, float axis_y, float axis_z)
    {
        Vector3 _rightPos = Vector3.zero;
        Quaternion _rightRot = Quaternion.identity;

        //-------- mouse control ---------
        if (Input.GetKey (KeyCode_Right [0]))
        {
            angleY_right += axis_x / 2;
            angleX_right -= (float)(axis_y * 1.5f);
        }
        if (Input.GetKey (KeyCode_Right [1]))
        {
            angleZ_right += axis_z * 5;
        }
        if (Input.GetKey (KeyCode_Right [2]))
        {
            shiftX_right += axis_x / 5;
            shiftY_right += axis_y / 5;
            shiftZ_right += axis_z;
        }

        //-------- keyboard control ---------
        const float _speed = 10.0f;
        if (Input.GetKey (KeyCode_Right [3]))
        {
            shiftY_right += _speed * Time.deltaTime;
        }

        if (Input.GetKey (KeyCode_Right [4]))
        {
            shiftY_right -= _speed * Time.deltaTime;
        }

        if (Input.GetKey (KeyCode_Right [5]))
        {
            shiftX_right -= _speed * Time.deltaTime;
        }

        if (Input.GetKey (KeyCode_Right [6]))
        {
            shiftX_right += _speed * Time.deltaTime;
        }

        _rightPos = new Vector3 (shiftX_right, shiftY_right, shiftZ_right);
        _rightRot = Quaternion.Euler (angleX_right, angleY_right, angleZ_right);
        rtPose_right.update (_rightPos, _rightRot);
    }

    public void Update()
    {
        float axis_x = Input.GetAxis (MOUSE_X);
        float axis_y = Input.GetAxis (MOUSE_Y);
        float axis_z = Input.GetAxis (MOUSE_SCROLLWHEEL);

        UpdateHeadPose (axis_x, axis_y, axis_z);

        /* ------------- right controller begins ------------- */
        UpdateRightHandPose (axis_x, axis_y, axis_z);

        if (Input.GetMouseButtonDown(1))   // right mouse key
        {
            Debug.Log ("WaveVR_PoseSimulator, mouse right button down");
            btn_right.BtnPressed |= Input_Mask_Touchpad;
            btn_right.BtnPressed |= Input_Mask_Menu;
            btn_right.BtnPressed |= Input_Mask_Grip;
            btn_right.BtnPressed |= Input_Mask_Trigger;
            //btn_right.BtnTouched |= Input_Mask_Touchpad;
            //btn_right.BtnTouched |= Input_Mask_Menu;
            //btn_right.BtnTouched |= Input_Mask_Grip;
            //btn_right.BtnTouched |= Input_Mask_Trigger;
            axis_right.x = axis_x;
            axis_right.y = axis_y;
        }
        if (Input.GetMouseButtonUp(1))   // right mouse key
        {
            Debug.Log ("WaveVR_PoseSimulator, mouse right button up");
            btn_right.BtnPressed = 0x0;
            //btn_right.BtnTouched = 0x0;
        }
        /* ------------- right controller ends ------------- */

        /* ------------- left controller begins ------------- */
        if (Input.GetMouseButtonDown(0))        // left mouse key
        {
            Debug.Log ("WaveVR_PoseSimulator, mouse left button down");
            //btn_left.BtnPressed |= Input_Mask_Touchpad;
            //btn_left.BtnPressed |= Input_Mask_Menu;
            //btn_left.BtnPressed |= Input_Mask_Grip;
            //btn_left.BtnPressed |= Input_Mask_Trigger;
            btn_left.BtnTouched |= Input_Mask_Touchpad;
            btn_left.BtnTouched |= Input_Mask_Menu;
            btn_left.BtnTouched |= Input_Mask_Grip;
            btn_left.BtnTouched |= Input_Mask_Trigger;
            axis_left.x = axis_x;
            axis_left.y = axis_y;
        }
        if (Input.GetMouseButtonUp(0))     // left mouse key
        {
            Debug.Log ("WaveVR_PoseSimulator, mouse left button up");
            //btn_left.BtnPressed = 0x0;
            btn_left.BtnTouched = 0x0;
        }
        /* ------------- left controller ends ------------- */
    }
}
