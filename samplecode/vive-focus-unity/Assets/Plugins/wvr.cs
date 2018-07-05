// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using System;
using System.Runtime.InteropServices;

namespace wvr
{
    public class Interop
    {
        #region Interaction
        // ------------- wvr_events.h -------------
        // Events: swipe, battery status.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_PollEventQueue", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_PollEventQueue(ref WVR_Event_t e);

        // ------------- wvr_device.h -------------
        // Button types for which device is capable.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInputDeviceCapability", CallingConvention = CallingConvention.Cdecl)]
        public static extern int WVR_GetInputDeviceCapability(WVR_DeviceType type, WVR_InputType inputType);

        // Button press and touch state.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInputDeviceState", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_GetInputDeviceState(WVR_DeviceType type, uint inputMask, ref uint buttons, ref uint touches,
            [In, Out] WVR_AnalogState_t[] analogArray, uint analogArrayCount);

        // Count of specified button type.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInputTypeCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int WVR_GetInputTypeCount(WVR_DeviceType type, WVR_InputType inputType);

        // Button press state.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInputButtonState", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_GetInputButtonState(WVR_DeviceType type, WVR_InputId id);

        // Button touch state.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInputTouchState", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_GetInputTouchState(WVR_DeviceType type, WVR_InputId id);

        // Axis of analog button: touchpad (x, y), trigger (x only)
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInputAnalogAxis", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_Axis_t WVR_GetInputAnalogAxis(WVR_DeviceType type, WVR_InputId id);

        // Get transform of specified device.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetPoseState", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_GetPoseState(WVR_DeviceType type, WVR_PoseOriginModel originModel, uint predictedMilliSec, ref WVR_PoseState_t poseState);

        // Get all attributes of pose of all devices.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetSyncPose", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_GetSyncPose(WVR_PoseOriginModel originModel, [In, Out] WVR_DevicePosePair_t[] poseArray, uint pairArrayCount);

        // Device connection state.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_IsDeviceConnected", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_IsDeviceConnected(WVR_DeviceType type);

        // Make device vibrate.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_TriggerVibrator", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_TriggerVibrator(WVR_DeviceType type, WVR_InputId id, ushort durationMicroSec);

        // Recenter the scene.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_TriggerRecenter", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_TriggerRecenter();

        // ------------- wvr_arena.h -------------
        // Get current attributes of arena.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetArena", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_Arena_t WVR_GetArena();

        // Set up arena.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_SetArena", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_SetArena(ref WVR_Arena_t arena);

        // Get visibility type of arena.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetArenaVisible", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_ArenaVisible WVR_GetArenaVisible();

        // Set visibility type of arena.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_SetArenaVisible", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_SetArenaVisible(WVR_ArenaVisible config);

        // Check if player is over range of arena.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_IsOverArenaRange", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_IsOverArenaRange();

        // ------------- wvr_status.h -------------
        // Battery electricity (%).
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetDeviceBatteryPercentage", CallingConvention = CallingConvention.Cdecl)]
        public static extern float WVR_GetDeviceBatteryPercentage(WVR_DeviceType type);

        // Battery life status.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetBatteryStatus", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_BatteryStatus WVR_GetBatteryStatus(WVR_DeviceType type);

        // Battery is charging or not.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetChargeStatus", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_ChargeStatus WVR_GetChargeStatus(WVR_DeviceType type);

        // Whether battery is overheat.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetBatteryTemperatureStatus", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_BatteryTemperatureStatus WVR_GetBatteryTemperatureStatus(WVR_DeviceType type);

        // Battery temperature.
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetBatteryTemperature", CallingConvention = CallingConvention.Cdecl)]
        public static extern float WVR_GetBatteryTemperature(WVR_DeviceType type);

        #endregion

        // wvr.h
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_Init", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_InitError WVR_Init(WVR_AppType eType);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_Quit", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_Quit();

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetInitErrorString", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr WVR_GetInitErrorString(WVR_InitError error);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetWaveRuntimeVersion", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint WVR_GetWaveRuntimeVersion();

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetWaveSDKVersion", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint WVR_GetWaveSDKVersion();

        // wvr_system.h
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_IsInputFocusCapturedBySystem", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_IsInputFocusCapturedBySystem();

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_RenderInit", CallingConvention = CallingConvention.Cdecl)]
        internal static extern WVR_RenderError WVR_RenderInit(ref WVR_RenderInitParams_t param);

        // wvr_camera.h
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_StartCamera", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_StartCamera(ref WVR_CameraInfo_t info);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_StopCamera", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_StopCamera();

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_UpdateTexture", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_UpdateTexture(uint textureid );

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetCameraIntrinsic", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_GetCameraIntrinsic(WVR_CameraPosition position, ref WVR_CameraIntrinsic_t intrinsic);

        // wvr_device.h
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_IsDeviceSuspend", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_IsDeviceSuspend(WVR_DeviceType type);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_ConvertMatrixQuaternion", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_ConvertMatrixQuaternion(ref WVR_Matrix4f_t mat, ref WVR_Quatf_t quat, bool m2q);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetDegreeOfFreedom", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_NumDoF WVR_GetDegreeOfFreedom(WVR_DeviceType type);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_SetParameters", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_SetParameters(WVR_DeviceType type, IntPtr pchValue);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetParameters", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint WVR_GetParameters(WVR_DeviceType type, IntPtr pchValue, IntPtr retValue, uint unBufferSize);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetDefaultControllerRole", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_DeviceType WVR_GetDefaultControllerRole();

        // TODO
        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetRenderTargetSize", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_GetRenderTargetSize(ref uint width, ref uint height);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetProjection", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_Matrix4f_t WVR_GetProjection(WVR_Eye eye, float near, float far);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetClippingPlaneBoundary", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_GetClippingPlaneBoundary(WVR_Eye eye, ref float left, ref float right, ref float top, ref float bottom);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetTransformFromEyeToHead", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_Matrix4f_t  WVR_GetTransformFromEyeToHead(WVR_Eye eye, WVR_NumDoF dof);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_SubmitFrame", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_SubmitError WVR_SubmitFrame(WVR_Eye eye, ref WVR_TextureParams_t param, ref WVR_PoseState_t pose, WVR_SubmitExtend extendMethod);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_RequestScreenshot", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_RequestScreenshot(uint width, uint height, WVR_ScreenshotMode mode, IntPtr filename);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_RenderMask", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_RenderMask(WVR_Eye eye);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetRenderProps", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WVR_GetRenderProps(ref WVR_RenderProps_t props);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_ObtainTextureQueue", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr WVR_ObtainTextureQueue(WVR_TextureTarget target, WVR_TextureFormat format, WVR_TextureType type, uint width, uint height, int level);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetTextureQueueLength", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint WVR_GetTextureQueueLength(IntPtr handle);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetTexture", CallingConvention = CallingConvention.Cdecl)]
        public static extern WVR_TextureParams_t WVR_GetTexture(IntPtr handle, int index);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetAvailableTextureIndex", CallingConvention = CallingConvention.Cdecl)]
        public static extern int WVR_GetAvailableTextureIndex(IntPtr handle);

        [DllImportAttribute("wvr_api", EntryPoint = "WVR_ReleaseTextureQueue", CallingConvention = CallingConvention.Cdecl)]
        public static extern void WVR_ReleaseTextureQueue(IntPtr handle);
    }

    public enum WVR_AppType
    {
        WVR_AppType_VRContent       = 1,
        WVR_AppType_NonVRContent    = 2,
    }

    public enum WVR_InitError
    {
        WVR_InitError_None           = 0,
        WVR_InitError_Unknown        = 1,
        WVR_InitError_NotInitialized = 2,
    }

    public enum WVR_EventType
    {
        WVR_EventType_Quit                               = 99,

        /* Device events region */
        WVR_EventType_DeviceConnected                    = 100,
        WVR_EventType_DeviceDisconnected                 = 101,
        WVR_EventType_DeviceStatusUpdate                 = 102,   // Occur when DeviceService's device configure change
        WVR_EventType_IpdChanged                         = 105,   // HMD only
        WVR_EventType_DeviceSuspend                      = 106,   // HMD only
        WVR_EventType_DeviceResume                       = 107,   // HMD only
        WVR_EventType_DeviceRoleChanged                  = 108,
        WVR_EventType_BatteryStatus_Update               = 30004,
        WVR_EventType_ChargeStatus_Update                = 30005, // HMD
        WVR_EventType_DeviceErrorStatus_Update           = 30006,
        WVR_EventType_BatteryTemperatureStatus_Update    = 30007,
        WVR_EventType_RecenterSuccess                    = 30010,
        WVR_EventType_RecenterFail                       = 30011,
        WVR_EventType_RecenterSuccess_3DoF               = 30012,
        WVR_EventType_RecenterFail_3DoF                  = 30013,
        WVR_EventType_TouchpadSwipe_LeftToRight          = 30014,
        WVR_EventType_TouchpadSwipe_RightToLeft          = 30015,
        WVR_EventType_TouchpadSwipe_DownToUp             = 30016,
        WVR_EventType_TouchpadSwipe_UpToDown             = 30017,
        WVR_EventType_Settings_Controller                = 30022,
        WVR_EventType_TrackingModeChanged                = 30106,

        /* Input Button Event region */
        WVR_EventType_ButtonPressed                      = 200,
        WVR_EventType_ButtonUnpressed                    = 201,
        /* Input Touch Event region  */
        WVR_EventType_TouchTapped                        = 202,
        WVR_EventType_TouchUntapped                      = 203,

    }

    public enum WVR_DeviceType
    {
        WVR_DeviceType_HMD                          = 1,
        WVR_DeviceType_Controller_Right             = 2,
        WVR_DeviceType_Controller_Left              = 3,
    };

    public enum WVR_InputType
    {
        WVR_InputType_Button = 1<<0,
        WVR_InputType_Touch  = 1<<1,
        WVR_InputType_Analog = 1<<2,
    };

    public enum WVR_BatteryStatus
    {
        WVR_BatteryStatus_Unknown  = 0,
        WVR_BatteryStatus_Normal   = 1,
        WVR_BatteryStatus_Low      = 2, //  5% <= Battery  < 15%
        WVR_BatteryStatus_UltraLow = 3, //  Battery < 5%
    }

    public enum WVR_ChargeStatus
    {
        WVR_ChargeStatus_Unknown      = 0,
        WVR_ChargeStatus_Discharging  = 1,
        WVR_ChargeStatus_Charging     = 2,
        WVR_ChargeStatus_Full         = 3,
    }

    public enum WVR_BatteryTemperatureStatus
    {
        WVR_BatteryTemperature_Unknown       = 0,
        WVR_BatteryTemperature_Normal        = 1,
        WVR_BatteryTemperature_Overheat      = 2,
        WVR_BatteryTemperature_UltraOverheat = 3,
    }

    public enum WVR_DeviceErrorStatus
    {
        WVR_DeviceErrorStatus_None                       = 0,
        WVR_DeviceErrorStatus_BatteryOverheat            = 1,
        WVR_DeviceErrorStatus_BatteryOverheatRestore     = 1 << 1,
        WVR_DeviceErrorStatus_BatteryOvervoltage         = 1 << 2,
        WVR_DeviceErrorStatus_BatteryOvervoltageRestore  = 1 << 3,
        WVR_DeviceErrorStatus_DeviceConnectFail          = 1 << 4,
        WVR_DeviceErrorStatus_DeviceConnectRestore       = 1 << 5,
        WVR_DeviceErrorStatus_DeviceLostTracking         = 1 << 6,
        WVR_DeviceErrorStatus_DeviceLostTrackingRestore  = 1 << 7,
        WVR_DeviceErrorStatus_ChargeFail                 = 1 << 8,
        WVR_DeviceErrorStatus_ChargeRestore              = 1 << 9,
    }

    public enum WVR_InputId
    {
        WVR_InputId_0     = 0,
        WVR_InputId_1     = 1,
        WVR_InputId_2     = 2,
        WVR_InputId_3     = 3,
        WVR_InputId_4     = 4,
        WVR_InputId_5     = 5,
        WVR_InputId_6     = 6,
        WVR_InputId_7     = 7,
        WVR_InputId_8     = 8,
        WVR_InputId_9     = 9,

        WVR_InputId_16    = 16,
        WVR_InputId_17    = 17,

        //alias group mapping
        WVR_InputId_Alias1_System      = WVR_InputId_0,
        WVR_InputId_Alias1_Menu        = WVR_InputId_1,
        WVR_InputId_Alias1_Grip        = WVR_InputId_2,
        WVR_InputId_Alias1_DPad_Left   = WVR_InputId_3,
        WVR_InputId_Alias1_DPad_Up     = WVR_InputId_4,
        WVR_InputId_Alias1_DPad_Right  = WVR_InputId_5,
        WVR_InputId_Alias1_DPad_Down   = WVR_InputId_6,
        WVR_InputId_Alias1_Volume_Up   = WVR_InputId_7,
        WVR_InputId_Alias1_Volume_Down = WVR_InputId_8,
        WVR_InputId_Alias1_Bumper      = WVR_InputId_9,

        WVR_InputId_Alias1_Touchpad    = WVR_InputId_16,
        WVR_InputId_Alias1_Trigger     = WVR_InputId_17,

        WVR_InputId_Max   = 32,
    }

    public enum WVR_AnalogType
    {
        WVR_AnalogType_None     = 0,
        WVR_AnalogType_TouchPad = 1,
        WVR_AnalogType_Trigger  = 2,
    }

    public enum WVR_PoseOriginModel
    {
        WVR_PoseOriginModel_OriginOnHead             = 0,
        WVR_PoseOriginModel_OriginOnGround           = 1,
        WVR_PoseOriginModel_OriginOnTrackingObserver = 2,
        WVR_PoseOriginModel_OriginOnHead_3DoF        = 3,
    }

    public enum WVR_ArenaVisible
    {
        WVR_ArenaVisible_Auto     = 0,  // show Arena while HMD out off bounds
        WVR_ArenaVisible_ForceOn  = 1,  // always show Arena
        WVR_ArenaVisible_ForceOff = 2,  // never show Arena
    }

    public enum WVR_GraphicsApiType
    {
        WVR_GraphicsApiType_OpenGL = 1,
    }

    public enum WVR_ScreenshotMode
    {
        WVR_ScreenshotMode_Default,      /**< Screenshot image is stereo. Just as show on screen*/
        WVR_ScreenshotMode_Raw,          /**< Screenshot image has only single eye, and without distortion correction*/
    }

    public enum WVR_SubmitError{
        WVR_SubmitError_None                        = 0,
        WVR_SubmitError_InvalidTexture              = 400,
        WVR_SubmitError_ThreadStop                  = 401,
        WVR_SubmitError_BufferSubmitFailed          = 402,
        WVR_SubmitError_Max                         = 65535
    }

    public enum WVR_SubmitExtend{
        WVR_SubmitExtend_Default = 0x00,
    }

    public enum WVR_Eye{
        WVR_Eye_Left = 0,
        WVR_Eye_Right = 1,
    }

    public enum WVR_TextureTarget
    {
        WVR_TextureTarget_2D,
        WVR_TextureTarget_2D_ARRAY
    }

    public enum WVR_TextureFormat
    {
        WVR_TextureFormat_RGBA
    }

    public enum WVR_TextureType
    {
        WVR_TextureType_UnsignedByte
    }

    public enum WVR_RenderError{
        WVR_RenderError_None                        = 0,
        WVR_RenderError_RuntimeInitFailed           = 410,
        WVR_RenderError_ContextSetupFailed          = 411,
        WVR_RenderError_DisplaySetupFailed          = 412,
        WVR_RenderError_LibNotSupported             = 413,
        WVR_RenderError_NullPtr                     = 414,
        WVR_RenderError_Max                         = 65535
    }

    public enum WVR_RenderConfig{
        WVR_RenderConfig_Default                        = 0x0000,
        WVR_RenderConfig_Direct_Mode                    = 0x0001,
        WVR_RenderConfig_MSAA                           = 0x0002,
        WVR_RenderConfig_Vertical_Sync                  = 0x0010,
        WVR_RenderConfig_Timewarp                       = 0x0100,
        WVR_RenderConfig_Timewarp_Asynchronous          = 0x0300
    }

    public enum WVR_CameraImageType
    {
        WVR_CameraImageType_Invalid   = 0,
        WVR_CameraImageType_SingleEye = 1,     // the image is comprised of one camera
        WVR_CameraImageType_DualEye   = 2,     // the image is comprised of dual cameras
    }

    public enum WVR_CameraImageFormat
    {
        WVR_CameraImageFormat_Invalid     = 0,
        WVR_CameraImageFormat_YUV_420     = 1, // the image format is YUV420
        WVR_CameraImageFormat_Grayscale   = 2, // the image format is 8-bit gray-scale
    }

    public enum WVR_CameraPosition
    {
        WVR_CameraPosition_Invalid   = 0,
        WVR_CameraPosition_left      = 1,
        WVR_CameraPosition_Right     = 2,
    }

    public enum WVR_OverlayError
    {
        WVR_OverlayError_None               = 0,
        WVR_OverlayError_UnknownOverlay     = 10,
        WVR_OverlayError_OverlayUnavailable = 11,
        WVR_OverlayError_InvalidParameter   = 20,
    }

    public enum WVR_OverlayTransformType
    {
        WVR_OverlayTransformType_None,
        WVR_OverlayTransformType_Absolute,
        WVR_OverlayTransformType_Fixed,
    }

    public enum WVR_NumDoF
    {
        WVR_NumDoF_3DoF = 0,
        WVR_NumDoF_6DoF = 1,
    }

    public enum WVR_ArenaShape
    {
        WVR_ArenaShape_None 	 = 0,
        WVR_ArenaShape_Rectangle = 1,
        WVR_ArenaShape_Round	 = 2,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_RenderInitParams_t {
        public WVR_GraphicsApiType graphicsApi;
            public UInt64 renderConfig;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Matrix4f_t {
        public float m0; //float[4][4]
	    public float m1;
	    public float m2;
	    public float m3;
	    public float m4;
	    public float m5;
	    public float m6;
	    public float m7;
	    public float m8;
	    public float m9;
	    public float m10;
	    public float m11;
	    public float m12;
	    public float m13;
	    public float m14;
	    public float m15;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Vector2f_t
    {
        public float v0;
        public float v1;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Vector3f_t
    {
        public float v0;  // float[3]
        public float v1;
        public float v2;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_CameraIntrinsic_t {
        public WVR_Vector2f_t    focalLength;
        public WVR_Vector2f_t    principalPoint;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_CameraInfo_t {
        public WVR_CameraImageType   imgType;    // SINGLE OR STEREO image
        public WVR_CameraImageFormat imgFormat;
        public uint              width;
        public uint              height;
        public uint              size;       // The buffer size for raw image data
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Quatf_t
    {
	      public float w;
	      public float x;
	      public float y;
	      public float z;
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct WVR_PoseState_t
    {
        [FieldOffset(0)] public bool IsValidPose;
        [FieldOffset(4)]  public WVR_Matrix4f_t PoseMatrix;
        [FieldOffset(68)] public WVR_Vector3f_t Velocity;
        [FieldOffset(80)] public WVR_Vector3f_t AngularVelocity;
        [FieldOffset(92)] public bool Is6DoFPose;
        [FieldOffset(96)] public long PoseTimestamp_ns;
        [FieldOffset(104)] public WVR_Vector3f_t Acceleration;
        [FieldOffset(116)] public WVR_Vector3f_t AngularAcceleration;
        [FieldOffset(128)] public float PredictedMilliSec;
        [FieldOffset(132)] public WVR_PoseOriginModel OriginModel;
    }

    [StructLayout(LayoutKind. Explicit)]
    public struct WVR_DevicePosePair_t
    {
        [FieldOffset(0)] public WVR_DeviceType type;
        [FieldOffset(8)] public WVR_PoseState_t pose;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Hexagon_t
    {
        public WVR_Vector3f_t vCorners0;  // WVR_Vector3f_t[4]
        public WVR_Vector3f_t vCorners1;
        public WVR_Vector3f_t vCorners2;
        public WVR_Vector3f_t vCorners3;
        public WVR_Vector3f_t vCorners4;
        public WVR_Vector3f_t vCorners5;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_TextureParams_t
    {
        public IntPtr id;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_RenderProps_t {
        public float refreshRate;
        public bool hasExternal;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_CommonEvent_t
    {
        public WVR_EventType type;
        public long timestamp;         // Delivered time in nanoseconds
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_DeviceEvent_t
    {
        public WVR_CommonEvent_t common;
        public WVR_DeviceType    type;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_InputEvent_t
    {
        public WVR_DeviceEvent_t device;
        public WVR_InputId      inputId;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_ErrorStatusEvent_t
    {
        public WVR_DeviceEvent_t     device;
        public WVR_DeviceErrorStatus status;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_IPDEvent_t
    {
        public WVR_CommonEvent_t common;
        public float ipdMeters;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_TrackingModeEvent_t
    {
        public WVR_CommonEvent_t common;
        public WVR_NumDoF  trackingMode;
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct WVR_Event_t
    {
        [FieldOffset(0)] public WVR_CommonEvent_t       common;
        [FieldOffset(0)] public WVR_DeviceEvent_t       device;
        [FieldOffset(0)] public WVR_InputEvent_t        input;
        [FieldOffset(0)] public WVR_ErrorStatusEvent_t  error;
        [FieldOffset(0)] public WVR_IPDEvent_t          ipd;
        [FieldOffset(0)] public WVR_TrackingModeEvent_t trmode;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Axis_t
    {
        public float x;
        public float y;
    }

[StructLayout(LayoutKind.Explicit)]
    public struct WVR_AnalogState_t
    {
        [FieldOffset(0)] public WVR_InputId     id;
        [FieldOffset(4)] public WVR_AnalogType  type;
        [FieldOffset(8)] public WVR_Axis_t      axis;
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct WVR_Pose_t
    {
        [FieldOffset(0)]  public WVR_Vector3f_t position;
        [FieldOffset(12)] public WVR_Quatf_t rotation;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_OverlayPosition_t {
        public float x;
        public float y;
        public float z;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_OverlayBlendColor_t {
        public float r;
        public float g;
        public float b;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_OverlayTexture_t {
        public uint textureId;
        public uint width;
        public uint height;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_ArenaRectangle_t {
        public float width;
        public float length;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_ArenaRound_t {
        public float diameter;
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct WVR_ArenaArea_t {
        [FieldOffset(0)] public WVR_ArenaRectangle_t rectangle;
        [FieldOffset(0)] public WVR_ArenaRound_t round;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_Arena_t {
        public WVR_ArenaShape shape;
        public WVR_ArenaArea_t area;
    }

    public delegate void WVR_OverlayInputEventCallback(int overlayId, WVR_EventType type, WVR_InputId inputId);
    [StructLayout(LayoutKind.Sequential)]
    public struct WVR_OverlayInputEvent_t {
        public int overlayId;
        public IntPtr callback;
    }

} // namespace wvr
