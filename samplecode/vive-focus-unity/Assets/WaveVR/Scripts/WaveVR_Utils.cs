// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

#define SYSTRACE_NATIVE  // Systrace in native support multi-thread rendering.
using UnityEngine;
using System.Collections;
using wvr;
using System.Runtime.InteropServices;
using WaveVR_Log;
using AOT;

/// This class is mainly for common handling:
/// including event handling and pose data handling.
public static class WaveVR_Utils
{
    public static string LOG_TAG = "WVR_Utils";

    #region Strings
    [System.Obsolete("Use same variable in Event instead.")]
    public static string DEVICE_CONNECTED = "device_connected";
    [System.Obsolete("Use same variable in Event instead.")]
    public static string NEW_POSES = "new_poses";
    [System.Obsolete("Use same variable in Event instead.")]
    public static string AFTER_NEW_POSES = "after_new_poses";
    #endregion

    public enum DegreeField
    {
        DOF3,
        DOF6
    }

    public struct WVR_ButtonState_t
    {
        public ulong BtnPressed;
        public ulong BtnTouched;
    }

    public class OEMConfig
    {
        private const string OEM_CONFIG_CLASSNAME = "com.htc.vr.unity.OEMConfig";
        private static AndroidJavaObject mOEMConfig = null;

        private static void initAJC()
        {
            if (mOEMConfig == null)
            {
                AndroidJavaClass ajc = new AndroidJavaClass(OEM_CONFIG_CLASSNAME);

                if (ajc == null)
                {
                    Log.e(LOG_TAG, "AndroidJavaClass is null");
                    return;
                }
                // Get the OEMConfig object
                mOEMConfig = ajc.CallStatic<AndroidJavaObject>("getInstance");
            }
        }

        public static string getControllerConfig()
        {
#if UNITY_EDITOR
            if (Application.isPlaying)
                return "";
#endif
                initAJC();
            string getString = "";
            if (mOEMConfig != null)
            {
                getString = mOEMConfig.Call<string>("getJsonRawData");
                const int charPerLine = 200;
                int len = (getString.Length / charPerLine);

                Log.d(LOG_TAG, "len = " + len + ", length of string = " + getString.Length);
                Log.d(LOG_TAG, "JSON raw data = ");

                for (int i=0; i<len; i++)
                {
                    string substr = getString.Substring(i * charPerLine, charPerLine);
                    Log.d(LOG_TAG, substr);
                }

                int remainLen = getString.Length - (len * charPerLine);
                string remainstr = getString.Substring(len * charPerLine, remainLen);
                Log.d(LOG_TAG, remainstr);
            }

            return getString;
        }
    }

    public class Event
    {
        public static string DEVICE_CONNECTED = "device_connected";
        public static string NEW_POSES = "new_poses";
        public static string AFTER_NEW_POSES = "after_new_poses";
        public static string ALL_VREVENT = "all_vrevent";  // Called when had event from WVR_PollEventQueue()
        public static string BATTERY_STATUS_UPDATE = "BatteryStatus_Update";
        public static string CONTROLLER_MODEL_LOADED = "controller_model_loaded";
        public static string CONTROLLER_MODEL_UNLOADED = "controller_model_unloaded";
        public static string RENDER_OBJECT_LEFT = "Render_left";
        public static string RENDER_OBJECT_RIGHT = "Render_right";
        public static string PRE_RENDER_OBJECT_LEFT = "Pre_Render_left";
        public static string PRE_RENDER_OBJECT_RIGHT = "Pre_Render_right";


        public delegate void Handler(params object[] args);

        public static void Listen(string message, Handler action)
        {
            var actions = listeners[message] as Handler;
            if (actions != null)
            {
                listeners[message] = actions + action;
            }
            else
            {
                listeners[message] = action;
            }
        }

        public static void Remove(string message, Handler action)
        {
            var actions = listeners[message] as Handler;
            if (actions != null)
            {
                listeners[message] = actions - action;
            }
        }

        public static void Send(string message, params object[] args)
        {
            var actions = listeners[message] as Handler;
            if (actions != null)
            {
                actions(args);
            }
        }

        private static Hashtable listeners = new Hashtable();
    }

    private static float _copysign(float sizeval, float signval)
    {
        return Mathf.Sign(signval) == 1 ? Mathf.Abs(sizeval) : -Mathf.Abs(sizeval);
    }

    public static Quaternion GetRotation(this Matrix4x4 matrix)
    {
        float tr = matrix.m00 + matrix.m11 + matrix.m22;
        float qw, qx, qy, qz;
        if (tr > 0) {
            float S = Mathf.Sqrt(tr + 1.0f) * 2; // S=4*qw
            qw = 0.25f * S;
            qx = (matrix.m21 - matrix.m12) / S;
            qy = (matrix.m02 - matrix.m20) / S;
            qz = (matrix.m10 - matrix.m01) / S;
        } else if ((matrix.m00 > matrix.m11) & (matrix.m00 > matrix.m22)) {
            float S = Mathf.Sqrt(1.0f + matrix.m00 - matrix.m11 - matrix.m22) * 2; // S=4*qx
            qw = (matrix.m21 - matrix.m12) / S;
            qx = 0.25f * S;
            qy = (matrix.m01 + matrix.m10) / S;
            qz = (matrix.m02 + matrix.m20) / S;
        } else if (matrix.m11 > matrix.m22) {
            float S = Mathf.Sqrt(1.0f + matrix.m11 - matrix.m00 - matrix.m22) * 2; // S=4*qy
            qw = (matrix.m02 - matrix.m20) / S;
            qx = (matrix.m01 + matrix.m10) / S;
            qy = 0.25f * S;
            qz = (matrix.m12 + matrix.m21) / S;
        } else {
            float S = Mathf.Sqrt(1.0f + matrix.m22 - matrix.m00 - matrix.m11) * 2; // S=4*qz
            qw = (matrix.m10 - matrix.m01) / S;
            qx = (matrix.m02 + matrix.m20) / S;
            qy = (matrix.m12 + matrix.m21) / S;
            qz = 0.25f * S;
        }
        return new Quaternion(qx, qy, qz, qw);
    }

    [System.Obsolete("Do NOT used any more.")]
    public static Quaternion GetRotation2(this Matrix4x4 matrix)
    {
        float qw = 0, qx = 0, qy = 0, qz = 0;
        return new Quaternion(qx, qy, qz, qw);
    }

    public static Vector3 GetPosition(this Matrix4x4 matrix)
    {
        var x = matrix.m03;
        var y = matrix.m13;
        var z = matrix.m23;

        return new Vector3(x, y, z);
    }

    // get new position and rotation from new pose
    [System.Serializable]
    public struct RigidTransform
    {
        public Vector3 pos;
        public Quaternion rot;

        public static RigidTransform identity
        {
            get { return new RigidTransform(Vector3.zero, Quaternion.identity); }
        }

        public RigidTransform(Vector3 pos, Quaternion rot)
        {
            this.pos = pos;
            this.rot = rot;
        }

        public RigidTransform(Transform t)
        {
            this.pos = t.position;
            this.rot = t.rotation;
        }

        public RigidTransform(WVR_Matrix4f_t pose)
        {
            var m = toMatrix44(pose);
            this.pos = m.GetPosition();
            this.rot = m.GetRotation();
        }

        public static Matrix4x4 toMatrix44(WVR_Matrix4f_t pose)
        {
            var m = Matrix4x4.identity;

            m[0, 0] = pose.m0;
            m[0, 1] = pose.m1;
            m[0, 2] = -pose.m2;
            m[0, 3] = pose.m3;

            m[1, 0] = pose.m4;
            m[1, 1] = pose.m5;
            m[1, 2] = -pose.m6;
            m[1, 3] = pose.m7;

            m[2, 0] = -pose.m8;
            m[2, 1] = -pose.m9;
            m[2, 2] = pose.m10;
            m[2, 3] = -pose.m11;

            return m;
        }

        public void update(WVR_Matrix4f_t pose)
        {
            var m = toMatrix44(pose);
            this.pos = m.GetPosition();
            this.rot = m.GetRotation();
        }

        public void update(Vector3 position, Quaternion orientation)
        {
            this.pos = position;
            this.rot = orientation;
        }

        public override bool Equals(object o)
        {
            if (o is RigidTransform)
            {
                RigidTransform t = (RigidTransform)o;
                return pos == t.pos && rot == t.rot;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return pos.GetHashCode() ^ rot.GetHashCode();
        }

        public static bool operator ==(RigidTransform a, RigidTransform b)
        {
            return a.pos == b.pos && a.rot == b.rot;
        }

        public static bool operator !=(RigidTransform a, RigidTransform b)
        {
            return a.pos != b.pos || a.rot != b.rot;
        }

        public static RigidTransform operator *(RigidTransform a, RigidTransform b)
        {
            return new RigidTransform
            {
                rot = a.rot * b.rot,
                pos = a.pos + a.rot * b.pos
            };
        }

        public void Inverse()
        {
            rot = Quaternion.Inverse(rot);
            pos = -(rot * pos);
        }

        public RigidTransform GetInverse()
        {
            var t = new RigidTransform(pos, rot);
            t.Inverse();
            return t;
        }

        public Vector3 TransformPoint(Vector3 point)
        {
            return pos + (rot * point);
        }

        public static Vector3 operator *(RigidTransform t, Vector3 v)
        {
            return t.TransformPoint(v);
        }

    }

#if SYSTRACE_NATIVE
    public static Queue TraceSessionNameQueue = new Queue(5);
#else
    public static AndroidJavaObject trace = new AndroidJavaObject("android.os.Trace");
#endif

    public class Trace {
        public static void BeginSection(string sectionName, bool inRenderThread = true)
        {
#if !UNITY_EDITOR && UNITY_ANDROID
#if SYSTRACE_NATIVE
            if (inRenderThread) {
                lock (TraceSessionNameQueue)
                {
                    TraceSessionNameQueue.Enqueue(sectionName);
                }
                SendRenderEvent(RENDEREVENTID_Systrace_BeginSession);
            } else {
                TraceBeginSection(sectionName);
            }
#else
            trace.CallStatic("beginSection", sectionName);
#endif
#endif
        }

        public static void EndSection(bool inRenderThread = true)
        {
#if !UNITY_EDITOR && UNITY_ANDROID
#if SYSTRACE_NATIVE
            if (inRenderThread) {
                SendRenderEvent(RENDEREVENTID_Systrace_EndSession);
            } else {
                TraceEndSection();
            }
#else
            trace.CallStatic("endSection");
#endif
#endif
        }
    }

    public static void notifyActivityUnityStarted()
    {
        AndroidJavaClass clazz = new AndroidJavaClass("com.htc.vr.unity.WVRUnityVRActivity");
        AndroidJavaObject activity = clazz.CallStatic<AndroidJavaObject>("getInstance");
        activity.Call("onUnityStarted");
    }

    public const int k_nRenderEventID_SubmitL = 1;
    public const int k_nRenderEventID_SubmitR = 2;
    public const int k_nRenderEventID_GraphicInitial = 8;
    public const int k_nRenderEventID_RenderEyeL = 0x100;
    public const int k_nRenderEventID_RenderEyeR = 0x101;
    public const int k_nRenderEventID_RenderEyeEndL = 0x102;
    public const int k_nRenderEventID_RenderEyeEndR = 0x103;

    [DllImportAttribute("wvrunity", CallingConvention = CallingConvention.Cdecl)]
    public static extern System.IntPtr GetRenderEventFunc();

    [DllImportAttribute("wvrunity", CallingConvention = CallingConvention.Cdecl)]
    public static extern void NativeRenderEvent(int eventID);

    [DllImportAttribute("wvrunity", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetCurrentRenderTexture(System.IntPtr current);

    // This api does not guarantee these argument are used by current frame.  Use it if you know what will happen.
    [DllImportAttribute("wvrunity", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetSubmitOptionalArgument([Out] WVR_PoseState_t [] poses, int submit_extend_flag);

    [DllImportAttribute("wvrunity", EntryPoint = "nativeProcessEngineEvent", CallingConvention = CallingConvention.Cdecl)]
    public static extern void NativeProcessEngineEvent(uint tID, uint eventID);

#if SYSTRACE_NATIVE
    [DllImportAttribute("wvrunity", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private static extern void TraceBeginSection(string name);

    [DllImportAttribute("wvrunity", CallingConvention = CallingConvention.Cdecl)]
    private static extern void TraceEndSection();
#endif

    [DllImportAttribute("wvr_api", EntryPoint = "WVR_IsATWActive", CallingConvention = CallingConvention.Cdecl)]
    public static extern bool WVR_IsATWActive();

    [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetNumberOfTextures", CallingConvention = CallingConvention.Cdecl)]
    public static extern int WVR_GetNumberOfTextures();

    [DllImportAttribute("wvr_api", EntryPoint = "WVR_StoreRenderTextures", CallingConvention = CallingConvention.Cdecl)]
    public static extern System.IntPtr WVR_StoreRenderTextures(System.IntPtr[] texturesIDs, int size, bool eEye);

    [DllImportAttribute("wvr_api", EntryPoint = "WVR_GetAvailableTextureID", CallingConvention = CallingConvention.Cdecl)]
    public static extern uint WVR_GetAvailableTextureID(System.IntPtr queue);

    [DllImportAttribute("wvr_api", EntryPoint = "WVR_IsPresentedOnExternalD", CallingConvention = CallingConvention.Cdecl)]
    public static extern bool WVR_IsPresentedOnExternal();

    public const int RENDEREVENTID_INIT_GRAPHIC = 0;
    public const int RENDEREVENTID_Systrace_BeginSession = 4;
    public const int RENDEREVENTID_Systrace_EndSession = 5;
    public const int RENDEREVENTID_StartCamera = 21;
    public const int RENDEREVENTID_StopCamera = 22;
    public const int RENDEREVENTID_UpdateCamera = 23;

    [MonoPInvokeCallback(typeof(RenderEventDelegate))]
    private static void RenderEvent(int eventId)
    {
        if ((eventId & (int)EngineEventID.ENGINE_EVENT_ID_BEGIN) == (int) EngineEventID.ENGINE_EVENT_ID_BEGIN)
        {
            NativeProcessEngineEvent((uint) EngineThreadID.RENDER_THREAD, (uint)eventId);
            return;
        }
        
        switch (eventId)
        {
            case RENDEREVENTID_INIT_GRAPHIC:
                // Use native code to initial compositor then get the c# instance later.
                NativeRenderEvent(k_nRenderEventID_GraphicInitial);
                break;
            case RENDEREVENTID_Systrace_BeginSession:
                string sectionName;
                lock (TraceSessionNameQueue)
                {
                    try
                    {
                        sectionName = (string)TraceSessionNameQueue.Dequeue();
                    }
                    catch (System.InvalidOperationException)
                    {
                        sectionName = "Empty";
                    }
                }
                TraceBeginSection(sectionName);
                break;
            case RENDEREVENTID_Systrace_EndSession:
                TraceEndSection();
                break;
            case RENDEREVENTID_StartCamera:
                {
                    WVR_CameraInfo_t camerainfo = new WVR_CameraInfo_t();
                    var result = Interop.WVR_StartCamera(ref camerainfo);

                    WaveVR_Utils.Event.Send("StartCameraCompleted", result, camerainfo);
                }

                break;
            case RENDEREVENTID_StopCamera:
                {
                    Interop.WVR_StopCamera();
                }

                break;
            case RENDEREVENTID_UpdateCamera:
                {
                    var updated = Interop.WVR_UpdateTexture(WaveVR_CameraTexture.instance.getNativeTextureId());
                    WaveVR_Utils.Event.Send("UpdateCameraCompleted", updated);
                }

                break;
        }
    }

    private delegate void RenderEventDelegate(int e);
    private static RenderEventDelegate RenderEventHandle = new RenderEventDelegate(RenderEvent);
    private static System.IntPtr RenderEventHandlePtr = Marshal.GetFunctionPointerForDelegate(RenderEventHandle);

    [MonoPInvokeCallback(typeof(RenderEventDelegate))]
    private static void SetRenderTextureRenderThread(int textureId)
    {
        System.IntPtr ptr = new System.IntPtr(textureId);
        SetCurrentRenderTexture(ptr);
    }

    private static RenderEventDelegate SetRenderTextureHandle = new RenderEventDelegate(SetRenderTextureRenderThread);
    private static System.IntPtr SetRenderTextureHandlePtr = Marshal.GetFunctionPointerForDelegate(SetRenderTextureHandle);

    public static void SetRenderTexture(System.IntPtr textureId)
    {
        GL.IssuePluginEvent(SetRenderTextureHandlePtr, textureId.ToInt32());
    }

    public static void SendRenderEvent(int eventId)
    {
        GL.IssuePluginEvent(RenderEventHandlePtr, eventId);
    }

    public static void SendRenderEventNative(int eventId)
    {
        GL.IssuePluginEvent(GetRenderEventFunc(), eventId);
    }

    public enum EngineThreadID {
        JAVA_THREAD,
        GAME_THREAD,
        RENDER_THREAD,
        WORKER1_THREAD,
        WORKER2_THREAD,
    }

    public enum EngineEventID
    {
        ENGINE_EVENT_ID_BEGIN = 0xA000,

        HMD_CREATE,
        HMD_INITIAILZED,
        HMD_RESUME,
        HMD_PAUSE,
        HMD_TERMINATED,

        FIRST_FRAME,
        FRAME_START,
        FRAME_END,

        UNITY_AWAKE,
        UNITY_ENABLE,
        UNITY_DISABLE,
        UNITY_START,
        UNITY_DESTROY,
        UNITY_APPLICATION_RESUME,
        UNITY_APPLICATION_PAUSE,
        UNITY_APPLICATION_QUIT,

        ENGINE_EVENT_ID_END
    }

    public static void IssueEngineEvent(EngineEventID eventID)
    {
        IssueEngineEvent(EngineThreadID.GAME_THREAD, eventID);
        IssueEngineEvent(EngineThreadID.RENDER_THREAD, eventID);
    }

    public static void IssueEngineEvent(EngineThreadID tID, EngineEventID eventID)
    {
#if UNITY_EDITOR
        if (Application.isEditor)
            return;
#endif
        if (tID == EngineThreadID.RENDER_THREAD)
        {
            SendRenderEvent((int) eventID);
        }
        else
        {
            NativeProcessEngineEvent((uint) tID, (uint)eventID);
        }
    }
}
