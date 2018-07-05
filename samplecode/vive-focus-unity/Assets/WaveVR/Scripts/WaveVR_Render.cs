// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Rendering;
using wvr;
using System;
using System.Collections;
using System.Collections.Generic;
using WaveVR_Log;

[RequireComponent(typeof(Camera))]
public class WaveVR_Render : MonoBehaviour
{
    private static string LOG_TAG = "WVR_Render";
    private static WaveVR_Render instance = null;
    public static WaveVR_Render Instance {
        get
        {
            return instance;
        }
    }
    public float ipd = 0.063f;
    private int targetFPS = -1;
    private static bool surfaceChanged = false;
    private static bool isNeedTimeout = false;

    public float sceneWidth { get; private set; }
    public float sceneHeight { get; private set; }
    public float[] projRawL { get; private set; }
    public float[] projRawR { get; private set; }
    public WaveVR_Utils.RigidTransform[] eyes { get; private set; }
    private bool configurationChanged = false;

    /// <summary>
    /// While enabling, 3D objects (eq. Cube) is touchable for EventSystem.
    /// </summary>
    public bool CameraGaze = false;
    public class RenderThreadSynchronizer
    {
        RenderTexture mutable = new RenderTexture(1,1,0);
        public RenderThreadSynchronizer()
        {
            mutable.useMipMap = false;
            mutable.Create();
        }

        // May call eglMakeCurrent inside.
        public void sync()
        {
            mutable.GetNativeTexturePtr();
        }
    }
    private RenderThreadSynchronizer synchronizer;

    public T GetComponentFromChildren<T>(string name)
    {
        var children = transform.Find(name);
        if (children != null)
        {
            var component = children.GetComponent<T>();
            return component;
        }
        return default(T);
    }
    const string OBJ_NAME_LEFT_EYE = "Eye Left";
    const string OBJ_NAME_RIGHT_EYE = "Eye Right";
    const string OBJ_NAME_EAR = "Ear";
    const string OBJ_NAME_DISTORTION = "Distortion";
    const string OBJ_NAME_RETICLE = "Reticle";
    const string OBJ_NAME_LOADING = "Loading";

    public bool isExpanded
    {
        get
        {
            if (lefteye == null)
                lefteye = GetComponentFromChildren<WaveVR_Camera>(OBJ_NAME_LEFT_EYE);
            if (righteye == null)
                righteye = GetComponentFromChildren<WaveVR_Camera>(OBJ_NAME_RIGHT_EYE);
            if (distortion == null)
                distortion = GetComponentFromChildren<WaveVR_Distortion>(OBJ_NAME_DISTORTION);
            return !(lefteye == null || righteye == null || distortion == null);
        }
    }

    public WaveVR_Camera lefteye = null;
    public WaveVR_Camera righteye = null;
    public WaveVR_Distortion distortion = null;
    public GameObject loadingCanvas = null;  // Loading canvas will force clean black to avoid any thing draw on screen before Wave's Graphic's ready.
    public GameObject ear = null;

    public class TexturePool
    {
        public struct TextureConfig
        {
            public int w;
            public int h;
            public int depth;
            public RenderTextureFormat format;
            public bool useMipMap;
            public int anisoLevel;
            public FilterMode filterMode;
            public TextureWrapMode wrapMode;
            public int antiAliasing;
        }

        private IntPtr queue;
        private Dictionary<IntPtr, RenderTexture> textures = new Dictionary<IntPtr, RenderTexture>();
        public int size { get; private set; }
        public RenderTexture currentRt { get; private set; }
        public IntPtr currentPtr { get; private set; }
        public bool isLeft { get; private set; }
        public bool isReleased { get; private set; }

        public TexturePool(TextureConfig cfg, int size, bool isLeft)
        {
            isReleased = false;

            // Editor doesn't need the texture queue.
            if (Application.isEditor)
                size = 1;

            this.isLeft = isLeft;
            this.size = size;
            for (int i = 0; i < size; i++)
            {
                currentRt = new RenderTexture(cfg.w, cfg.h, cfg.depth, cfg.format, RenderTextureReadWrite.Default);
                currentRt.useMipMap = cfg.useMipMap;
                currentRt.wrapMode = cfg.wrapMode;
                currentRt.filterMode = cfg.filterMode;
                currentRt.anisoLevel = cfg.anisoLevel;
                currentRt.antiAliasing = cfg.antiAliasing;
                currentRt.Create();
                currentPtr = currentRt.GetNativeTexturePtr();

                textures.Add(currentPtr, currentRt);
            }
#if UNITY_EDITOR
            if (Application.isEditor) return;
#endif
            var array = new IntPtr[textures.Count];
            textures.Keys.CopyTo(array, 0);
            queue = WaveVR_Utils.WVR_StoreRenderTextures(array, size, isLeft);
        }

        ~TexturePool()
        {
#if UNITY_EDITOR
            // Application.isEditor can't be used in the destructure.
            bool editor = true;
            if (editor)
                return;
#endif
            if (textures != null)
            {
                foreach (RenderTexture texture in textures.Values)
                {
                    texture.Release();
                }

                Interop.WVR_ReleaseTextureQueue(queue);
            }
        }

        private RenderTexture GetRenderTextureByPtr(IntPtr ptr)
        {
            RenderTexture rt = null;
            if (!textures.TryGetValue(ptr, out rt))
                Log.e(LOG_TAG, "Unknown RenderTexture ID" + ((int) ptr));
            return rt;
        }

        public void next()
        {
#if UNITY_EDITOR
            if (Application.isEditor) return;
#endif
            if (isReleased)
                return;
            Log.gpl.d(LOG_TAG, "Get texture from queue");
            currentPtr = (IntPtr)WaveVR_Utils.WVR_GetAvailableTextureID(queue);
            currentRt = GetRenderTextureByPtr(currentPtr);
            if (currentPtr == IntPtr.Zero)
                currentPtr = IntPtr.Zero;
            //Log.d(LOG_TAG, "TextureID" + currentPtr.ToInt32());
        }

        public void Release()
        {
            Log.d(LOG_TAG, "TexturePool Release()");
            if (isReleased)
                return;
            isReleased = true;

            if (textures != null)
            {
                foreach (RenderTexture texture in textures.Values)
                    texture.Release();
                textures.Clear();
            }
            textures = null;

#if UNITY_EDITOR
            if (!Application.isEditor)
#endif
            {
                if (queue != null)
                {
                    Interop.WVR_ReleaseTextureQueue(queue);
                }
                queue = IntPtr.Zero;
            }

            size = 0;
            currentPtr = IntPtr.Zero;
            currentRt = null;
        }
    }



    public class TextureManager
    {
        private int poolSize = 3;

        public TexturePool left { get; private set; }
        public TexturePool right { get; private set; }

        // Must init in Awake and make sure VRCompositor initialized.
        public TextureManager()
        {
            Log.d(LOG_TAG, "TextureManager()");
            left = null;
            right = null;
            reset();
        }

        // After Release, TextureManager will be reset when first invoke Next().
        public void ReleaseTexturePools()
        {
            // Not set pools to null.  The pools can be accessed after released.
            if (left != null)
                left.Release();

            if (right != null)
                right.Release();
        }


    public void reset() {
            Log.d(LOG_TAG, "TextureManager reset()");
            poolSize = 3;
            if (!Application.isEditor)
                poolSize = WaveVR_Utils.WVR_GetNumberOfTextures();

            uint w = (uint)Screen.width / 2;
            uint h = (uint)Screen.height;
            if (!Application.isEditor)
                Interop.WVR_GetRenderTargetSize(ref w, ref h);
            int screenWidth = (int)(w);
            int screenHeight = (int)(h);

            Log.d(LOG_TAG, "TextureManager: screenWidth=" + screenWidth + " screenHeight=" + screenHeight);

            int antiAliasing = QualitySettings.antiAliasing;
            if (antiAliasing == 0)
                antiAliasing = 1;

            var cfg = new TexturePool.TextureConfig();
            cfg.w = screenWidth;
            cfg.h = screenHeight;
            cfg.depth = 24;
            cfg.format = RenderTextureFormat.ARGB32;
            cfg.useMipMap = false;
            cfg.wrapMode = TextureWrapMode.Clamp;
            cfg.filterMode = FilterMode.Bilinear;
            cfg.anisoLevel = 1;
            cfg.antiAliasing = antiAliasing;

            if (!validate())
                ReleaseTexturePools();

            left = new TexturePool(cfg, poolSize, true);
            right = new TexturePool(cfg, poolSize, false);
        }

        public bool validate()
        {
            return left != null && right != null && !left.isReleased && !right.isReleased;
        }

        public void Next()
        {
            if (!validate()) {
                reset();
            }
#if UNITY_EDITOR
            if (Application.isEditor)
                return;
#endif
            left.next();
            right.next();
        }

        public IntPtr GetNativePtr(WVR_Eye eye)
        {
            return eye == WVR_Eye.WVR_Eye_Left ? left.currentPtr : right.currentPtr;
        }

        public IntPtr GetNativePtr(bool isLeftEye)
        {
            return isLeftEye ? left.currentPtr : right.currentPtr;
        }

        public RenderTexture GetRenderTexture(WVR_Eye eye)
        {
            return eye == WVR_Eye.WVR_Eye_Left ? left.currentRt : right.currentRt;
        }

        public RenderTexture GetRenderTexture(bool isLeftEye)
        {
            return isLeftEye ? left.currentRt : right.currentRt;
        }
    }
    public TextureManager textureManager { get; private set; }

    public static int globalOrigin = -1;
    public WVR_PoseOriginModel _origin = WVR_PoseOriginModel.WVR_PoseOriginModel_OriginOnGround;
    public WVR_PoseOriginModel origin { get { return _origin; } set { _origin = value; OnIpdChanged(null); } }

    public static void InitializeGraphic(RenderThreadSynchronizer synchronizer = null)
    {
#if UNITY_EDITOR
        if (Application.isEditor) return;
#endif
        WaveVR_Utils.SendRenderEvent(WaveVR_Utils.RENDEREVENTID_INIT_GRAPHIC);
        if (synchronizer != null)
            synchronizer.sync();
    }

    public void OnIpdChanged(params object[] args)
    {
        Log.d(LOG_TAG, "configurationChanged");
#if UNITY_EDITOR
        if (Application.isEditor) return;
#endif

        WVR_NumDoF dof;
        if (WaveVR.Instance.is6DoFTracking() == 3)
        {
            dof = WVR_NumDoF.WVR_NumDoF_3DoF;
        }
        else
        {
            if (origin == WVR_PoseOriginModel.WVR_PoseOriginModel_OriginOnHead_3DoF)
                dof = WVR_NumDoF.WVR_NumDoF_3DoF;
            else
                dof = WVR_NumDoF.WVR_NumDoF_6DoF;
        }

        //for update EyeToHead transform
        WVR_Matrix4f_t eyeToHeadL = Interop.WVR_GetTransformFromEyeToHead(WVR_Eye.WVR_Eye_Left, dof);
        WVR_Matrix4f_t eyeToHeadR = Interop.WVR_GetTransformFromEyeToHead(WVR_Eye.WVR_Eye_Right, dof);

        eyes = new WaveVR_Utils.RigidTransform[] {
                new WaveVR_Utils.RigidTransform(eyeToHeadL),
                new WaveVR_Utils.RigidTransform(eyeToHeadR)
            };

        //for update projection matrix
        Interop.WVR_GetClippingPlaneBoundary(WVR_Eye.WVR_Eye_Left, ref projRawL[0], ref projRawL[1], ref projRawL[2], ref projRawL[3]);
        Interop.WVR_GetClippingPlaneBoundary(WVR_Eye.WVR_Eye_Right, ref projRawR[0], ref projRawR[1], ref projRawR[2], ref projRawR[3]);

        Log.d(LOG_TAG, "targetFPS=" + targetFPS + " sceneWidth=" + sceneWidth + " sceneHeight=" + sceneHeight +
            "\nprojRawL[0]=" + projRawL[0] + " projRawL[1]=" + projRawL[1] + " projRawL[2]=" + projRawL[2] + " projRawL[3]=" + projRawL[3] +
            "\nprojRawR[0]=" + projRawR[0] + " projRawR[1]=" + projRawR[1] + " projRawR[2]=" + projRawR[2] + " projRawR[3]=" + projRawR[3] +
            "\neyes[L]=" + eyes[0].pos + " eyes[R]=" + eyes[1].pos);
        configurationChanged = true;
    }

    void Awake()
    {
        Log.d(LOG_TAG, "Awake()+");
        if (instance == null)
            instance = this;
        else
            Log.w(LOG_TAG, "Render already Awaked");
        synchronizer = new RenderThreadSynchronizer();

        if (globalOrigin >= 0 && globalOrigin <= 3)
        {
            _origin = (WVR_PoseOriginModel) globalOrigin;
            Log.d(LOG_TAG, "Has global tracking space " + _origin);
        }

#if UNITY_EDITOR
        if (!Application.isEditor)
#endif
        {
            if (WaveVR_Init.Instance == null || WaveVR.Instance == null)
                Log.e(LOG_TAG, "Fail to initialize");

            // This command can make sure native's render code are initialized in render thread.
            InitializeGraphic(synchronizer);

            // Setup render values
            uint w = 0, h = 0;
            Interop.WVR_GetRenderTargetSize(ref w, ref h);
            sceneWidth = (float)w;
            sceneHeight = (float)h;

            projRawL = new float[4] { 0.0f, 0.0f, 0.0f, 0.0f };
            projRawR = new float[4] { 0.0f, 0.0f, 0.0f, 0.0f };

            WVR_RenderProps_t props = new WVR_RenderProps_t();
            Interop.WVR_GetRenderProps(ref props);
            targetFPS = (int)props.refreshRate;

            OnIpdChanged(null);
        }

        // May call eglMakeCurrent inside TextureManager()

        textureManager = new TextureManager();

        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.HMD_INITIAILZED);

        Screen.sleepTimeout = SleepTimeout.NeverSleep;
        Application.targetFrameRate = targetFPS;
        Log.d(LOG_TAG, "Awake()-");
    }

    private Coroutine renderLooperCoroutine = null;
    private void enableRenderLoop(bool start)
    {
        if (start)
        {
            if (renderLooperCoroutine != null)
                return;
            var renderLoop = RenderLoop();
            renderLooperCoroutine = StartCoroutine(renderLoop);
        }
        else
        {
            StopCoroutine(renderLooperCoroutine);
            renderLooperCoroutine = null;
        }
    }

    void OnEnable()
    {
        Log.d(LOG_TAG, "OnEnable()+");
        WaveVR_Utils.Event.Listen("IpdChanged", OnIpdChanged);
        enableRenderLoop(true);
        setLoadingCanvas(true);
        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_ENABLE);
        Log.d(LOG_TAG, "OnEnable()-");
    }

    void Start()
    {
        Log.d(LOG_TAG, "Start()+");
        var camera = GetComponent<Camera>();
        camera.cullingMask = CameraGaze ? camera.cullingMask : 0;
        camera.backgroundColor = Color.black;
        camera.clearFlags = CameraClearFlags.Nothing;
        // Camera.main need this camera to enabled
        camera.enabled = true;
        camera.rect = new Rect (0.5f - 0.01f, 0.5f - 0.01f, 0.02f, 0.02f);
        camera.tag = "MainCamera";

        WaveVR_Render.Expand(this);
        configurationChanged = false;
        Log.d(LOG_TAG, "Start()-");
    }

    public static void signalSurfaceState(string msg) {
        Log.d(LOG_TAG, "signalSurfaceState[ " + msg + " ]");
        if (String.Equals(msg, "CHANGED")) {
            surfaceChanged = false;
        } else if (String.Equals(msg, "CHANGED_WRONG")) {
            surfaceChanged = false;
            isNeedTimeout = true;
        } else if (String.Equals(msg, "CHANGED_RIGHT")) {
            surfaceChanged = true;
        } else if (String.Equals(msg, "DESTROYED")) {
            surfaceChanged = false;
            Log.d(LOG_TAG, "surfaceDestroyed");
        }
    }

    void OnApplicationPause(bool pauseStatus)
    {
        Log.d(LOG_TAG, "Pause(" + pauseStatus + ")");

        if (pauseStatus)
        {
            WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_APPLICATION_PAUSE);
            if (synchronizer != null)
                synchronizer.sync();
            lefteye.getCamera().targetTexture = null;
            righteye.getCamera().targetTexture = null;
            textureManager.ReleaseTexturePools();
        }
        else
        {
            WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_APPLICATION_RESUME);
        }

        setLoadingCanvas(true);
        enableRenderLoop(!pauseStatus);
    }

    public void PauseUnity()
    {
        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_APPLICATION_PAUSE);
        if (synchronizer != null)
            synchronizer.sync();
        lefteye.getCamera().targetTexture = null;
        righteye.getCamera().targetTexture = null;
        textureManager.ReleaseTexturePools();

        setLoadingCanvas(true);
        enableRenderLoop(false);
    }

    public void ResumeUnity()
    {
        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_APPLICATION_RESUME);
        if (synchronizer != null)
            synchronizer.sync();
        setLoadingCanvas(true);
        enableRenderLoop(true);
    }

    private bool isSettingQualityLevel = false;
    public int SetQualityLevel(int level, bool applyExpensiveChanges = true)
    {
        if (level < 0) return -1;
        string[] names = QualitySettings.names;
        if (level >= names.Length) return -1;
        if (isSettingQualityLevel) return -1;
        isSettingQualityLevel = true;
        int qualityLevel = QualitySettings.GetQualityLevel();
        if (qualityLevel != level)
        {
            PauseUnity();
            QualitySettings.SetQualityLevel(level, applyExpensiveChanges);
            Invoke("ResumeUnity", 1);

            qualityLevel = QualitySettings.GetQualityLevel();
        }
        isSettingQualityLevel = false;
        return qualityLevel;
    }

    void LateUpdate()
    {
        Log.gpl.check();
    }

    void OnApplicationQuit()
    {
        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_APPLICATION_QUIT);
        if (synchronizer != null)
            synchronizer.sync();
    }

    void OnDisable()
    {
        Log.d(LOG_TAG, "OnDisable()+");
        enableRenderLoop(false);
#if UNITY_EDITOR
        if (!Application.isEditor)
#endif
        {
            WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_DISABLE);
            if (synchronizer != null)
                synchronizer.sync();
        }
        WaveVR_Utils.Event.Remove("IpdChanged", OnIpdChanged);
        setLoadingCanvas(false);

        if (lefteye != null)
            lefteye.getCamera().targetTexture = null;
        if (righteye != null)
            righteye.getCamera().targetTexture = null;
        if (textureManager != null)
            textureManager.ReleaseTexturePools();

        Log.d(LOG_TAG, "OnDisable()-");
    }

    void OnDestroy()
    {
        Log.d(LOG_TAG, "OnDestroy()+");
        textureManager = null;
        instance = null;
        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.UNITY_DESTROY);
        Log.d(LOG_TAG, "OnDestroy()-");
    }

    private IEnumerator RenderLoop()
    {
        var wait = new WaitForEndOfFrame();
        yield return wait;
        Log.d(LOG_TAG, "RenderLoop() is started");
        var tim = Time.time;
#if UNITY_EDITOR
        if (!Application.isEditor)
#endif
        {
            // Restart ATW thread before rendering.
            while (!WaveVR_Utils.WVR_IsATWActive()) {
                yield return wait;
                if (surfaceChanged && isNeedTimeout == false)
                    break;
                if (Time.time - tim > 1.0f)
                {
                    Log.w(LOG_TAG, "Waiting for surface change is timeout.");
                    break;
                }
            }
            // Reset isNeedTimeout flag
            isNeedTimeout = false;

            if (textureManager != null)
            {
                if (!textureManager.validate())
                    textureManager.reset();
            }
        }
        Log.d(LOG_TAG, "First frame");
        WaveVR_Utils.IssueEngineEvent(WaveVR_Utils.EngineEventID.FIRST_FRAME);

        setLoadingCanvas(false);
        Log.d(LOG_TAG, "RenderLoop() is running");

        while (true)
        {
            Log.gpl.d(LOG_TAG, "RenderLoop() is still running");
            WaveVR_Utils.Trace.BeginSection("RenderLoop", false);
#if UNITY_EDITOR
            if (Application.isEditor)
            {
                WaveVR_Utils.Event.Send(WaveVR_Utils.Event.NEW_POSES, new WVR_DevicePosePair_t[0], new WaveVR_Utils.RigidTransform[0]);
                WaveVR_Utils.Event.Send(WaveVR_Utils.Event.AFTER_NEW_POSES);
                textureManager.Next();
            }
            else
#endif
            {
                WaveVR.Instance.UpdatePoses(origin);
                // Set next texture before running any graphic command.
                textureManager.Next();
            }

            if (configurationChanged)
            {
                WaveVR_Render.Expand(this);
                configurationChanged = false;
            }

            RenderEye(lefteye.getCamera(), WVR_Eye.WVR_Eye_Left);
            RenderEye(righteye.getCamera(), WVR_Eye.WVR_Eye_Right);
            WaveVR_Utils.Trace.EndSection(false);

            // Put here to control the time of next frame.
            TimeControl();

            Log.gpl.d(LOG_TAG, "End of frame");
            yield return wait;
        }
    }

    private void RenderEye(Camera camera, WVR_Eye eye)
    {
        WaveVR_Utils.Trace.BeginSection("Render_" + eye);
        Log.gpl.d(LOG_TAG, "Render_" + eye);

        bool isleft = eye == WVR_Eye.WVR_Eye_Left;
#if UNITY_EDITOR
        if (!Application.isEditor)
#endif
            WaveVR_Utils.SendRenderEventNative(isleft ?
                WaveVR_Utils.k_nRenderEventID_RenderEyeL :
                WaveVR_Utils.k_nRenderEventID_RenderEyeR);
        WaveVR_CanvasEye.changeEye(camera);
        camera.enabled = true;
        RenderTexture rt = textureManager.GetRenderTexture(isleft);
        camera.targetTexture = rt;
        camera.Render();
        camera.enabled = false;
#if UNITY_EDITOR
        if (Application.isEditor)
        {
            distortion.RenderEye(eye, rt);
            return;
        }
#endif
        // Do submit
        WaveVR_Utils.SetRenderTexture(isleft ?
            textureManager.left.currentPtr :
            textureManager.right.currentPtr);

        WaveVR_Utils.SendRenderEventNative(isleft ?
            WaveVR_Utils.k_nRenderEventID_SubmitL :
            WaveVR_Utils.k_nRenderEventID_SubmitR);

        WaveVR_Utils.SendRenderEventNative(isleft ?
            WaveVR_Utils.k_nRenderEventID_RenderEyeEndL :
            WaveVR_Utils.k_nRenderEventID_RenderEyeEndR);
        WaveVR_Utils.Trace.EndSection();
    }

    private void AddRaycaster()
    {
        PhysicsRaycaster ray = gameObject.GetComponent<PhysicsRaycaster>();
        if (ray == null)
            ray = gameObject.AddComponent<PhysicsRaycaster>();
        LayerMask mask = -1;
        mask.value = LayerMask.GetMask("Default", "TransparentFX", "Water");
        ray.eventMask = mask;
    }

    private WaveVR_Camera CreateEye(WVR_Eye eye)
    {
        Log.d(LOG_TAG, "CreateEye(" + eye + ")+");

        bool isleft = eye == WVR_Eye.WVR_Eye_Left;
        WaveVR_Camera vrcamera = isleft ? lefteye : righteye;
        Camera camera;
        if (vrcamera == null)
        {
            string eyename = isleft ? OBJ_NAME_LEFT_EYE : OBJ_NAME_RIGHT_EYE;
            GameObject go = new GameObject(eyename);
            go.transform.SetParent(transform, false);
            camera = go.AddComponent<Camera>();
            camera.nearClipPlane = GetComponent<Camera>().nearClipPlane;
            camera.farClipPlane = GetComponent<Camera>().farClipPlane;
            go.AddComponent<FlareLayer>();
            go.AddComponent<GUILayer>();
            vrcamera = go.AddComponent<WaveVR_Camera>();
        }
        else
        {
            camera = vrcamera.GetComponent<Camera>();
        }

        if (Application.isEditor)
        {
            camera.transform.localPosition = new Vector3(isleft ? -ipd / 2 : ipd / 2, 0, 0.15f);
        }
        else
        {
            camera.transform.localPosition = eyes[isleft ? 0 : 1].pos;
        }

        vrcamera.eye = eye;
        camera.enabled = false;

        if (Application.isEditor)
            camera.projectionMatrix = GetEditorProjection(eye);
        else
            camera.projectionMatrix = GetProjection(eye);

        Log.d(LOG_TAG, "CreateEye(" + eye + ")-");
        return vrcamera;
    }

    private void createDistortion()
    {
        Log.d(LOG_TAG, "createDistortion()+");

        if (distortion == null)
        {
            GameObject distortionobj = new GameObject(OBJ_NAME_DISTORTION);
            distortionobj.transform.SetParent(transform, false);
            distortionobj.AddComponent<Camera>();
            distortion = distortionobj.AddComponent<WaveVR_Distortion>();
        }
        distortion.init();
        Log.d(LOG_TAG, "createDistortion()-");
    }

    /**
     * The loading black is used to block the other camera or UI drawing on the display.
     * The native render will use the screen after WaitForEndOfFrame.  And the
     * native render need time to be ready for sync with Android's flow.  Therefore, the
     * Screen or HMD may show othehr camera or UI's drawing.  For example, the graphic
     * raycast need the camera has real output on screen.  We draw it, and cover it by
     * binocular vision.  It let the gaze or the controller work well.  If we don't
     * have a black canvas and the native render is delayed, the screen may show a BG
     * color or the raycast image on the screen for a while.
    **/
    private void createLoadingBlack()
    {
        var found = GetComponentFromChildren<Canvas>(OBJ_NAME_LOADING);
        if (found == null)
        {
            loadingCanvas = new GameObject(OBJ_NAME_LOADING);
            var canvas = loadingCanvas.AddComponent<Canvas>();
            loadingCanvas.AddComponent<UnityEngine.UI.CanvasScaler>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            GameObject loadingImage = new GameObject("Loading Image");
            loadingImage.transform.SetParent(loadingCanvas.transform, false);
            loadingImage.AddComponent<CanvasRenderer>();
            UnityEngine.UI.Image loading = loadingImage.AddComponent<UnityEngine.UI.Image>();
            loading.material = null;
            loading.color = Color.black;
            loading.raycastTarget = false;
            loading.rectTransform.anchoredPosition = new Vector2(0.5f, 0.5f);
            loading.rectTransform.anchorMin = new Vector2(0, 0);
            loading.rectTransform.anchorMax = new Vector2(1, 1);
            loading.rectTransform.offsetMin = new Vector2(0, 0);
            loading.rectTransform.offsetMax = new Vector2(0, 0);

            canvas.enabled = false;  // Avoid black in Editor GameView preview or configuraiton change.
            loadingCanvas.transform.SetParent(transform, true);
        }
    }

    private void setLoadingCanvas(Boolean enabled)
    {
        if (loadingCanvas)
        {
            var canvas = loadingCanvas.GetComponent<Canvas>();
            if (canvas)
                canvas.enabled = enabled;
        }
    }

    public static void Expand(WaveVR_Render head)
    {
        Log.d(LOG_TAG, "Expand()+");

        if (head.isExpanded) {
            //Debug.Log("Expanded");
        }
        head.righteye = head.CreateEye(WVR_Eye.WVR_Eye_Right);
        head.lefteye = head.CreateEye(WVR_Eye.WVR_Eye_Left);
        head.createDistortion();

        var found = head.GetComponentFromChildren<AudioListener>(OBJ_NAME_EAR);
        if (found == null) {
            var earObj = new GameObject(OBJ_NAME_EAR);
            earObj.transform.SetParent(head.transform, false);
            earObj.transform.localPosition = new Vector3(0, 0, -0.01f);  // TODO if 6DOF should be around -0.025f
            earObj.AddComponent<AudioListener>();
            head.ear = earObj;
        }

        head.AddRaycaster();

        head.createLoadingBlack();
        Log.d(LOG_TAG, "Expand()-");
    }

    public static void Collapse(WaveVR_Render head)
    {
        if (head.lefteye != null)
            DestroyImmediate(head.lefteye.gameObject);
        head.lefteye = null;

        if (head.righteye != null)
            DestroyImmediate(head.righteye.gameObject);
        head.righteye = null;

        if (head.distortion != null)
            DestroyImmediate(head.distortion.gameObject);
        head.distortion = null;

        Transform ear = head.transform.Find(OBJ_NAME_EAR);
        if (ear != null)
            DestroyImmediate(ear.gameObject);
        ear = null;

        var raycast = head.GetComponent<PhysicsRaycaster>();
        if (raycast != null)
            DestroyImmediate(raycast);
        raycast = null;

        if (head.loadingCanvas != null)
        {
            var loading = head.loadingCanvas.gameObject;
            head.loadingCanvas = null;
            DestroyImmediate(loading);
        }
    }

    private Matrix4x4 GetEditorProjection(WVR_Eye eye)
    {
        var camera = GetComponent<Camera>();
        Matrix4x4 proj = Matrix4x4.identity;
        float w = Mathf.Tan(camera.fieldOfView / 2 * Mathf.Deg2Rad);
        float h = w / (Screen.width / 2) * Screen.height;
        float l = -w / 2, r = w / 2, t = h / 2, b = -h / 2;
        proj = MakeProjection(l, r, t, b, camera.nearClipPlane, camera.farClipPlane);
        return proj;
    }

    private Matrix4x4 GetProjection(WVR_Eye eye)
    {
        Log.d(LOG_TAG, "GetProjection()");
        var camera = GetComponent<Camera>();
        Matrix4x4 proj = Matrix4x4.identity;
        float[] rect = new float[4] { 0.0f, 0.0f, 0.0f, 0.0f };
        if (eye == WVR_Eye.WVR_Eye_Left)
            rect = projRawL;
        else
            rect = projRawR;

        // The values in ProjectionRaw are made by assuming the near value is 1.
        proj = MakeProjection(rect[0], rect[1], rect[2], rect[3], camera.nearClipPlane, camera.farClipPlane);
        return proj;
    }

    //private void frustum(float ipd, float dpi, int width, int height, float fov, float near, float far, float)

    public static Matrix4x4 MakeProjection(float l, float r, float t, float b, float n, float f)
    {
        Matrix4x4 m = Matrix4x4.zero;
        m[0, 0] = 2 / (r - l);
        m[1, 1] = 2 / (t - b);
        m[0, 2] = (r + l) / (r - l);
        m[1, 2] = (t + b) / (t - b);
        m[2, 2] = -(f + n) / (f - n);
        m[2, 3] = -2 * f * n / (f - n);
        m[3, 2] = -1;
        return m;
    }

#region TimeControl
    // TimeControl: Set Time.timeScale = 0 if input focus in gone.
    private bool previousInputFocus = true;
    public bool needTimeControl = false;

    private void TimeControl()
    {
        if (needTimeControl)
        {
#if UNITY_EDITOR
            // Nothing can simulate the focus lost in editor.  Just leave.
            if (Application.isEditor)
                return;
#endif
            bool hasInputFocus = !Interop.WVR_IsInputFocusCapturedBySystem();

            if (!previousInputFocus || !hasInputFocus)
            {
                previousInputFocus = hasInputFocus;
                Time.timeScale = hasInputFocus ? 1 : 0;
                Log.d(LOG_TAG, "InputFocus " + hasInputFocus + "Time.timeScale " + Time.timeScale);
            }
        }
    }
#endregion
}
