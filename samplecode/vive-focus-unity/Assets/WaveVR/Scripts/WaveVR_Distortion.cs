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
using wvr;

[RequireComponent(typeof(Camera))]
public class WaveVR_Distortion : MonoBehaviour
{
    public class Config
    {
        public Mesh mesh;
        public Rect rect;
        public Material material;
        public bool eye;
        public Vector2 center;
    }
    private static wvr.distortion.Config current;
    private wvr.distortion.Config Cleft, Cright;
    private Camera cam;

    Vector2 GetCenter(WVR_Eye eye)
    {
        Vector2 o;
        if (Application.isEditor)
        {
            o = new Vector2(0.5f, 0.5f);
        }
        else
        {
            float l = 0f, r = 0f, t = 0f, b = 0f;
            Interop.WVR_GetClippingPlaneBoundary(eye, ref l, ref r, ref t, ref b);
            o = new Vector2(-l / (r - l), t / (t - b));
        }
        return o;
    }

    void Awake() {
        if (Application.isEditor)
            return;
    }

    void Start()
    {
        init();
        reset();
    }

    void reset()
    {
        // NOTE: On Unity version 5.6.03f, If the antiAliasing was enabled on the
        // main window surface with front buffer rendering in editor mode. The view will
        // render black.  And HDR has the same problem.  Disable them for Editor mode.
        #if UNITY_5_6_OR_NEWER && UNITY_EDITOR
            cam.allowMSAA = false;
            cam.allowHDR = false;
        #endif
    }

    public void init()
    {
        wvr.distortion.Distortion obj = new wvr.distortion.Distortion();
        Cleft = new wvr.distortion.Config();
        Cleft.eye = true;
        Cleft.rect = new Rect(0, 0, 0.5f, 1);
        Cleft.expandRatio = 1;
        Cleft.center = GetCenter(WVR_Eye.WVR_Eye_Left);
        Cleft.mesh = obj.createMesh(Cleft);
        Cleft.mesh.name = "left_eye_distortion";
        Cright = new wvr.distortion.Config();
        Cright.eye = false;
        Cright.rect = new Rect(0.5f, 0, 0.5f, 1);
        Cright.expandRatio = 1;
        Cright.center = GetCenter(WVR_Eye.WVR_Eye_Right);
        Cright.mesh = obj.createMesh(Cright);
        Cright.mesh.name = "right_eye_distortion";

        Shader shader = Resources.Load<Shader>("WaveVR_Distortion");
        if (shader == null)
        {
            Debug.LogError("Can't find WaveVR/Distortion shader");
        }
        var material = new Material(shader);
        Cleft.material = material;
        Cright.material = material;

        cam = GetComponent<Camera>();
        cam.orthographic = true;
        cam.orthographicSize = 1;
        cam.cullingMask = 0;
        cam.backgroundColor = Color.black;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.projectionMatrix = Matrix4x4.identity;
        cam.enabled = false;

        current = Cright;
    }

    void OnPostRender()
    {
        if (Camera.current != cam)
            return;
        Matrix4x4 mat = Matrix4x4.identity;

        current.material.SetPass(0);
        Graphics.DrawMeshNow(current.mesh, mat);
    }

    public void RenderEye(WVR_Eye eye, RenderTexture texture)
    {
        reset();
        WaveVR_Utils.Trace.BeginSection("Distortion_" + eye);
        current = eye == WVR_Eye.WVR_Eye_Left ? Cleft : Cright;
        current.material.mainTexture = texture;

        cam.enabled = true;
        cam.rect = current.rect;
        cam.Render();
        cam.enabled = false;
        WaveVR_Utils.Trace.EndSection();
    }
}
