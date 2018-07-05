using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;
using WaveVR_Log;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class HalfUVSquare : MonoBehaviour {
    private Mesh meshL, meshR;
    public Material material;
    private bool updated = true;
    private bool started = false;
    Texture2D nativeTexture = null;
    System.IntPtr textureid;
    private MeshRenderer meshrenderer;

    // Use this for initialization
    void Start()
    {
        meshL = CreateMesh(true);
        meshR = CreateMesh(false);

#if UNITY_EDITOR
        if (Application.isEditor)
        {
            return;
        }
#endif
        WaveVR_CameraTexture.UpdateCameraCompletedDelegate += updateTextureCompleted;
        started = WaveVR_CameraTexture.instance.startCamera();
        nativeTexture = new Texture2D(1280, 400);
        textureid = nativeTexture.GetNativeTexturePtr();
        meshrenderer = GetComponent<MeshRenderer>();
        meshrenderer.material.mainTexture = nativeTexture;
    }

    void updateTextureCompleted(uint textureId)
    {
//        Log.d(LOG_TAG, "updateTextureCompleted, textureid = " + textureId);

        updated = true;
        material.mainTexture = nativeTexture;
    }

    void Update()
    {
#if UNITY_EDITOR
        if (Application.isEditor)
        {
            return;
        }
#endif

        if (updated && started)
        {
            updated = false;
            WaveVR_CameraTexture.instance.updateTexture((uint)textureid);
        }
    }

    private Mesh CreateMesh(bool isLeft)
    {
        // ab
        // cd
        List<Vector3> vertices = new List<Vector3>();
        vertices.Add(new Vector3(-1, 1, 0.5f)); // a
        vertices.Add(new Vector3(1, 1, 0.5f)); // b
        vertices.Add(new Vector3(-1, -1, 0.5f)); // c
        vertices.Add(new Vector3(1, -1, 0.5f)); // d

        List<Vector2> uvsL = new List<Vector2>();
        uvsL.Add(new Vector2(0, 1));
        uvsL.Add(new Vector2(0.5f, 1));
        uvsL.Add(new Vector2(0, 0));
        uvsL.Add(new Vector2(0.5f, 0));
        //uvsL.Add(new Vector2(0, 0));
        //uvsL.Add(new Vector2(0.5f, 0));
        //uvsL.Add(new Vector2(0, 1));
        //uvsL.Add(new Vector2(0.5f, 1));

        List<Vector2> uvsR = new List<Vector2>();
        uvsR.Add(new Vector2(0.5f, 1));
        uvsR.Add(new Vector2(1, 1));
        uvsR.Add(new Vector2(0.5f, 0));
        uvsR.Add(new Vector2(1, 0));
        //uvsR.Add(new Vector2(0.5f, 0));
        //uvsR.Add(new Vector2(1, 0));
        //uvsR.Add(new Vector2(0.5f, 1));
        //uvsR.Add(new Vector2(1, 1));

        List<int> indices = new List<int>();
        indices.Add(0);
        indices.Add(2);
        indices.Add(1);

        indices.Add(1);
        indices.Add(2);
        indices.Add(3);

        Mesh mesh = new Mesh();
        mesh.vertices = vertices.ToArray();
        if (isLeft)
            mesh.SetUVs(0, uvsL);
        else
            mesh.SetUVs(0, uvsR);

        mesh.SetIndices(indices.ToArray(), MeshTopology.Triangles, 0);

        return mesh;
    }

    void OnEnable()
    {
     //   WaveVR_Utils.Event.Listen(WaveVR_Utils.Event.RENDER_OBJECT_LEFT, RenderLeft);
     //   WaveVR_Utils.Event.Listen(WaveVR_Utils.Event.RENDER_OBJECT_RIGHT, RenderRight);
    }

    void OnDisable()
    {
        WaveVR_Utils.Event.Remove(WaveVR_Utils.Event.RENDER_OBJECT_LEFT, RenderLeft);
        WaveVR_Utils.Event.Remove(WaveVR_Utils.Event.RENDER_OBJECT_RIGHT, RenderRight);

        WaveVR_CameraTexture.instance.stopCamera();
        WaveVR_CameraTexture.UpdateCameraCompletedDelegate -= updateTextureCompleted;
    }

    public void RenderLeft(params object[] args)
    {
        if (material != null)
        {
            material.SetPass(0);
            Graphics.DrawMeshNow(meshL, Matrix4x4.identity);
        }
    }

    public void RenderRight(params object[] args)
    {
        if (material != null)
        {
            material.SetPass(0);
            Graphics.DrawMeshNow(meshR, Matrix4x4.identity);
        }
    }
}
