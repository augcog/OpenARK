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
using System.Collections.Generic;
using wvr;
using WaveVR_Log;
using System;
using System.Text;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class WaveVR_Beam: MonoBehaviour
{
    private static string LOG_TAG = "WaveVR_Beam";
    public bool useSystemConfig = true;
    public float startWidth = 0.000625f;    // in x,y axis
    public float endWidth = 0.00125f;       // let the bean seems the same width in far distance.
    public float startOffset = 0.015f;

    public float endOffset = 0.65f;
    private const float endOffsetMin = 0.5f;     // Minimum distance of end offset (in meters).
    private float endOffsetMax = 9.5f;    // Maximum distance of end offset (in meters).

    public Color32 StartColor = new Color32 (255, 255, 255, 255);
    private Color32 TailColor = new Color32 (255, 255, 255, 255);
    public Color32 EndColor = new Color32 (255, 255, 255, 77);

    private bool enableBeam = true;

    public int count = 3;
    public bool updateEveryFrame = true;
    public bool makeTail = true; // Offset from 0

    private int maxUVAngle = 30;
    private const float epsilon = 0.001f;

    private void Validate()
    {

        if (startWidth < epsilon)
            startWidth = epsilon;

        if (endWidth < epsilon)
            endWidth = epsilon;

        if (startOffset < epsilon)
            startOffset = epsilon;

        if (endOffset < epsilon * 2)
            endOffset = epsilon * 2;

        if (endOffset < startOffset)
            endOffset = startOffset + epsilon;

        if (count < 3)
            count = 3;

        /**
         * The texture pattern should be a radiated image starting 
         * from the texture center.
         * If the mesh's count is too low, the uv map can't keep a 
         * good radiation shap.  Therefore the maxUVAngle should be
         * reduced to avoid the uv area cutting the radiation circle.
        **/
        int uvAngle = 360 / count;
        if (uvAngle > 30)
            maxUVAngle = 30;
        else
            maxUVAngle = uvAngle;
    }

    private int Count = -1, verticesCount = -1, indicesCount = -1;

    public List<Vector3> vertices;
    public List<Vector2> uvs;
    public List<Vector3> normals;
    public List<int> indices;
    public List<Color32> colors;
    public Vector3 position;

    private Mesh emptyMesh;
    private Mesh updateMesh;

    private Color32 StringToColor32(string color_string)
    {
        byte[] _color_r = BitConverter.GetBytes (Convert.ToInt32 (color_string.Substring (1, 2), 16));
        byte[] _color_g = BitConverter.GetBytes (Convert.ToInt32 (color_string.Substring (3, 2), 16));
        byte[] _color_b = BitConverter.GetBytes (Convert.ToInt32 (color_string.Substring (5, 2), 16));
        byte[] _color_a = BitConverter.GetBytes (Convert.ToInt32 (color_string.Substring (7, 2), 16));

        return new Color32 (_color_r[0], _color_g[0], _color_b[0], _color_a[0]);
    }

    private void ReadJsonValues()
    {
        string json_values = WaveVR_Utils.OEMConfig.getControllerConfig ();

        if (!json_values.Equals (""))
        {
            SimpleJSON.JSONNode jsNodes = SimpleJSON.JSONNode.Parse (json_values);

            string node_value = "";
            node_value = jsNodes ["beam"] ["start_width"].Value;
            if (!node_value.Equals(""))
                startWidth = float.Parse (node_value);

            node_value = jsNodes ["beam"] ["end_width"].Value;
            if (!node_value.Equals(""))
                endWidth = float.Parse (node_value);

            node_value = jsNodes ["beam"] ["length"].Value;
            if (!node_value.Equals(""))
                endOffset = float.Parse (node_value);

            node_value = jsNodes ["beam"] ["start_color"].Value;
            if (!node_value.Equals (""))
                StartColor = StringToColor32 (node_value);

            node_value = jsNodes ["beam"] ["end_color"].Value;
            if (!node_value.Equals (""))
                EndColor = StringToColor32 (node_value);

            Log.d ("WaveVR_Beam", "startWidth: " + startWidth + "endWidth: " + endWidth + "endOffset: " + endOffset +
                "StartColor: " + StartColor.ToString () + ", EndColor: " + EndColor.ToString ());
        }
    }

    void OnEnable()
    {
        if (useSystemConfig)
        {
            Log.d(LOG_TAG, "use system config in WaveVR_Beam!");
            ReadJsonValues ();
        }
        else
        {
            Log.w(LOG_TAG, "use custom config in WaveVR_Beam!");
        }
        endOffsetMax = endOffset;

        emptyMesh = new Mesh();
        updateMesh = new Mesh();

        TailColor = StartColor;

        Count = count + 1;
        verticesCount = Count * 2 + (makeTail ? 1 : 0);
        indicesCount = Count * 6 + (makeTail ? count * 3 : 0);

        uvs = new List<Vector2>(verticesCount);
        vertices = new List<Vector3> (verticesCount);
        normals = new List<Vector3>(verticesCount);
        indices = new List<int>(indicesCount);

        Validate();

        var meshfilter = GetComponent<MeshFilter>();
        meshfilter.mesh = createMesh();

        var meshRenderer = GetComponent<MeshRenderer>();
        meshRenderer.enabled = true;
    }

    public void Update()
    {
        var meshfilter = GetComponent<MeshFilter>();

        if (false == enableBeam)
        {
            meshfilter.mesh = emptyMesh;
        } else
        {
            if (!updateEveryFrame)
                return;

            Validate ();

            meshfilter.mesh = createMesh ();
        }
    }

    public void SetEndOffset (Vector3 target, bool interactive)
    {
        Vector3 targetLocalPosition = transform.InverseTransformPoint(target);
        //endOffset = targetLocalPosition.z - 0.5f;
        //endOffset = Mathf.Clamp(endOffset, endOffsetMin, endOffsetMax);
        Debug.Log("targetLocalPosition.z: " + targetLocalPosition.z + ", endOffsetMax: " + endOffsetMax);
        if (targetLocalPosition.z <= endOffsetMax)
            enableBeam = false;
    }

    public void ResetEndOffset()
    {
        endOffset = endOffsetMax;
        enableBeam = true;
    }

    private Mesh createMesh()
    {
        updateMesh.Clear ();
        uvs.Clear ();
        vertices.Clear ();
        normals.Clear ();
        indices.Clear ();
        colors.Clear ();

        Matrix4x4 mat = new Matrix4x4();
        Matrix4x4 matUV = new Matrix4x4();

        for (int i = 0; i < Count; i++)
        {
            int angle = (int) (i * 360.0f / count);
            int UVangle = (int)(i * maxUVAngle / count);
            // make rotation matrix
            mat.SetTRS(new Vector3(0, 0, 0), Quaternion.AngleAxis(angle, new Vector3(0, 0, 1)), new Vector3(1, 1, 1));
            matUV.SetTRS(new Vector3(0, 0, 0), Quaternion.AngleAxis(UVangle, new Vector3(0, 0, 1)), new Vector3(1, 1, 1));

            // start
            vertices.Add(mat.MultiplyVector(new Vector3(0, startWidth, startOffset)));
            uvs.Add(new Vector2(0.5f,0.5f));
            colors.Add (StartColor);
            normals.Add(mat.MultiplyVector(new Vector3(0, 1, 0)).normalized);
            

            // end
            vertices.Add(mat.MultiplyVector(new Vector3(0, endWidth, endOffset)));
            Vector2 uv = matUV.MultiplyVector(new Vector3(0, 0.5f, 0));
            uv.x = uv.x + 0.5f;
            uv.y = uv.y + 0.5f;
            uvs.Add(uv);
            colors.Add (EndColor);
            normals.Add(mat.MultiplyVector(new Vector3(0, 1, 0)).normalized);
        }

        for (int i = 0; i < count; i++)
        {
            // bd
            // ac
            int a, b, c, d;
            a = i * 2;
            b = i * 2 + 1;
            c = i * 2 + 2;
            d = i * 2 + 3;

            // first
            indices.Add(a);
            indices.Add(d);
            indices.Add(b);

            // second
            indices.Add(a);
            indices.Add(c);
            indices.Add(d);
        }

        // Make Tail
        if (makeTail)
        {
            vertices.Add(new Vector3(0, 0, 0));
            colors.Add (TailColor);
            uvs.Add(new Vector2(0.5f, 0.5f));
            normals.Add(new Vector3(0, 0, 0));
            int tailIdx = count * 2;
            for (int i = 0; i < count; i++)
            {
                int idx = i * 2;

                indices.Add(tailIdx);
                indices.Add(idx + 2);
                indices.Add(idx);
            }
        }
        updateMesh.vertices = vertices.ToArray();
        //updateMesh.SetUVs(0, uvs);
        //updateMesh.SetUVs(1, uvs);
        updateMesh.colors32  = colors.ToArray ();
        updateMesh.normals = normals.ToArray();
        updateMesh.SetIndices(indices.ToArray(), MeshTopology.Triangles, 0);
        updateMesh.name = "Beam";

        return updateMesh;
    }
}
