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

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Beam: MonoBehaviour
{
    public float startWidth = 0.01f;  // in x,y axis
    public float endWidth = 0.02f;  // let the bean seems the same width in far distance.
    public float startOffset = 0.03f;
    public float endOffset = 10f;

    public int count = 3;
    public bool updateEveryFrame = false;
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

    void OnEnable()
    {
        Validate();

        var meshfilter = GetComponent<MeshFilter>();
        meshfilter.mesh = createMesh();

        var meshRenderer = GetComponent<MeshRenderer>();
        meshRenderer.enabled = true;
    }

    public List<Vector3> vertices;
    public List<Vector2> uvs;
    public List<Vector3> normals;
    public List<int> indices;
    public Vector3 position;

    public void Update()
    {
        if (!updateEveryFrame)
            return;

        Validate();

        var meshfilter = GetComponent<MeshFilter>();
        meshfilter.mesh = createMesh();
    }

    private Mesh createMesh()
    {
        Mesh mesh = new Mesh();
        int Count = count + 1;
        int verticesCount = Count * 2 + (makeTail ? 1 : 0);
        int indicesCount = Count * 6 + (makeTail ? count * 3 : 0);

        vertices = new List<Vector3>(verticesCount);
        uvs = new List<Vector2>(verticesCount);
        normals = new List<Vector3>(verticesCount);
        indices = new List<int>(indicesCount);

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
            normals.Add(mat.MultiplyVector(new Vector3(0, 1, 0)).normalized);
            

            // end
            vertices.Add(mat.MultiplyVector(new Vector3(0, endWidth, endOffset)));
            Vector2 uv = matUV.MultiplyVector(new Vector3(0, 0.5f, 0));
            uv.x = uv.x + 0.5f;
            uv.y = uv.y + 0.5f;
            uvs.Add(uv);
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
        mesh.vertices = vertices.ToArray();
        mesh.SetUVs(0, uvs);
        mesh.SetUVs(1, uvs);
        mesh.normals = normals.ToArray();
        mesh.SetIndices(indices.ToArray(), MeshTopology.Triangles, 0);
        mesh.name = "Beam";
        return mesh;
    }
}
