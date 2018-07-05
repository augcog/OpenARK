using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class IndicatorLine : MonoBehaviour {
    [Header("Line customization")]
    [Range(0.0004f, 0.001f)]
    public float startWidth = 0.0004f;    // in x,y axis
    [Range(0.0004f, 0.001f)]
    public float endWidth = 0.0004f;       // let the bean seems the same width in far distance.
    [Range(0.02f, 0.2f)]
    public float lineLength = 0.03f;
    public Color lineColor = Color.white;
    public ButtonIndication.Alignment alignment = ButtonIndication.Alignment.RIGHT;

    private float startOffset = 0f;
    private float endOffset = 0.03f;

    private int count = 12;
    private bool makeTail = true; // Offset from 0

    private int maxUVAngle = 30;

    private void Validate()
    {
        endOffset = lineLength;
        if (alignment == ButtonIndication.Alignment.LEFT)
        {
            endOffset = endOffset * (-1.0f);
        }

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

    private List<Vector3> vertices = new List<Vector3>();
    private List<Vector2> uvs = new List<Vector2>();
    private List<Vector3> normals = new List<Vector3>();
    private List<int> indices = new List<int>();
    private List<Color32> colors = new List<Color32>();
    private Vector3 position;

    private Mesh updateMesh;

    void OnEnable()
    {
        updateMesh = new Mesh();

        Count = count + 1;
        verticesCount = Count * 2 + (makeTail ? 1 : 0);
        indicesCount = Count * 6 + (makeTail ? count * 3 : 0);

        uvs = new List<Vector2>(verticesCount);
        vertices = new List<Vector3>(verticesCount);
        normals = new List<Vector3>(verticesCount);
        indices = new List<int>(indicesCount);

        Validate();
    }

    void Start()
    {
        updateMeshSettings();
    }

    public void Update()
    {

    }

    public void Circle(Texture2D tex, int cx, int cy, int r, Color col)
    {
        int x, y, px, nx, py, ny, d;

        for (x = 0; x <= r; x++)
        {
            d = (int)Mathf.Ceil(Mathf.Sqrt(r * r - x * x));
            for (y = 0; y <= d; y++)
            {
                px = cx + x;
                nx = cx - x;
                py = cy + y;
                ny = cy - y;

                tex.SetPixel(px, py, col);
                tex.SetPixel(nx, py, col);

                tex.SetPixel(px, ny, col);
                tex.SetPixel(nx, ny, col);
            }
        }
        tex.Apply();
    }

    public void updateMeshSettings()
    {
        Debug.Log("updateMeshSettings");

        Validate();

        var meshfilter = GetComponent<MeshFilter>();
        meshfilter.mesh = createMesh();

        var meshRenderer = GetComponent<MeshRenderer>();
        meshRenderer.enabled = true;
        meshRenderer.material = new Material(Shader.Find("Unlit/Texture"));

        var texture = new Texture2D(256, 256, TextureFormat.ARGB32, false);
        Color o = lineColor;
        for (int i = 0; i < 256; i++)
        {
            for (int j = 0; j < 256; j++)
            {
                texture.SetPixel(i, j, o);
            }
        }
        texture.Apply();

        meshRenderer.material.mainTexture = texture;
    }

    private Mesh createMesh()
    {
        updateMesh.Clear();
        uvs.Clear();
        vertices.Clear();
        normals.Clear();
        indices.Clear();
        colors.Clear();

        Matrix4x4 mat = new Matrix4x4();
        Matrix4x4 matUV = new Matrix4x4();
        Debug.Log("Count: " + Count + ", count: " + count);
        for (int i = 0; i < Count; i++)
        {
            int angle = (int)(i * 360.0f / count);
            int UVangle = (int)(i * maxUVAngle / count);
            // make rotation matrix
            mat.SetTRS(new Vector3(0, 0, 0), Quaternion.AngleAxis(angle, new Vector3(1, 0, 0)), new Vector3(1, 1, 1));
            matUV.SetTRS(new Vector3(0, 0, 0), Quaternion.AngleAxis(UVangle, new Vector3(1, 0, 0)), new Vector3(1, 1, 1));

            // start
            vertices.Add(mat.MultiplyVector(new Vector3(startOffset, 0, startWidth)));
            uvs.Add(new Vector2(0.5f, 0.5f));
            colors.Add(lineColor);
            normals.Add(mat.MultiplyVector(new Vector3(0, 1, 0)).normalized);

            // end
            vertices.Add(mat.MultiplyVector(new Vector3(endOffset, 0, endWidth)));
            Vector2 uv = matUV.MultiplyVector(new Vector3(0, 0.5f, 0));
            uv.x = uv.x + 0.5f;
            uv.y = uv.y + 0.5f;
            uvs.Add(uv);
            colors.Add(lineColor);
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
            colors.Add(lineColor);
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
        updateMesh.colors32 = colors.ToArray();
        updateMesh.normals = normals.ToArray();
        updateMesh.SetIndices(indices.ToArray(), MeshTopology.Triangles, 0);
        updateMesh.name = "IndicatorLine";

        return updateMesh;
    }
}
