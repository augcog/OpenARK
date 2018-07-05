// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

#if true
using UnityEngine;
using UnityEditor;
using System.IO;

[CustomEditor(typeof(WaveVR_Render)), CanEditMultipleObjects]
public class WaveVR_RenderEditor : Editor
{
    private int bannerHeightMax = 150;
    Texture logo = null;

    SerializedProperty script;
    SerializedProperty CameraGaze;
    //SerializedProperty useCompositor;
    //SerializedProperty useSingleBuffer;
    //SerializedProperty targetFPS;
    //SerializedProperty prediction;
    //SerializedProperty textureExpand;
    SerializedProperty _origin;
    //SerializedProperty useATW;
    SerializedProperty needTimeControl;

    string GetResourcePath()
    {
        var ms = MonoScript.FromScriptableObject(this);
        var path = AssetDatabase.GetAssetPath(ms);
        path = Path.GetDirectoryName(path);
        return path.Substring(0, path.Length - "Editor".Length) + "Textures/";
    }

    void OnEnable()
    {
        var resourcePath = GetResourcePath();
#if UNITY_5_0
		logo = Resources.LoadAssetAtPath<Texture2D>(resourcePath + "vivewave_logo_flat.png");
#else
        logo = AssetDatabase.LoadAssetAtPath<Texture2D>(resourcePath + "vivewave_logo_flat.png");
#endif
        script = serializedObject.FindProperty("m_Script");
        CameraGaze =  serializedObject.FindProperty("CameraGaze");
        //useCompositor = serializedObject.FindProperty("useCompositor");
        //useSingleBuffer = serializedObject.FindProperty("useSingleBuffer");
        //prediction = serializedObject.FindProperty("predict");
        //targetFPS = serializedObject.FindProperty("targetFPS");
        //textureExpand = serializedObject.FindProperty("textureExpand");
        _origin = serializedObject.FindProperty("_origin");
        needTimeControl = serializedObject.FindProperty("needTimeControl");
        //useATW = serializedObject.FindProperty("useATW");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        if (logo)
        {
            // Logo need have aspect rate 2:1
            int bannerWidth, bannerHeight;
            bannerWidth = Screen.width - 35;
            bannerHeight = (int) (bannerWidth / (float) 2);
            if (bannerHeight > bannerHeightMax)
            {
                bannerHeight = bannerHeightMax;
                bannerWidth = bannerHeight * 2;
            }
            var rect = GUILayoutUtility.GetRect(bannerWidth, bannerHeight, GUI.skin.box);
            GUI.DrawTexture(rect, logo, ScaleMode.ScaleToFit);
        }
        EditorGUILayout.PropertyField(script);

        EditorGUILayout.PropertyField(CameraGaze);

        //EditorGUILayout.PropertyField(useSingleBuffer);
        //EditorGUILayout.PropertyField(useCompositor);
        //EditorGUILayout.PropertyField(prediction);
        //EditorGUILayout.PropertyField(textureExpand);
        //EditorGUILayout.PropertyField(targetFPS);
        EditorGUILayout.PropertyField(_origin);
        EditorGUILayout.PropertyField(needTimeControl);
        //EditorGUILayout.PropertyField(useATW);

        if (!Application.isPlaying)
        {
            var expand = false;
            var collapse = false;

            foreach (WaveVR_Render target in targets)
            {
                if (AssetDatabase.Contains(target))
                    continue;
                if (target.isExpanded)
                    collapse = true;
                else
                    expand = true;
            }

            if (expand)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Expand"))
                {
                    foreach (WaveVR_Render target in targets)
                    {
                        if (AssetDatabase.Contains(target))
                            continue;
                        if (!target.isExpanded)
                        {
                            WaveVR_Render.Expand(target);
                            EditorUtility.SetDirty(target);
                        }
                    }
                }
                GUILayout.Space(18);
                GUILayout.EndHorizontal();
            }

            if (collapse)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Collapse"))
                {
                    foreach (WaveVR_Render target in targets)
                    {
                        if (AssetDatabase.Contains(target))
                            continue;
                        if (target.isExpanded)
                        {
                            WaveVR_Render.Collapse(target);
                            EditorUtility.SetDirty(target);
                        }
                    }
                }
                GUILayout.Space(18);
                GUILayout.EndHorizontal();
            }
        }
        serializedObject.ApplyModifiedProperties();
    }
}
#endif