using UnityEngine;
using UnityEditor;
using System.IO;

[InitializeOnLoad]
public class WaveVR_Settings : EditorWindow
{
    const bool forceShow = false; // Set to true to get the dialog to show back up in the case you clicked Ignore All.

    const string ignore = "ignore.";
    const string useRecommended = "Use recommended ({0})";
    const string currentValue = " (current = {0})";

    const string buildTarget = "Build Target";
    const string defaultOrientation = "Default orientation";
    const string enableMTRendering = "Enable multi-threading rendering";
    const string graphicsJobs = "Enable Graphics Jobs ";
    const string gpuSkinning = "GPU Skinning";
    const string AndroidMinSDK = "Android Min SDK version";
    const string AndroidTargetSDK = "Android Target SDK version";
    const BuildTarget recommended_BuildTarget = BuildTarget.Android;
    const UIOrientation recommended_defaultOrientation = UIOrientation.LandscapeLeft;
    const bool recommended_enableMTRendering = true;
    const bool recommended_graphicsJobs = true;
    const bool recommended_GpuSkinning = false;
    const AndroidSdkVersions recommended_AndroidMinSDK = AndroidSdkVersions.AndroidApiLevel25;
    const AndroidSdkVersions recommended_AndroidTargetSDK = AndroidSdkVersions.AndroidApiLevel25;

    static WaveVR_Settings window;

    static WaveVR_Settings()
    {
        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android)
            return;
        EditorApplication.update += Update;
    }

    [UnityEditor.MenuItem("WaveVR/Preference/DefaultPreferenceDialog")]
    static void UpdateWithClearIgnore()
    {
        EditorPrefs.DeleteKey(ignore + buildTarget);
        EditorPrefs.DeleteKey(ignore + defaultOrientation);
        EditorPrefs.DeleteKey(ignore + enableMTRendering);
        EditorPrefs.DeleteKey(ignore + graphicsJobs);
        EditorPrefs.DeleteKey(ignore + AndroidMinSDK);
        EditorPrefs.DeleteKey(ignore + AndroidTargetSDK);
        EditorPrefs.DeleteKey(ignore + gpuSkinning);
        Update();
    }

    static void Update()
    {
        bool show =
           (!EditorPrefs.HasKey(ignore + buildTarget) &&
               EditorUserBuildSettings.activeBuildTarget != recommended_BuildTarget) ||
           (!EditorPrefs.HasKey(ignore + defaultOrientation) &&
               PlayerSettings.defaultInterfaceOrientation != recommended_defaultOrientation) ||
           (!EditorPrefs.HasKey(ignore + enableMTRendering) &&
#if UNITY_2017_2_OR_NEWER
                PlayerSettings.GetMobileMTRendering(BuildTargetGroup.Android) != recommended_enableMTRendering) ||
#else
                PlayerSettings.mobileMTRendering != recommended_enableMTRendering) ||
#endif
            (!EditorPrefs.HasKey(ignore + graphicsJobs) &&
               PlayerSettings.graphicsJobs != recommended_graphicsJobs) ||
           (!EditorPrefs.HasKey(ignore + AndroidMinSDK) &&
               PlayerSettings.Android.minSdkVersion < recommended_AndroidMinSDK) ||
           (!EditorPrefs.HasKey(ignore + AndroidTargetSDK) &&
               PlayerSettings.Android.targetSdkVersion != recommended_AndroidTargetSDK) ||
           (!EditorPrefs.HasKey(ignore + gpuSkinning) &&
               PlayerSettings.gpuSkinning != recommended_GpuSkinning) ||
           forceShow;

        if (show)
        {
            window = GetWindow<WaveVR_Settings>(true);
            window.minSize = new Vector2(640, 320);
        }
        EditorApplication.update -= Update;
    }

    Vector2 scrollPosition;

    string GetResourcePath()
    {
        var ms = MonoScript.FromScriptableObject(this);
        var path = AssetDatabase.GetAssetPath(ms);
        path = Path.GetDirectoryName(path);
        return path.Substring(0, path.Length - "Editor".Length) + "Textures/";
    }

    public void OnGUI()
    {
        var resourcePath = GetResourcePath();
        var logo = AssetDatabase.LoadAssetAtPath<Texture2D>(resourcePath + "vivewave_logo_flat.png");
        var rect = GUILayoutUtility.GetRect(position.width, 150, GUI.skin.box);
        if (logo)
            GUI.DrawTexture(rect, logo, ScaleMode.ScaleToFit);

        EditorGUILayout.HelpBox("Recommended project settings for WaveVR:", MessageType.Warning);

        scrollPosition = GUILayout.BeginScrollView(scrollPosition);

        int numItems = 0;

        if (!EditorPrefs.HasKey(ignore + buildTarget) &&
            EditorUserBuildSettings.activeBuildTarget != recommended_BuildTarget)
        {
            ++numItems;

            GUILayout.Label(buildTarget + string.Format(currentValue, EditorUserBuildSettings.activeBuildTarget));

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, recommended_BuildTarget)))
            {
                EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, recommended_BuildTarget);
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + buildTarget, true);
            }

            GUILayout.EndHorizontal();
        }

        if (!EditorPrefs.HasKey(ignore + defaultOrientation) &&
            PlayerSettings.defaultInterfaceOrientation != recommended_defaultOrientation)
        {
            ++numItems;

            GUILayout.Label(defaultOrientation + string.Format(currentValue, PlayerSettings.defaultInterfaceOrientation));

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, recommended_defaultOrientation)))
            {
                PlayerSettings.defaultInterfaceOrientation = recommended_defaultOrientation;
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + defaultOrientation, true);
            }

            GUILayout.EndHorizontal();
        }

        if (!EditorPrefs.HasKey(ignore + enableMTRendering) &&
#if UNITY_2017_2_OR_NEWER
                PlayerSettings.GetMobileMTRendering(BuildTargetGroup.Android) != recommended_enableMTRendering)
#else
                PlayerSettings.mobileMTRendering != recommended_enableMTRendering)
#endif
        {
            ++numItems;

#if UNITY_2017_2_OR_NEWER
            GUILayout.Label(enableMTRendering + string.Format(currentValue, PlayerSettings.GetMobileMTRendering(BuildTargetGroup.Android)));
#else
            GUILayout.Label(enableMTRendering + string.Format(currentValue, PlayerSettings.mobileMTRendering));
#endif

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, enableMTRendering)))
            {
#if UNITY_2017_2_OR_NEWER
                PlayerSettings.SetMobileMTRendering(BuildTargetGroup.Android, recommended_enableMTRendering);
#else
                PlayerSettings.mobileMTRendering = recommended_enableMTRendering;
#endif
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + enableMTRendering, true);
            }

            GUILayout.EndHorizontal();
        }

        if (!EditorPrefs.HasKey(ignore + graphicsJobs) &&
            PlayerSettings.graphicsJobs != recommended_graphicsJobs)
        {
            ++numItems;

            GUILayout.Label(graphicsJobs + string.Format(currentValue, PlayerSettings.graphicsJobs));

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, recommended_graphicsJobs)))
            {
                PlayerSettings.graphicsJobs = recommended_graphicsJobs;
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + graphicsJobs, true);
            }

            GUILayout.EndHorizontal();
        }

        if (!EditorPrefs.HasKey(ignore + AndroidMinSDK) &&
            PlayerSettings.Android.minSdkVersion != recommended_AndroidMinSDK)
        {
            ++numItems;

            GUILayout.Label(AndroidMinSDK + string.Format(currentValue, PlayerSettings.Android.minSdkVersion));

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, recommended_AndroidMinSDK)))
            {
                PlayerSettings.Android.minSdkVersion = recommended_AndroidMinSDK;
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + AndroidMinSDK, true);
            }

            GUILayout.EndHorizontal();
        }

        if (!EditorPrefs.HasKey(ignore + AndroidTargetSDK) &&
            PlayerSettings.Android.targetSdkVersion != recommended_AndroidTargetSDK)
        {
            ++numItems;

            GUILayout.Label(AndroidTargetSDK + string.Format(currentValue, PlayerSettings.Android.targetSdkVersion));

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, recommended_AndroidTargetSDK)))
            {
                PlayerSettings.Android.targetSdkVersion = recommended_AndroidTargetSDK;
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + AndroidTargetSDK, true);
            }

            GUILayout.EndHorizontal();
        }

        if (!EditorPrefs.HasKey(ignore + gpuSkinning) &&
            PlayerSettings.gpuSkinning != recommended_GpuSkinning)
        {
            ++numItems;

            GUILayout.Label(gpuSkinning + string.Format(currentValue, PlayerSettings.gpuSkinning));

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(string.Format(useRecommended, recommended_GpuSkinning)))
            {
                PlayerSettings.gpuSkinning = recommended_GpuSkinning;
            }

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("Ignore"))
            {
                EditorPrefs.SetBool(ignore + gpuSkinning, true);
            }

            GUILayout.EndHorizontal();
        }



        GUILayout.BeginHorizontal();

        GUILayout.FlexibleSpace();

        if (GUILayout.Button("Clear All Ignores"))
        {
            EditorPrefs.DeleteKey(ignore + buildTarget);
            EditorPrefs.DeleteKey(ignore + defaultOrientation);
            EditorPrefs.DeleteKey(ignore + enableMTRendering);
            EditorPrefs.DeleteKey(ignore + graphicsJobs);
            EditorPrefs.DeleteKey(ignore + AndroidMinSDK);
            EditorPrefs.DeleteKey(ignore + AndroidTargetSDK);
            EditorPrefs.DeleteKey(ignore + gpuSkinning);
        }

        GUILayout.EndHorizontal();

        GUILayout.EndScrollView();

        GUILayout.FlexibleSpace();

        GUILayout.BeginHorizontal();

        if (numItems > 0)
        {
            if (GUILayout.Button("Accept All"))
            {
                // Only set those that have not been explicitly ignored.
                if (!EditorPrefs.HasKey(ignore + buildTarget))
                    EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Standalone, recommended_BuildTarget);

                if (!EditorPrefs.HasKey(ignore + defaultOrientation))
                    PlayerSettings.defaultInterfaceOrientation = recommended_defaultOrientation;

                if (!EditorPrefs.HasKey(ignore + enableMTRendering))
#if UNITY_2017_2_OR_NEWER
                    PlayerSettings.SetMobileMTRendering(BuildTargetGroup.Android, recommended_enableMTRendering);
#else
                    PlayerSettings.mobileMTRendering = recommended_enableMTRendering;
#endif
                if (!EditorPrefs.HasKey(ignore + AndroidMinSDK))
                    PlayerSettings.Android.minSdkVersion = recommended_AndroidMinSDK;
                if (!EditorPrefs.HasKey(ignore + graphicsJobs))
                    PlayerSettings.graphicsJobs = recommended_graphicsJobs;

                if (!EditorPrefs.HasKey(ignore + AndroidTargetSDK))
                    PlayerSettings.Android.targetSdkVersion = recommended_AndroidTargetSDK;

                if (!EditorPrefs.HasKey(ignore + gpuSkinning))
                    PlayerSettings.gpuSkinning = recommended_GpuSkinning;

                EditorUtility.DisplayDialog("Accept All", "You made the right choice!", "Ok");

                Close();
            }

            if (GUILayout.Button("Ignore All"))
            {
                if (EditorUtility.DisplayDialog("Ignore All", "Are you sure?", "Yes, Ignore All", "Cancel"))
                {
                    // Only ignore those that do not currently match our recommended settings.
                    if (EditorUserBuildSettings.activeBuildTarget != recommended_BuildTarget)
                        EditorPrefs.SetBool(ignore + buildTarget, true);

                    if (PlayerSettings.defaultInterfaceOrientation != recommended_defaultOrientation)
                        EditorPrefs.SetBool(ignore + defaultOrientation, true);

#if UNITY_2017_2_OR_NEWER
                    if (PlayerSettings.GetMobileMTRendering(BuildTargetGroup.Android) != recommended_enableMTRendering)
#else
                    if (PlayerSettings.mobileMTRendering != recommended_enableMTRendering)
#endif
                        EditorPrefs.SetBool(ignore + enableMTRendering, true);

                    if (PlayerSettings.graphicsJobs != recommended_graphicsJobs)
                        EditorPrefs.SetBool(ignore + graphicsJobs, true);

                    if (PlayerSettings.Android.minSdkVersion != recommended_AndroidMinSDK)
                        EditorPrefs.SetBool(ignore + AndroidMinSDK, true);

                    if (PlayerSettings.Android.targetSdkVersion != recommended_AndroidTargetSDK)
                        EditorPrefs.SetBool(ignore + AndroidTargetSDK, true);

                    if (PlayerSettings.gpuSkinning != recommended_GpuSkinning)
                        EditorPrefs.SetBool(ignore + gpuSkinning, true);

                    Close();
                }
            }
        }
        else if (GUILayout.Button("Close"))
        {
            Close();
        }

        GUILayout.EndHorizontal();
    }

}
