#if UNITY_EDITOR
using UnityEngine;
using System.IO;
using System;
using UnityEditor.Build;
using UnityEditor;

static class CustomBuildProcessor
{
    private static string version_depended_aar = "wavevr_unity_plugin";
    private static string aar5 = Application.dataPath + "/Plugins/Android/" + version_depended_aar + ".aar";
    private static string aar2017 = Application.dataPath + "/Plugins/Android/" + version_depended_aar + "_2017.aar";
    private static string skip = "";
    private static string recovery = "";
    private class CustomPreprocessor : IPreprocessBuild
    {
        public int callbackOrder { get { return 0; } }

        public void OnPreprocessBuild(BuildTarget target, string path)
        {
            if (target == BuildTarget.Android)
            {
                if (Application.unityVersion.StartsWith("2017."))
                {
                    if (!File.Exists(aar2017) && File.Exists(aar2017+".skip"))
                    {
                        try
                        {
                            File.Move(aar2017+".skip", aar2017);
                            AssetDatabase.Refresh();
                        }
                        catch (Exception e)
                        {
                            Debug.LogWarning("Caught " + e.ToString() + " while moving \"" + aar2017 + ".skip" + "\" to \"" + aar2017 + "\"");
                        }
                    }
                    skip = aar5;
                }
                else
                {
                    if (!File.Exists(aar5) && File.Exists(aar5 + ".skip"))
                    {
                        try
                        {
                            File.Move(aar5 + ".skip", aar5);
                            AssetDatabase.Refresh();
                        }
                        catch (Exception e)
                        {
                            Debug.LogWarning("Caught " + e.ToString() + " while moving \"" + aar5 + ".skip" + "\" to \"" + aar5 + "\"");
                        }
                    }
                    skip = aar2017;
                }
                recovery = skip + ".skip";
                if (File.Exists(skip))
                {
                    //Debug.Log("Skip " + skip);
                    try
                    {
                        File.Move(skip, recovery);
                        AssetDatabase.Refresh();
                    }
                    catch (Exception e)
                    {
                        Debug.LogWarning("Caught " + e.ToString() + " while moving \"" + skip + "\" to \"" + recovery +"\"");
                    }
                }
            }
        }
    }

    private class CustomPostprocessor : IPostprocessBuild
    {
        public int callbackOrder { get { return 0; } }
        public void OnPostprocessBuild(BuildTarget target, string path)
        {
            if (target == BuildTarget.Android)
            {
                if (File.Exists(recovery) && !File.Exists(skip))
                {
                    //Debug.Log("Recover " + skip);
                    try
                    {
                        File.Move(recovery, skip);
                        AssetDatabase.Refresh();
                        skip = "";
                        recovery = "";
                    }
                    catch (Exception e)
                    {
                        Debug.LogWarning("Caught " + e.ToString() + " while moving \"" + recovery + "\" to \"" + skip + "\"");
                    }
                }
            }
        }
    }
}
#endif
