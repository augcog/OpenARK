using UnityEngine;
using UnityEditor;

public class WaveVR_Preferences
{
	/// <summary>
	/// Should SteamVR automatically enable VR when opening Unity or pressing play.
	/// </summary>
	public static bool AutoEnableVR
	{
		get
		{
			return EditorPrefs.GetBool("WaveVR_AutoEnableVR", true);
		}
		set
		{
			EditorPrefs.SetBool("WaveVR_AutoEnableVR", value);
		}
	}

    /*
    [PreferenceItem("WaveVR")]
    static void PreferencesGUI()
    {
        EditorGUILayout.BeginVertical();
        EditorGUILayout.Space();

        // Automatically Enable VR
        {
            string title = "Automatically Enable VR";
            string tooltip = "Should WaveVR automatically enable VR on launch and play?";
            AutoEnableVR = EditorGUILayout.Toggle(new GUIContent(title, tooltip), AutoEnableVR);
            string helpMessage = "To enable VR manually:\n";
            helpMessage += "- go to Edit -> Project Settings -> Player,\n";
            helpMessage += "- tick 'Virtual Reality Supported',\n";
            helpMessage += "- make sure WaveVR is in the 'Virtual Reality SDKs' list.";
            EditorGUILayout.HelpBox(helpMessage, MessageType.Info);
        }

    EditorGUILayout.EndVertical();
	}
    */
}

