using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;
using WaveVR_Log;
using System;
using System.Runtime.InteropServices;
using UnityEngine.EventSystems;

#if UNITY_EDITOR
using UnityEditor;

[CustomEditor(typeof(WaveVR_ControllerLoader))]
public class WaveVR_ControllerLoaderEditor : Editor
{
    public override void OnInspectorGUI()
    {
        WaveVR_ControllerLoader myScript = target as WaveVR_ControllerLoader;

        myScript.WhichHand = (WaveVR_ControllerLoader.ControllerHand)EditorGUILayout.EnumPopup ("Type", myScript.WhichHand);
        myScript.ControllerComponents = (WaveVR_ControllerLoader.CComponent)EditorGUILayout.EnumPopup ("Controller Components", myScript.ControllerComponents);

        myScript.TrackPosition = EditorGUILayout.Toggle ("Track Position", myScript.TrackPosition);
        if (true == myScript.TrackPosition)
        {
            myScript.SimulationOption = (WVR_SimulationOption)EditorGUILayout.EnumPopup ("    Simulate Position", myScript.SimulationOption);
            if (myScript.SimulationOption == WVR_SimulationOption.ForceSimulation || myScript.SimulationOption == WVR_SimulationOption.WhenNoPosition)
            {
                myScript.FollowHead = (bool)EditorGUILayout.Toggle ("        Follow Head", myScript.FollowHead);
            }
        }

        myScript.TrackRotation = EditorGUILayout.Toggle ("Track Rotation", myScript.TrackRotation);

        EditorGUILayout.LabelField("Indication feature");
        myScript.overwriteIndicatorSettings = EditorGUILayout.Toggle("Overwrite Indicator Settings", myScript.overwriteIndicatorSettings);
        if (true == myScript.overwriteIndicatorSettings)
        {
            myScript.showIndicator = EditorGUILayout.Toggle("Show Indicator", myScript.showIndicator);
            if (true == myScript.showIndicator)
            {
                myScript.hideIndicatorByRoll = EditorGUILayout.Toggle("Hide Indicator when roll angle > 90 ", myScript.hideIndicatorByRoll);
                myScript.showIndicatorAngle = EditorGUILayout.FloatField("Show When Angle > ", myScript.showIndicatorAngle);
                EditorGUILayout.Space();

                EditorGUILayout.LabelField("Line customization");
                myScript.lineLength = EditorGUILayout.FloatField("Line Length", myScript.lineLength);
                myScript.lineStartWidth = EditorGUILayout.FloatField("Line Start Width", myScript.lineStartWidth);
                myScript.lineEndWidth = EditorGUILayout.FloatField("Line End Width", myScript.lineEndWidth);
                myScript.lineColor = EditorGUILayout.ColorField("Line Color", myScript.lineColor);
                EditorGUILayout.Space();

                EditorGUILayout.LabelField("Text customization");
                myScript.textCharacterSize = EditorGUILayout.FloatField("Text Character Size", myScript.textCharacterSize);
                myScript.zhCharactarSize = EditorGUILayout.FloatField("Chinese Character Size", myScript.zhCharactarSize);
                myScript.textFontSize = EditorGUILayout.IntField("Text Font Size", myScript.textFontSize);
                myScript.textColor = EditorGUILayout.ColorField("Text Color", myScript.textColor);
                EditorGUILayout.Space();

                EditorGUILayout.LabelField("Key indication");
                var list = myScript.buttonIndicationList;

                int newCount = Mathf.Max(0, EditorGUILayout.IntField("Button indicator size", list.Count));

                while (newCount < list.Count)
                    list.RemoveAt(list.Count - 1);
                while (newCount > list.Count)
                    list.Add(new ButtonIndication());

                for (int i = 0; i < list.Count; i++)
                {
                    EditorGUILayout.LabelField("Button indication " + i);
                    myScript.buttonIndicationList[i].keyType = (ButtonIndication.KeyIndicator)EditorGUILayout.EnumPopup("Key Type", myScript.buttonIndicationList[i].keyType);
                    myScript.buttonIndicationList[i].alignment = (ButtonIndication.Alignment)EditorGUILayout.EnumPopup("Alignment", myScript.buttonIndicationList[i].alignment);
                    myScript.buttonIndicationList[i].indicationOffset = EditorGUILayout.Vector3Field("Indication offset", myScript.buttonIndicationList[i].indicationOffset);
                    myScript.buttonIndicationList[i].indicationText = EditorGUILayout.TextField("Indication text", myScript.buttonIndicationList[i].indicationText);
                    myScript.buttonIndicationList[i].followButtonRotation = EditorGUILayout.Toggle("Follow button rotation", myScript.buttonIndicationList[i].followButtonRotation);
                    EditorGUILayout.Space();
                }
            }
        }

        if (GUI.changed)
            EditorUtility.SetDirty ((WaveVR_ControllerLoader)target);
    }
}
#endif

public class WaveVR_ControllerLoader : MonoBehaviour {
    private static string LOG_TAG = "WaveVR_ControllerLoader";
    private void PrintDebugLog(string msg)
    {
        #if UNITY_EDITOR
        Debug.Log(LOG_TAG + "  Hand: " + WhichHand + ", " + msg);
        #endif
        Log.d (LOG_TAG, "Hand: " + WhichHand + ", " + msg);
    }

    private void PrintInfoLog(string msg)
    {
        #if UNITY_EDITOR
        PrintDebugLog(msg);
        #endif
        Log.i (LOG_TAG, "Hand: " + WhichHand + ", " + msg);
    }

    public enum ControllerHand
    {
        Controller_Right,
        Controller_Left
    };

    public enum CComponent
    {
        One_Bone,
        Multi_Component
    };

    public enum CTrackingSpace
    {
        REAL_POSITION_ONLY,
        FAKE_POSITION_ONLY,
        AUTO_POSITION_ONLY,
        ROTATION_ONLY,
        ROTATION_AND_REAL_POSITION,
        ROTATION_AND_FAKE_POSITION,
        ROTATION_AND_AUTO_POSITION,
        CTS_SYSTEM
    };

    [Header("Loading options")]
    public ControllerHand WhichHand = ControllerHand.Controller_Right;
    public CComponent ControllerComponents = CComponent.Multi_Component;
    public bool TrackPosition = true;
    public WVR_SimulationOption SimulationOption = WVR_SimulationOption.WhenNoPosition;
    public bool FollowHead = false;
    public bool TrackRotation = true;

    [Header("Indication feature")]
    public bool overwriteIndicatorSettings = true;
    public bool showIndicator = false;
    public bool hideIndicatorByRoll = true;

    [Range(0, 90.0f)]
    public float showIndicatorAngle = 30.0f;

    [Header("Line customization")]
    [Range(0.01f, 0.1f)]
    public float lineLength = 0.03f;
    [Range(0.0001f, 0.1f)]
    public float lineStartWidth = 0.0004f;
    [Range(0.0001f, 0.1f)]
    public float lineEndWidth = 0.0004f;
    public Color lineColor = Color.white;

    [Header("Text customization")]
    [Range(0.01f, 0.2f)]
    public float textCharacterSize = 0.08f;
    [Range(0.01f, 0.2f)]
    public float zhCharactarSize = 0.07f;
    [Range(50, 200)]
    public int textFontSize = 100;
    public Color textColor = Color.white;

    [Header("Indications")]
    public List<ButtonIndication> buttonIndicationList = new List<ButtonIndication>();

    private GameObject controllerPrefab = null;
    private GameObject originalControllerPrefab = null;
    private string controllerFileName = "";
    private string controllerModelFoler = "Controller/";
    private string genericControllerFileName = "Generic_";

    private WVR_DeviceType deviceType = WVR_DeviceType.WVR_DeviceType_Controller_Right;
    private bool connected = false;
#if UNITY_EDITOR
    public delegate void ControllerModelLoaded(GameObject go);
    public static event ControllerModelLoaded onControllerModelLoaded = null;
#endif

    void OnEnable()
    {
        controllerPrefab = null;
        controllerFileName = "";
        genericControllerFileName = "Generic_";
        if (WhichHand == ControllerHand.Controller_Right)
        {
            deviceType = WVR_DeviceType.WVR_DeviceType_Controller_Right;
        }
        else
        {
            deviceType = WVR_DeviceType.WVR_DeviceType_Controller_Left;
        }
#if UNITY_EDITOR
        if (Application.isPlaying)
        {
            WVR_DeviceType _type = WaveVR_Controller.Input(this.deviceType).DeviceType;
            onLoadController(_type);
            return;
        }
#endif

        WaveVR_Utils.Event.Listen(WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
    }

    void OnDisable()
    {
#if UNITY_EDITOR
        if (Application.isPlaying)
        {
            return;
        }
#endif
        WaveVR_Utils.Event.Remove(WaveVR_Utils.Event.DEVICE_CONNECTED, onDeviceConnected);
    }
    // Use this for initialization
    void Start()
    {
        if (checkConnection () != connected)
            connected = !connected;

        if (connected)
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.deviceType);
            onLoadController (_device.type);
        }

        WaveVR_EventSystemControllerProvider.Instance.MarkControllerLoader (deviceType, true);
    }

    private void onDeviceConnected(params object[] args)
    {
        bool _connected = false;
        WVR_DeviceType _type = this.deviceType;

        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            _connected = WaveVR_Controller.Input (this.deviceType).connected;
            _type = WaveVR_Controller.Input(this.deviceType).DeviceType;
        }
        else
        #endif
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.deviceType);
            _connected = _device.connected;
            _type = _device.type;
        }

        PrintDebugLog ("onDeviceConnected() " + _type + " is " + (_connected ? "connected" : "disconnected") + ", left-handed? " + WaveVR_Controller.IsLeftHanded);

        if (connected != _connected)
        {
            connected = _connected;
        }

        if (connected)
        {
            if (controllerPrefab == null) onLoadController (_type);
        }
    }

    private void onLoadController(WVR_DeviceType type)
    {
        // Make up file name
        // Rule =
        // ControllerModel_TrackingMethod_CComponent_Hand
#if UNITY_EDITOR
        if (Application.isPlaying)
        {
            genericControllerFileName = "Generic_";

            genericControllerFileName += "MC_";

            if (WhichHand == ControllerHand.Controller_Right)
            {
                genericControllerFileName += "R";
            }
            else
            {
                genericControllerFileName += "L";
            }

            originalControllerPrefab = Resources.Load(controllerModelFoler + genericControllerFileName) as GameObject;
            if (originalControllerPrefab == null)
            {
                PrintDebugLog("Cant load generic controller model, Please check file under Resources/" + controllerModelFoler + genericControllerFileName + ".prefab is exist!");
            }
            else
            {
                PrintDebugLog(genericControllerFileName + " controller model is found!");
                SetControllerOptions (originalControllerPrefab);
                controllerPrefab = Instantiate(originalControllerPrefab);
                controllerPrefab.transform.parent = this.transform.parent;

                PrintDebugLog("Controller model loaded");
                ApplyIndicatorParameters();
                if (onControllerModelLoaded != null)
                {
                    PrintDebugLog("trigger delegate");
                    onControllerModelLoaded(controllerPrefab);
                }

                WaveVR_EventSystemControllerProvider.Instance.SetControllerModel(deviceType, controllerPrefab);
            }
            return;
        }
#endif
        string parameterName = "GetRenderModelName";
        IntPtr ptrParameterName = Marshal.StringToHGlobalAnsi(parameterName);

        IntPtr ptrResult = Marshal.AllocHGlobal(30);
        uint resultVertLength = 30;

        Interop.WVR_GetParameters (type, ptrParameterName, ptrResult, resultVertLength);

        string renderModelName = Marshal.PtrToStringAnsi(ptrResult);

        PrintInfoLog("get controller id from runtime is " + renderModelName);

        controllerFileName += renderModelName;
        controllerFileName += "_";

        if (ControllerComponents == CComponent.Multi_Component)
        {
            controllerFileName += "MC_";
        }
        else
        {
            controllerFileName += "OB_";
        }

        if (WhichHand == ControllerHand.Controller_Right)
        {
            controllerFileName += "R";
        }
        else
        {
            controllerFileName += "L";
        }

        PrintInfoLog("controller file name is " + controllerFileName);

        originalControllerPrefab = Resources.Load(controllerModelFoler + controllerFileName) as GameObject;
        var found = true;

        if (originalControllerPrefab == null)
        {
            if (WhichHand == ControllerHand.Controller_Right)
            {
                genericControllerFileName += "MC_R";
            }
            else
            {
                genericControllerFileName += "MC_L";
            }
            Log.w(LOG_TAG, "cant find preferred controller model, load generic controller : " + genericControllerFileName);
            PrintInfoLog("Please download controller model from .... to have better experience!");
            originalControllerPrefab = Resources.Load(controllerModelFoler + genericControllerFileName) as GameObject;
            if (originalControllerPrefab == null)
            {
                Log.e(LOG_TAG, "Cant load generic controller model, Please check file under Resources/" + controllerModelFoler + genericControllerFileName + ".prefab is exist!");
                found = false;
            } else
            {
                PrintInfoLog(genericControllerFileName + " controller model is found!");
            }
        } else
        {
            PrintInfoLog(controllerFileName + " controller model is found!");
        }

        if (found)
        {
            SetControllerOptions (originalControllerPrefab);
            controllerPrefab = Instantiate (originalControllerPrefab);
            controllerPrefab.transform.parent = this.transform.parent;
            ApplyIndicatorParameters();

            WaveVR_Utils.Event.Send(WaveVR_Utils.Event.CONTROLLER_MODEL_LOADED, deviceType, controllerPrefab);
            WaveVR_EventSystemControllerProvider.Instance.SetControllerModel (deviceType, controllerPrefab);
        }
        Marshal.FreeHGlobal(ptrParameterName);
        Marshal.FreeHGlobal(ptrResult);
    }

    private void SetControllerOptions(GameObject controller_prefab)
    {
        WaveVR_PoseTrackerManager _ptm = controller_prefab.GetComponent<WaveVR_PoseTrackerManager> ();
        if (_ptm != null)
        {
            _ptm.TrackPosition = TrackPosition;
            _ptm.SimulationOption = SimulationOption;
            _ptm.FollowHead = FollowHead;
            _ptm.TrackRotation = TrackRotation;
        }
    }
	
	// Update is called once per frame
	void Update () {
    }

    private bool checkConnection()
    {
        #if UNITY_EDITOR
        if (Application.isEditor)
        {
            return false;
        } else
        #endif
        {
            WaveVR.Device _device = WaveVR.Instance.getDeviceByType (this.deviceType);
            return _device.connected;
        }
    }

    private void ApplyIndicatorParameters()
    {
        if (!overwriteIndicatorSettings) return;
        WaveVR_ShowIndicator si = null;

        var ch = controllerPrefab.transform.childCount;
        bool found = false;

        for (int i = 0; i < ch; i++)
        {
            PrintInfoLog(controllerPrefab.transform.GetChild(i).gameObject.name);

            GameObject CM = controllerPrefab.transform.GetChild(i).gameObject;

            si = CM.GetComponentInChildren<WaveVR_ShowIndicator>();

            if (si != null)
            {
                found = true;
                break;
            }
        }


        if (found)
        {
            PrintInfoLog("WaveVR_ControllerLoader forced update WaveVR_ShowIndicator parameter!");
            si.showIndicator = this.showIndicator;

            if (showIndicator != true)
            {
                PrintInfoLog("WaveVR_ControllerLoader forced don't show WaveVR_ShowIndicator!");
                return;
            }
            si.showIndicator = this.showIndicator;
            si.showIndicatorAngle = showIndicatorAngle;
            si.hideIndicatorByRoll = hideIndicatorByRoll;
            si.lineColor = lineColor;
            si.lineEndWidth = lineEndWidth;
            si.lineStartWidth = lineStartWidth;
            si.lineLength = lineLength;
            si.textCharacterSize = textCharacterSize;
            si.zhCharactarSize = zhCharactarSize;
            si.textColor = textColor;
            si.textFontSize = textFontSize;

            if (buttonIndicationList.Count == 0)
            {
                PrintInfoLog("WaveVR_ControllerLoader uses controller model default button indication!");
                return;
            }
            PrintInfoLog("WaveVR_ControllerLoader uses customized button indication!");
            si.buttonIndicationList.Clear();
            foreach (ButtonIndication bi in buttonIndicationList)
            {
                PrintInfoLog("indication: "+ bi.indicationText);
                PrintInfoLog("alignment: " + bi.alignment);
                PrintInfoLog("offset: " + bi.indicationOffset);
                PrintInfoLog("keyType: " + bi.keyType);
                PrintInfoLog("followRotation: " + bi.followButtonRotation);

                si.buttonIndicationList.Add(bi);
            }
        } else
        {
            PrintInfoLog("Controller model doesn't support button indication feature!");
        }
    }
}
