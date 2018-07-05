//#define DEBUG
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;
using WaveVR_Log;

[System.Serializable]
public class ButtonIndication
{
    public enum Alignment
    {
        RIGHT,
        LEFT
    };

    public enum KeyIndicator
    {
        TriggerKey,
        TouchPad,
        BumperKey,
        AppButton,
        HomeButton,
        VolumeKey
    };

    public KeyIndicator keyType;
    public Alignment alignment = Alignment.RIGHT;
    public Vector3 indicationOffset = new Vector3(0f, 0f, 0f);

    public string indicationText = "system";
    public bool followButtonRotation = false;
}

[System.Serializable]
public class ComponentsIndication
{
    public string Name;  // Component name
    public string Description = "system";  // Component description
    public GameObject SourceObject;
    public GameObject LineIndicator;
    public GameObject DestObject;
    public ButtonIndication.Alignment alignment = ButtonIndication.Alignment.RIGHT;
    public Vector3 Offset;
    public bool followButtonRoration = false;
 }

public class WaveVR_ShowIndicator : MonoBehaviour {
    private static string LOG_TAG = "WaveVR_ShowIndicator";
    private void PrintDebugLog(string msg)
    {
#if UNITY_EDITOR
        Debug.Log(LOG_TAG +  msg);
#endif
        Log.d(LOG_TAG, msg);
    }

    [Header("Indication feature")]
    public bool showIndicator = true;
    [Range(0, 90.0f)]
    public float showIndicatorAngle = 30.0f;
    public bool hideIndicatorByRoll = true;

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

#if !UNITY_EDITOR
    private WaveVR_Resource rw = null;
    private string sysLang = null;
    private string sysCountry = null;
    private int checkCount = 0;
#endif
    private GameObject indicatorPrefab = null;
    private GameObject linePrefab = null;
    private List<ComponentsIndication> compInd = new List<ComponentsIndication>();
    private GameObject _HMD = null;
    private bool needRedraw = true;

    // reset for redraw
    void resetIndicator()
    {
        if (showIndicator)
        {
            needRedraw = true;
            clearResourceAndObject();
        }
    }

    void OnApplicationPause(bool pauseStatus)
    {
        if (pauseStatus == true)
        {
            resetIndicator();
        }
    }

    void clearResourceAndObject()
    {
        PrintDebugLog("clear Indicator!");
        foreach (ComponentsIndication ci in compInd)
        {
            if (ci.DestObject != null)
            {
                Destroy(ci.DestObject);
            }
            if (ci.LineIndicator != null)
            {
                Destroy(ci.LineIndicator);
            }
        }
        compInd.Clear();

        Resources.UnloadUnusedAssets();
    }

    // Use this for initialization
    void Start() {
        createIndicator();
    }

    void createIndicator() {
        if (!showIndicator) return;
        clearResourceAndObject();
        PrintDebugLog("create Indicator!");
#if !UNITY_EDITOR
        rw = WaveVR_Resource.instance;
#endif
        indicatorPrefab = Resources.Load("ComponentIndicator") as GameObject;

        if (indicatorPrefab == null)
        {
            PrintDebugLog("ComponentIndicator is not found!");
            return;
        }
        else
        {
            PrintDebugLog("ComponentIndicator is found!");
        }

        linePrefab = Resources.Load("LineIndicator") as GameObject;

        if (linePrefab == null)
        {
            PrintDebugLog("LineIndicator is not found!");
            return;
        }
        else
        {
            PrintDebugLog("LineIndicator is found!");
        }

        if (_HMD == null)
            _HMD = WaveVR_Render.Instance.gameObject;

        if (_HMD == null)
        {
            PrintDebugLog("Can't get HMD!");
            return;
        }
        PrintDebugLog("showIndicatorAngle: " + showIndicatorAngle + ", hideIndicatorByRoll: " + hideIndicatorByRoll);
        PrintDebugLog("Line settings--\n lineLength: " + lineLength + ", lineStartWidth: " + lineStartWidth + ", lineEndWidth: " + lineEndWidth + ", lineColor: " + lineColor);
        PrintDebugLog("Text settings--\n textCharacterSize: " + textCharacterSize + ", zhCharactarSize: " + zhCharactarSize + ", textFontSize: " + textFontSize + ", textColor: " + textColor);

        foreach (ButtonIndication bi in buttonIndicationList)
        {
            PrintDebugLog("keyType: " + bi.keyType + ", alignment: " + bi.alignment + ", offset: " + bi.indicationOffset +
                ", indication: " + bi.indicationText + ", followRotation: " + bi.followButtonRotation);

            // find component by name
            string partName = null;
            string indicationKey = null;
            switch(bi.keyType)
            {
                case ButtonIndication.KeyIndicator.AppButton:
                    partName = "_[CM]_AppButton";
                    indicationKey = "AppKey";
                    break;
                case ButtonIndication.KeyIndicator.BumperKey:
                    partName = "_[CM]_BumperKey";
                    indicationKey = "BumperKey";
                    break;
                case ButtonIndication.KeyIndicator.HomeButton:
                    partName = "_[CM]_HomeButton";
                    indicationKey = "HomeKey";
                    break;
                case ButtonIndication.KeyIndicator.TouchPad:
                    partName = "_[CM]_TouchPad";
                    indicationKey = "TouchPad";
                    break;
                case ButtonIndication.KeyIndicator.TriggerKey:
                    partName = "_[CM]_TriggerKey";
                    indicationKey = "TriggerKey";
                    break;
                case ButtonIndication.KeyIndicator.VolumeKey:
                    partName = "_[CM]_VolumeKey";
                    indicationKey = "VolumeKey";
                    break;
                default:
                    partName = "_[CM]_unknown";
                    indicationKey = "unknown";
                    PrintDebugLog("Unknown key type!");
                    break;
            }

            Transform tmp = transform.Find(partName);

            if (tmp != null)
            {
                ComponentsIndication tmpCom = new ComponentsIndication();

                tmpCom.Name = partName;
                tmpCom.SourceObject = tmp.gameObject;
                tmpCom.alignment = bi.alignment;
                tmpCom.followButtonRoration = bi.followButtonRotation;

                Vector3 linePos;
                tmpCom.LineIndicator = null;

                linePos = transform.TransformPoint(new Vector3(0, tmp.localPosition.y, tmp.localPosition.z) + bi.indicationOffset);
                Quaternion spawnRot = Quaternion.identity;
                if (bi.followButtonRotation == true)
                {
                    spawnRot = transform.rotation;
                }

                GameObject lineGO = Instantiate(linePrefab, linePos, spawnRot);
                lineGO.name = partName + "Line";

                var li = lineGO.GetComponent<IndicatorLine>();
                li.lineColor = lineColor;
                li.lineLength = lineLength;
                li.startWidth = lineStartWidth;
                li.endWidth = lineEndWidth;
                li.alignment = bi.alignment;
                li.updateMeshSettings();

                if (bi.followButtonRotation == true)
                {
                    lineGO.transform.parent = tmpCom.SourceObject.transform;
                }
                lineGO.SetActive(false);
                tmpCom.LineIndicator = lineGO;

                tmpCom.DestObject = null;

                Vector3 spawnPos;
                if (bi.alignment == ButtonIndication.Alignment.RIGHT)
                {
                    spawnPos = transform.TransformPoint(new Vector3(lineLength, tmp.localPosition.y, tmp.localPosition.z) + bi.indicationOffset);
                } else
                {
                    spawnPos = transform.TransformPoint(new Vector3(lineLength * (-1), tmp.localPosition.y, tmp.localPosition.z) + bi.indicationOffset);
                }

                GameObject destGO = Instantiate(indicatorPrefab, spawnPos, transform.rotation);
                destGO.name = partName + "Ind";
                if (bi.followButtonRotation == true)
                {
                    destGO.transform.parent = tmpCom.SourceObject.transform;
                }

                PrintDebugLog(" Source PartName: " + tmp.gameObject.name + " pos: " + tmp.position + " Rot: " + tmp.rotation);
                PrintDebugLog(" Line Name: " + lineGO.name + " pos: " + lineGO.transform.position + " Rot: " + lineGO.transform.rotation);
                PrintDebugLog(" Destination Name: " + destGO.name + " pos: " + destGO.transform.position + " Rot: " + destGO.transform.rotation);

                int childC = destGO.transform.childCount;
                for (int i = 0; i < childC; i++)
                {
                    GameObject c = destGO.transform.GetChild(i).gameObject;
                    if (bi.alignment == ButtonIndication.Alignment.LEFT)
                    {
                        float tx = c.transform.localPosition.x;
                        c.transform.localPosition = new Vector3(tx * (-1), c.transform.localPosition.y, c.transform.localPosition.z);
                    }
                    TextMesh tm = c.GetComponent<TextMesh>();
                    MeshRenderer mr = c.GetComponent<MeshRenderer>();

                    if (tm == null) PrintDebugLog(" tm is null ");
                    if (mr == null) PrintDebugLog(" mr is null ");

                    if (tm != null && mr != null)
                    {
                        tm.characterSize = textCharacterSize;
                        if (c.name != "Shadow")
                        {
                            mr.material.SetColor("_Color", textColor);
                        } else
                        {
                            PrintDebugLog(" Shadow found ");
                        }
                        tm.fontSize = textFontSize;
#if !UNITY_EDITOR
                        if (bi.indicationText == "system")
                        {
                            sysLang = rw.getSystemLanguage();
                            sysCountry = rw.getSystemCountry();
                            PrintDebugLog(" System language is " + sysLang);
                            if (sysLang.StartsWith("zh"))
                            {
                                PrintDebugLog(" Chinese language");
                                tm.characterSize = zhCharactarSize;
                            }

                            // use default string - multi-language
                            tm.text = rw.getString(indicationKey);
                            PrintDebugLog(" Name: " + destGO.name + " uses default multi-language -> " + tm.text);
                        } else {
                            tm.text = bi.indicationText;
                        }
#else
                        tm.text = indicationKey;
#endif

                        if (bi.alignment == ButtonIndication.Alignment.LEFT)
                        {
                            tm.anchor = TextAnchor.MiddleRight;
                            tm.alignment = TextAlignment.Right;
                        }
                    }
                }

                destGO.SetActive(false);
                tmpCom.DestObject = destGO;
                tmpCom.Offset = bi.indicationOffset;

                PrintDebugLog(tmpCom.Name + " line -> " + tmpCom.LineIndicator.name + " destObjName -> " + tmpCom.DestObject.name);
                compInd.Add(tmpCom);
            }
            else
            {
                PrintDebugLog(partName + " is not in the model!");
            }
        }

        needRedraw = false;
    }

    // Update is called once per frame
    void Update () {
        if (!showIndicator) return;
        if (_HMD == null) return;
#if !UNITY_EDITOR
        checkCount++;
        if (checkCount > 50) {
            checkCount = 0;
            if (rw.getSystemLanguage() != sysLang || rw.getSystemCountry() != sysCountry) resetIndicator();
        }
#endif
        if (needRedraw == true) createIndicator();

        Vector3 _targetForward = transform.rotation * Vector3.forward;
        Vector3 _targetRight = transform.rotation * Vector3.right;
        Vector3 _targetUp = transform.rotation * Vector3.up;

        float zAngle = Vector3.Angle(_targetForward, _HMD.transform.forward);
        float xAngle = Vector3.Angle(_targetRight, _HMD.transform.right);
        float yAngle = Vector3.Angle(_targetUp, _HMD.transform.up);

#if DEBUG
        Log.gpl.d(LOG_TAG, "Z: " + _targetForward + ":" + zAngle + ", X: " + _targetRight + ":" + xAngle + ", Y: " + _targetUp + ":" + yAngle);
#endif
        if ((_targetForward.y < (showIndicatorAngle / 90f)) || (zAngle < showIndicatorAngle))
        {
            foreach (ComponentsIndication ci in compInd)
            {
                ci.LineIndicator.SetActive(false);
                ci.DestObject.SetActive(false);
            }

            return;
        }

        if (hideIndicatorByRoll)
        {
            if (xAngle > 90.0f)
            //if ((_targetRight.x < 0f) || (xAngle > 90f))
            {
                foreach (ComponentsIndication ci in compInd)
                {
                    ci.LineIndicator.SetActive (false);
                    ci.DestObject.SetActive (false);
                }

                return;
            }
        }

        foreach (ComponentsIndication ci in compInd)
        {
            if (ci.LineIndicator != null)
            {
                ci.LineIndicator.SetActive(true);
            }

            if (ci.DestObject != null)
            {
                ci.DestObject.SetActive(true);
                if (ci.followButtonRoration == false)
                {
                    ci.LineIndicator.transform.position = ci.SourceObject.transform.position + ci.Offset;
                    if (ci.alignment == ButtonIndication.Alignment.RIGHT)
                    {
                        ci.DestObject.transform.position = new Vector3(transform.position.x + lineLength, ci.SourceObject.transform.position.y, ci.SourceObject.transform.position.z) + ci.Offset;
                    } else
                    {
                        ci.DestObject.transform.position = new Vector3(transform.position.x - lineLength, ci.SourceObject.transform.position.y, ci.SourceObject.transform.position.z) + ci.Offset;

                        TextMesh[] texts = ci.DestObject.GetComponentsInChildren<TextMesh>();
                        foreach (TextMesh tm in texts)
                        {
                            if (tm != null)
                            {
                                tm.anchor = TextAnchor.MiddleRight;
                                tm.alignment = TextAlignment.Right;
                            }
                        }
                    }

                    Transform[] transforms = ci.DestObject.GetComponentsInChildren<Transform>();
                    foreach (Transform tf in transforms)
                    {
                        if (tf != null)
                        {
                            tf.rotation = Quaternion.identity;
                        }
                    }
                }
            }
        }
	}
}
