// "WaveVR SDK
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using wvr;
using WaveVR_Log;
using System;

public class WaveVR_Resource {
    private static string LOG_TAG = "WVR_Resource";
    private const string RESOURCE_WRAPPER_CLASSNAME = "com.htc.vr.unity.ResourceWrapper";
    private AndroidJavaObject ResourceWrapper = null;

    private static WaveVR_Resource mInstance = null;

    public static WaveVR_Resource instance {
        get
        {
            if (mInstance == null)
            {
                mInstance = new WaveVR_Resource();
            }

            return mInstance;
        }
    }

    private void initializeJavaObject()
    {
        Log.d(LOG_TAG, "initializeJavaObject");
        AndroidJavaClass ajc = new AndroidJavaClass(RESOURCE_WRAPPER_CLASSNAME);

        if (ajc == null)
        {
            Log.e(LOG_TAG, "AndroidJavaClass is null");
            return;
        }
        // Get the PermissionManager object
        ResourceWrapper = ajc.CallStatic<AndroidJavaObject>("getInstance");
        if (ResourceWrapper != null)
        {
            Log.d(LOG_TAG, "WaveVR_Resource get object success");
        } else
        {
            Log.e(LOG_TAG, "WaveVR_Resource get object failed");
        }
    }

    public string getString(string stringName)
    {
        Log.d(LOG_TAG, "getString, string " + stringName);
        if (ResourceWrapper == null)
        {
            initializeJavaObject();
        }

        if (ResourceWrapper == null)
        {
            Log.e(LOG_TAG, "getString failed because fail to get WaveVR_Resource object");
            return "";
        }

        string retString = "";

        if (useSystemLanguageFlag == true)
        {
            retString = ResourceWrapper.Call<string>("getStringByName", stringName);
        } else
        {
            retString = ResourceWrapper.Call<string>("getPreferredStringByName", stringName, mPreferredLanguage, mCountry);
        }

        return retString;
    }

    public string getStringByLanguage(string stringName, string lang, string country)
    {
        Log.d(LOG_TAG, "getPreferredString, string " + stringName + " language is " + lang + " country is " + country);
        if (ResourceWrapper == null)
        {
            initializeJavaObject();
        }

        if (ResourceWrapper == null)
        {
            Log.e(LOG_TAG, "getString failed because fail to get WaveVR_Resource object");
            return "";
        }

        return ResourceWrapper.Call<string>("getPreferredStringByName", stringName, lang, country);
    }
    public string getSystemLanguage()
    {
        if (ResourceWrapper == null)
        {
            initializeJavaObject();
        }

        if (ResourceWrapper == null)
        {
            Log.e(LOG_TAG, "getSystenLanguage failed because fail to get WaveVR_Resource object");
            return "";
        }

        return ResourceWrapper.Call<string>("getSystemLanguage");
    }

    public string getSystemCountry()
    {
        if (ResourceWrapper == null)
        {
            initializeJavaObject();
        }

        if (ResourceWrapper == null)
        {
            Log.e(LOG_TAG, "getSystenCountry failed because fail to get WaveVR_Resource object");
            return "";
        }

        return ResourceWrapper.Call<string>("getSystemCountry");
    }

    public bool setPreferredLanguage(string lang, string country)
    {
        if (lang == "" || country == "")
            return false;

        useSystemLanguageFlag = false;
        mPreferredLanguage = lang;
        mCountry = country;
        return true;
    }

    public void useSystemLanguage()
    {
        mPreferredLanguage = "system";
        mCountry = "system";
        useSystemLanguageFlag = true;
    }
    private string mPreferredLanguage = "system";
    private string mCountry = "system";
    private bool useSystemLanguageFlag = true;
}
