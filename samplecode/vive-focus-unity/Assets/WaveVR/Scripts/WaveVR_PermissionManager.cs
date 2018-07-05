// "WaveVR SDK"  
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

public class WaveVR_PermissionManager {
    private static string LOG_TAG = "WVR_PermissionManager";
    private const string PERMISSION_MANAGER_CLASSNAME = "com.htc.vr.unity.PermissionManager";
    private AndroidJavaObject permissionsManager = null;

    private static WaveVR_PermissionManager mInstance = null;

    public static WaveVR_PermissionManager instance {
        get
        {
            if (mInstance == null)
            {
                mInstance = new WaveVR_PermissionManager();
            }

            return mInstance;
        }
    }

    private AndroidJavaObject javaArrayFromCS(string[] values)
    {
        AndroidJavaClass arrayClass = new AndroidJavaClass("java.lang.reflect.Array");
        AndroidJavaObject arrayObject = arrayClass.CallStatic<AndroidJavaObject>("newInstance", new AndroidJavaClass("java.lang.String"), values.Length);
        for (int i = 0; i < values.Length; ++i)
        {
            arrayClass.CallStatic("set", arrayObject, i, new AndroidJavaObject("java.lang.String", values[i]));
        }

        return arrayObject;
    }

    private void initializeJavaObject()
    {
        Log.d(LOG_TAG, "initializeJavaObject");
        AndroidJavaClass ajc = new AndroidJavaClass(PERMISSION_MANAGER_CLASSNAME);

        if (ajc == null)
        {
            Log.e(LOG_TAG, "AndroidJavaClass is null");
            return;
        }
        // Get the PermissionManager object
        permissionsManager = ajc.CallStatic<AndroidJavaObject>("getInstance");
        if (permissionsManager != null)
        {
            Log.d(LOG_TAG, "permissionsManager get object success");
        } else
        {
            Log.e(LOG_TAG, "permissionsManager get object failed");
        }
    }

    public bool isInitialized()
    {
        if (permissionsManager == null)
        {
            initializeJavaObject();
        }

        if (permissionsManager == null)
        {
            Log.e(LOG_TAG, "isInitialized failed because fail to get permissionsManager object");
            return false;
        }

        return permissionsManager.Call<bool>("isInitialized");
    }

    public void requestPermissions(string[] permissions, requestCompleteCallback cb)
    {
        Log.d(LOG_TAG, "requestPermission");

        if (!isInitialized())
        {
            Log.e(LOG_TAG, "requestPermissions failed because permissionsManager doesn't initialize");
            return;
        }

        if(!permissionsManager.Call<bool>("isShow2D"))
        {
           mCallback = cb;
           permissionsManager.Call("requestPermissions", javaArrayFromCS(permissions), new RequestCompleteHandler());
        }
        else
        {
            mCallback = cb;
            using (AndroidJavaClass jc = new AndroidJavaClass ("com.unity3d.player.UnityPlayer"))
    		{
			    using (AndroidJavaObject jo=jc.GetStatic<AndroidJavaObject>("currentActivity"))
                {
                    jo.Call("setRequestPermissionCallback", new RequestCompleteHandler());
			    }
		    }
            permissionsManager.Call("requestPermissions", javaArrayFromCS(permissions), new RequestCompleteHandler());
        }     
    }

    public bool isPermissionGranted(string permission)
    {
        if (!isInitialized())
        {
            Log.e(LOG_TAG, "isPermissionGranted failed because permissionsManager doesn't initialize");
            return false;
        }

        return permissionsManager.Call<bool>("isPermissionGranted", permission);
    }

    public bool shouldGrantPermission(string permission)
    {
        if (!isInitialized())
        {
            Log.e(LOG_TAG, "shouldGrantPermission failed because permissionsManager doesn't initialize");
            return false;
        }

        return permissionsManager.Call<bool>("shouldGrantPermission", permission);
    }

    public bool showDialogOnScene()
    {
        if (!isInitialized())
        {
            Log.e(LOG_TAG, "showDialogOnScene failed because permissionsManager doesn't initialize");
            return false;
        }

        return permissionsManager.Call<bool>("showDialogOnVRScene");
    }

    public class RequestResult
    {
        private string mPermission;
        private bool mGranted;

        public RequestResult(string name, bool granted)
        {
            mPermission = name;
            mGranted = granted;
        }
        public string PermissionName
        {
            get { return mPermission; }
        }

        public bool Granted
        {
            get { return mGranted; }
        }
    }

    public delegate void requestCompleteCallback(List<RequestResult> results);

    private static requestCompleteCallback mCallback = null;

    class RequestCompleteHandler : AndroidJavaProxy
    {
        internal RequestCompleteHandler() : base(new AndroidJavaClass("com.htc.vr.unity.PermissionCallback")) {
        }

        public void onRequestCompletedwithObject(AndroidJavaObject resultObject)
        {
            Log.i(LOG_TAG, "unity callback with result object");
            if (mCallback == null)
            {
                Log.w(LOG_TAG, "unity callback but user callback is null ");
            }

            List<RequestResult> permissionResults = new List<RequestResult>();
            bool[] resultArray = null;

            AndroidJavaObject boolbuffer = resultObject.Get<AndroidJavaObject>("result");
            if ((boolbuffer) != null && (boolbuffer.GetRawObject() != IntPtr.Zero))
            {
                resultArray = AndroidJNIHelper.ConvertFromJNIArray<bool[]>(boolbuffer.GetRawObject());
                Log.i(LOG_TAG, "ConvertFromJNIArray to bool array : " + resultArray.Length);
            }

            string[] permissionArray = null;

            AndroidJavaObject stringbuffer = resultObject.Get<AndroidJavaObject>("requestPermissions");
            if ((stringbuffer) != null && (stringbuffer.GetRawObject() != IntPtr.Zero))
            {
                permissionArray = AndroidJNIHelper.ConvertFromJNIArray<string[]>(stringbuffer.GetRawObject());
                Log.i(LOG_TAG, "ConvertFromJNIArray to string array : " + permissionArray.Length);
            }

            if (permissionArray != null && resultArray != null)
            {
                for (int i = 0; i < permissionArray.Length; i++)
                {
                    permissionResults.Add(new RequestResult(permissionArray[i], resultArray[i]));
                }
            }

            mCallback(permissionResults);
        }
    }
}
