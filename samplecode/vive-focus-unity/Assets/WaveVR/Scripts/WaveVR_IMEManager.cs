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

public class WaveVR_IMEManager
{
    private static string LOG_TAG = "WVR_IMEManager";
    private const string IME_MANAGER_CLASSNAME = "com.htc.vr.unity.IMEManager";
    private AndroidJavaObject imeManager = null;

    private static WaveVR_IMEManager mInstance = null;
    private bool inited = false;
    public static WaveVR_IMEManager instance
    {
        get
        {
            if (mInstance == null)
            {
                mInstance = new WaveVR_IMEManager();
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
    private AndroidJavaObject javaIMEParameterFromCS(IMEParameter parameter)
    {
        //AndroidJavaClass imeParameterClass = new AndroidJavaClass("com.htc.vr.ime.server.IMEParameter");
        AndroidJavaObject imeParameterObj = new AndroidJavaObject("com.htc.vr.ime.server.IMEParameter",
            parameter.id, parameter.type, parameter.mode, parameter.exist, parameter.cursor, parameter.selectStart,
            parameter.selectEnd, parameter.pos, parameter.rot, parameter.width, parameter.height, parameter.shadow,
            parameter.locale, parameter.title, parameter.extraInt, parameter.extraString, parameter.buttonId);

        return imeParameterObj;
    }

    private void initializeJavaObject()
    {
        Log.d(LOG_TAG, "initializeJavaObject");
        AndroidJavaClass ajc = new AndroidJavaClass(IME_MANAGER_CLASSNAME);

        if (ajc == null)
        {
            Log.e(LOG_TAG, "AndroidJavaClass is null");
            return;
        }
        // Get the IMEManager object
        imeManager = ajc.CallStatic<AndroidJavaObject>("getInstance");
        if (imeManager != null)
        {
            Log.d(LOG_TAG, "imeManager get object success");
        }
        else
        {
            Log.e(LOG_TAG, "imeManager get object failed");
        }
    }

    public bool isInitialized()
    {
        if (imeManager == null)
        {
            initializeJavaObject();
        }

        if (imeManager == null)
        {
            Log.e(LOG_TAG, "isInitialized failed because fail to get imeManager object");
            return false;
        }

        inited = imeManager.Call<bool>("isInitialized");
        return inited;
    }

    public void showKeyboard(IMEParameter parameter, inputCompleteCallback cb)
    {
        Log.d(LOG_TAG, "showKeyboard");

        if (imeManager == null)
        {
            initializeJavaObject();
        }

        if (imeManager == null)
        {
            Log.e(LOG_TAG, "isInitialized failed because fail to get imeManager object");
            return;
        }

        mCallback = cb;

        imeManager.Call("showKeyboard", javaIMEParameterFromCS(parameter), new RequestCompleteHandler());
    }
    public void hideKeyboard()
    {
        Log.d(LOG_TAG, "hideKeyboard");

        if (imeManager == null)
        {
            Log.e(LOG_TAG, "hideKeyboard() failed because fail to get imeManager object");
            return;
        }

        imeManager.Call("hideKeyboard");
    }

    public int getKeyboardState()
    {
        if (imeManager == null)
        {
            initializeJavaObject();
        }

        if (imeManager == null)
        {
            Log.e(LOG_TAG, "isInitialized failed because fail to get imeManager object");
            return -1;
        }
        int ret = imeManager.Call<int>("getKeyboardState");
        return ret;
    }
    public class InputResult
    {
        private int mId;
        private string mContent;
        private int mErrorCode;

        public InputResult(int id, string content, int errorCode)
        {
            mId = id;
            mContent = content;
            mErrorCode = errorCode;
        }
        public string InputContent
        {
            get { return mContent; }
        }

        public int ErrorCode
        {
            get { return mErrorCode; }
        }

        public int Id
        {
            get { return mId; }
        }
    }
    public class IMEParameter
    {
        public int id;
        public int type;
        public int mode;
        public string exist;
        public int cursor;
        public int selectStart;
        public int selectEnd;
        public double[] pos;
        public double[] rot;
        public int width;
        public int height;
        public int shadow;
        public string locale;
        public string title;
        public int buttonId;
        public int extraInt;
        public string extraString;

        public IMEParameter(int id, int type, int mode, string exist, int cursor, int selectStart, int selectEnd, double[] pos,
                            double[] rot, int width, int height, int shadow, string locale, string title, int extraInt, string extraString, int buttonId)
        {
            this.id = id;
            this.type = type;
            this.mode = mode;
            this.exist = exist;
            this.cursor = cursor;
            this.selectStart = selectStart;
            this.selectEnd = selectEnd;
            this.pos = pos;
            this.rot = rot;
            this.width = width;
            this.height = height;
            this.shadow = shadow;
            this.locale = locale;
            this.title = title;
            this.buttonId = buttonId;
            this.extraInt = extraInt;
            this.extraString = extraString;
        }

        public IMEParameter(int id, int type, int mode, string exist, int cursor, int selectStart, int selectEnd, double[] pos,
                            double[] rot, int width, int height, int shadow, string locale, string title, int buttonId)
        {
            this.id = id;
            this.type = type;
            this.mode = mode;
            this.exist = exist;
            this.cursor = cursor;
            this.selectStart = selectStart;
            this.selectEnd = selectEnd;
            this.pos = pos;
            this.rot = rot;
            this.width = width;
            this.height = height;
            this.shadow = shadow;
            this.locale = locale;
            this.title = title;
            this.buttonId = buttonId;
            this.extraInt = 0;
            this.extraString = null;
        }

        public IMEParameter(int id, int type, int mode, string exist, int cursor, double[] pos,
                            double[] rot, int width, int height, int shadow, string locale, string title, int buttonId)
        {
            this.id = id;
            this.type = type;
            this.mode = mode;
            this.exist = exist;
            this.cursor = cursor;
            this.selectStart = 0;
            this.selectEnd = 0;
            this.pos = pos;
            this.rot = rot;
            this.width = width;
            this.height = height;
            this.shadow = shadow;
            this.locale = locale;
            this.title = title;
            this.buttonId = buttonId;
            this.extraInt = 0;
            this.extraString = null;
        }

        public IMEParameter(int id, int type, int mode, double[] pos, double[] rot, int width, int height,
                            int shadow, string locale, string title, int buttonId)
        {
            this.id = id;
            this.type = type;
            this.mode = mode;
            this.exist = null;
            this.cursor = 0;
            this.selectStart = 0;
            this.selectEnd = 0;
            this.pos = pos;
            this.rot = rot;
            this.width = width;
            this.height = height;
            this.shadow = shadow;
            this.locale = locale;
            this.title = title;
            this.buttonId = buttonId;
            this.extraInt = 0;
            this.extraString = null;
        }

        public IMEParameter(int id, int type, int mode, double[] pos, double[] rot, int width, int height, int shadow, int buttonId)
        {
            this.id = id;
            this.type = type;
            this.mode = mode;
            this.exist = null;
            this.cursor = 0;
            this.selectStart = 0;
            this.selectEnd = 0;
            this.pos = pos;
            this.rot = rot;
            this.width = width;
            this.height = height;
            this.shadow = shadow;
            this.locale = null;
            this.title = null;
            this.buttonId = buttonId;
            this.extraInt = 0;
            this.extraString = null;
        }
    }



    public delegate void inputCompleteCallback(InputResult results);

    private static inputCompleteCallback mCallback = null;

    class RequestCompleteHandler : AndroidJavaProxy
    {
        internal RequestCompleteHandler() : base(new AndroidJavaClass("com.htc.vr.unity.IMECallback"))
        {
        }

        public void onInputCompletedwithObject(AndroidJavaObject resultObject)
        {
            Log.i(LOG_TAG, "unity callback with result object");
            if (mCallback == null)
            {
                Log.w(LOG_TAG, "unity callback but user callback is null ");
            }

            int id = resultObject.Get<int>("id");
            int errorCode = resultObject.Get<int>("errorCode");
            string inputContent = resultObject.Get<string>("inputContent");

            InputResult inputResult = new InputResult(id, inputContent, errorCode);
            mCallback(inputResult);
        }
    }
}
