// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using System.Runtime.InteropServices;
using UnityEngine;

namespace WaveVR_Log
{
    class Log
    {
        private const int ANDROID_LOG_VERBOSE = 2;
        private const int ANDROID_LOG_DEBUG = 3;
        private const int ANDROID_LOG_INFO = 4;
        private const int ANDROID_LOG_WARN = 5;
        private const int ANDROID_LOG_ERROR = 6;

#if UNITY_ANDROID && !UNITY_EDITOR
        [DllImportAttribute("log", EntryPoint = "__android_log_print", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern int __android_log_print(int prio, string tag, string fmt, System.IntPtr ptr);
#else
        private static int __android_log_print(int prio, string tag, string fmt, System.IntPtr ptr)
        {
            //Debug.Log(fmt);
            return 0;
        }
#endif

        public static void d(string tag, string message)
        {
            __android_log_print(ANDROID_LOG_DEBUG, tag, message, System.IntPtr.Zero);
        }
        public static void i(string tag, string message)
        {
            __android_log_print(ANDROID_LOG_INFO, tag, message, System.IntPtr.Zero);
        }
        public static void w(string tag, string message)
        {
            __android_log_print(ANDROID_LOG_WARN, tag, message, System.IntPtr.Zero);
        }
        public static void e(string tag, string message)
        {
            __android_log_print(ANDROID_LOG_ERROR, tag, message, System.IntPtr.Zero);
        }

        public class PeriodLog
        {
            public float interval = 3;   // default is 3 seconds
            private float lastTime = 0;
            private bool print = true;

            public PeriodLog()
            {
                lastTime = Time.time;
            }

            public void check()
            {
                var time = Time.time;
                print = false;
                if (time > (lastTime + interval))
                {
                    lastTime = time;
                    print = true;
                }
            }

            public void d(string tag, string message)
            {
                if (print) Log.d(tag, message);
            }
        }

        public static PeriodLog gpl = new PeriodLog();
    }
}