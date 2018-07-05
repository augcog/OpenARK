// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

#pragma warning disable 0219
#pragma warning disable 0414

using WaveVR_Log;

public class WaveVR_IMEManagerWrapper {
	private static string LOG_TAG = "IMEManagerWrapper";
	private static int VERSION_ID = 3;

	private static WaveVR_IMEManager mIMEManager = null;
	private static WaveVR_IMEManagerWrapper mInstance = null;
	private WaveVR_IMEManager.IMEParameter mParameter = null;
	private static InputDoneCallback mCallback = null;

	private static int CONTROLLER_BUTTON_MIN = 0;
	private static int CONTROLLER_BUTTON_MENU = 1;
	private static int CONTROLLER_BUTTON_GRIP = 2;
	private static int CONTROLLER_BUTTON_VOLUME_UP = 4;
	private static int CONTROLLER_BUTTON_VOLUME_DOWN = 8;
	private static int CONTROLLER_BUTTON_TOUCH_PAD = 16;
	private static int CONTROLLER_BUTTON_TRIGGER = 32;
	private static int CONTROLLER_BUTTON_BUMPER = 64;
	private static int CONTROLLER_BUTTON_DEFAULT = 112;
	private static int CONTROLLER_BUTTON_MAX = 127;

	public enum Locale {
		en_US = 0,
		zh_CN = 1,
    };

	public enum Action {
		Done = 0,
		Enter = 1,
		Search = 2,
		Go = 3,
		Send = 4,
		Next = 5,
		Submit = 6,
	};

	private WaveVR_IMEManagerWrapper() {
		InitParameter();
	}

	public static WaveVR_IMEManagerWrapper GetInstance() {
		if (mInstance == null || mIMEManager == null) {
			mInstance = new WaveVR_IMEManagerWrapper();
			mIMEManager = WaveVR_IMEManager.instance;
			mIMEManager.isInitialized();
		}

		Log.d(LOG_TAG, "VERSION_ID=" + VERSION_ID);

		return mInstance;
	}

	public void SetText(string text) {
		mParameter.exist = text;
	}

	public void SetTitle(string title) {
		mParameter.title = title;
	}
		
	public void SetLocale(Locale locale) {
        Log.d(LOG_TAG, "SetLocale, locale = " + locale);
        if (locale == Locale.en_US)
        {
            mParameter.locale = "en_US";
        }
        else if (locale == Locale.zh_CN)
        {
            mParameter.locale = "zh_CN";
        }
        else
        {
            mParameter.locale = "";
        }
    }

	public void SetCallback(InputDoneCallback callback) {
		mCallback = callback;
	}

	public void Show() {
		mIMEManager.showKeyboard(mParameter, inputDoneCallback);
    }

	public void Hide() {
		mIMEManager.hideKeyboard();
	}

	public void SetAction(Action action) {
		mParameter.extraString = "action=" + (int)action;
	}

	public void InitParameter() {
		int MODE_FLAG_FIX_MOTION = 0x02;
		int MODE_FLAG_AUTO_FIT_CAMERA = 0x04;
		int id = 0;
		int type = MODE_FLAG_FIX_MOTION;
		int mode = 2;

		string exist = "";
		int cursor = 0;
		int selectStart = 0;
		int selectEnd = 0;
		double[] pos = new double[] { 0, 0, -1 };
		double[] rot = new double[] { 1.0, 0.0, 0.0,0.0 }; 
		int width = 800;
		int height = 800;
		int shadow = 100;
		string locale = "";
		string title = "";
		int extraInt = 0;
		string extraString = "";
		int buttonId = CONTROLLER_BUTTON_DEFAULT;
		mParameter = new WaveVR_IMEManager.IMEParameter(id, type, mode, exist, cursor, selectStart, selectEnd, pos,
			rot, width, height, shadow, locale, title, extraInt, extraString, buttonId);
	}

	public delegate void InputDoneCallback(InputResult results);

	public class InputResult {
		private string mContent;

		public InputResult(string content) { 
			mContent = content;
		}

		public string GetContent() {
			return mContent;
		}
	}

	private void inputDoneCallback(WaveVR_IMEManager.InputResult results) {
		if (mCallback != null) {
			InputResult inputResult = new InputResult(results.InputContent);
			mCallback(inputResult);
		}
	}
}
