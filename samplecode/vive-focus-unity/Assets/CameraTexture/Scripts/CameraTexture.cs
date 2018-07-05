using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;
using UnityEngine;
using UnityEngine.UI;
using wvr;
using WaveVR_Log;
using System.Text;

public class CameraTexture : MonoBehaviour
{
    public HandController controller;
    private bool started = false;
    private bool updated = true;
    private static string LOG_TAG = "CameraTexture";
    private Texture2D nativeTexture = null;
    public Camera playerCamera;

    private RenderTexture rt;
    private Texture2D t;
    System.IntPtr textureid;
    private Camera canvasCamera;
    private RawImage image;
    private protob.Hands hands;
    UdpClient client = new UdpClient();

    private String lastRecvString;
    
    void Start()
    {
        image = GetComponent<RawImage>();
        canvasCamera = GameObject.Find("Camera").GetComponent<Camera>();
        
        try
        {
            client.InitSocket(Config.SERVER_IP, Config.SERVER_PORT);
        }
        catch (Exception e)
        {
            Log.e(LOG_TAG, e.ToString());
            Log.e(LOG_TAG, "UDP Start Error");
        }
        
#if UNITY_EDITOR
        if (Application.isPlaying)
            return;
#endif
        startCamera();
    }

    private void startCamera()
    {
        WaveVR_CameraTexture.UpdateCameraCompletedDelegate += updateTextureCompleted;
        WaveVR_CameraTexture.StartCameraCompletedDelegate += onStartCameraCompleted;
        started = WaveVR_CameraTexture.instance.startCamera();
        nativeTexture = new Texture2D(1280, 400);
        textureid = nativeTexture.GetNativeTexturePtr();

        // This rawimage is used to display the camera image
        image.texture = nativeTexture;
        rt = new RenderTexture(1280, 400, 24);
        t = new Texture2D(1280, 400);
    }

    void onStartCameraCompleted(bool result)
    {
        started = result;
    }

    void updateTextureCompleted(uint textureId)
    {
        image.texture = nativeTexture;
        updated = true;
    }

    void OnApplicationPause(bool pauseStatus)
    {
#if UNITY_EDITOR
        if (Application.isPlaying)
            return;
#endif
        if (!pauseStatus)
        {
            if (started)
            {
                startCamera();
            }
        }
    }
    
    // Convert real world point (protobuf format) to Unity point. shift/scale values were determined experimentally.
    private Vector3 toUnityPoint(protob.PointXYZ value, bool flipY = true,
                        float scaleX = 300.0f, float scaleY = 300.0f, float scaleZ = 725.0f,
                        float shiftX = -20.0f, float shiftY = -25.0f, float shiftZ = -330.0f)
    {
        Vector3 pt = new Vector3(value.x * scaleX + shiftX, value.y * scaleY + shiftY, value.z * scaleZ + shiftZ);
        if (flipY) pt.y = -pt.y;
        Log.e(LOG_TAG, pt.ToString());
        return playerCamera.transform.TransformPoint(pt);
    }

    // Update is called once per frame
    void Update()
    {

#if UNITY_EDITOR
        if (Application.isPlaying)
            return;
#endif
        if (started && updated)
        {
            WaveVR_CameraTexture.instance.updateTexture((uint)textureid);
            updated = false;
        }

        // Capture screenshots as Texture2D 
        Log.e(LOG_TAG, "OpenARK Vive Focus v1.0.0");
        Log.e(LOG_TAG , "Ready to capture:" + DateTime.Now.Millisecond);
        canvasCamera.targetTexture = rt;
        canvasCamera.Render();
        RenderTexture.active = rt;
        t.ReadPixels(new Rect(0, 0, 1280f, 400f), 0, 0);
        t.Apply();
        Log.e(LOG_TAG, "Finish to capture:" + DateTime.Now.Millisecond);

        // Serialize
        Log.e(LOG_TAG, "Ready to Serialize");
        protob.Image image = new protob.Image();
        image.ImageString = t.EncodeToJPG();
        MemoryStream ms = new MemoryStream();
        ProtoBuf.Serializer.Serialize<protob.Image>(ms, image);
        byte[] result = new byte[ms.Length];
        ms.Position = 0;
        ms.Read(result, 0, result.Length);
        ms.Close();
        Log.e(LOG_TAG, "Finish Serializing");

        Log.e(LOG_TAG, "Ready to send message by UDP");
        int length = client.SocketSend(result);
        Log.e(LOG_TAG, "Finish sending" + length);

        Log.e(LOG_TAG, "Listening for hand detections from server");
        String recvString = client.GetRecvStr();
        Log.e(LOG_TAG, "Received bytes: " + client.GetRecvLen().ToString());
        if (!String.IsNullOrEmpty(recvString)) {
            if (!String.IsNullOrEmpty(lastRecvString) && lastRecvString == recvString)
            {
                return;
            }
            byte[] recvBytes = Convert.FromBase64String(recvString);
            using (MemoryStream recvStream = new MemoryStream(recvBytes))
            {
                hands = ProtoBuf.Serializer.Deserialize<protob.Hands>(recvStream);
            }
            if (hands.hands.Count > 0)
            {
                Log.e(LOG_TAG, hands.hands.Count.ToString() + " hands detected");

                var cenPt = hands.hands[0].palmCenter.pointXYZ;
                var wristLPt = hands.hands[0].wrist[0].pointXYZ;
                var wristRPt = hands.hands[0].wrist[1].pointXYZ;
                Log.e(LOG_TAG, "center " + cenPt.x.ToString() + " " + cenPt.y.ToString() + " " + cenPt.z.ToString());
                controller.Center.addPoint(toUnityPoint(cenPt));
                controller.WristL.addPoint(toUnityPoint(wristLPt));
                controller.WristR.addPoint(toUnityPoint(wristRPt));
                controller.numFingers = hands.hands[0].fingers.Count;
                for (int i = 0; i < controller.numFingers; ++i)
                {
                    controller.Fingers[0].addPoint(toUnityPoint(hands.hands[0].fingers[i].pointXYZ));
                }
                Log.e(LOG_TAG, "Position updated (debug)");
            } else
            {
                Log.e(LOG_TAG, "No hands detected");
            }
        } else
        {
            Log.e(LOG_TAG, "No detections received");
        }
        lastRecvString = recvString;
    }
}
