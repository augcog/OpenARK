// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using WaveVR_Log;

public class WaveVR_EventHandler: MonoBehaviour,
    IPointerEnterHandler,
    IPointerExitHandler,
    IPointerDownHandler,
    IBeginDragHandler,
    IDragHandler,
    IEndDragHandler,
    IDropHandler,
    IPointerHoverHandler
{
    private const string LOG_TAG = "WaveVR_EventHandler";
    private WaitForEndOfFrame waitForEndOfFrame = new WaitForEndOfFrame();
    private Vector3 goPosition;
    private float goPositionZ;

    private void TeleportRandomly ()
    {
        Vector3 direction = UnityEngine.Random.onUnitSphere;
        direction.y = Mathf.Clamp (direction.y, 0.5f, 1f);
        direction.z = Mathf.Clamp (direction.z, 3f, 10f);
        float distance = 2 * UnityEngine.Random.value + 1.5f;
        transform.localPosition = direction * distance;
    }

    private void Rotate()
    {
        transform.Rotate (72 * (10 * Time.deltaTime), 0, 0);
        transform.Rotate (0, 72 * (10 * Time.deltaTime), 0);
    }

    #region override event handling function
    public void OnPointerEnter (PointerEventData eventData)
    {
        #if UNITY_EDITOR
        Debug.Log("OnPointerEnter, camera: " + eventData.enterEventCamera);
        #endif
        Log.d (LOG_TAG, "OnPointerEnter, camera: " + eventData.enterEventCamera);
    }

    public void OnPointerExit (PointerEventData eventData)
    {
        // Do nothing
    }

    public void OnPointerDown (PointerEventData eventData)
    {
        #if UNITY_EDITOR
        Debug.Log("WaveVR_EventHandler::OnPointerDown");
        #endif
        Rotate ();
    }
    // Called when the pointer enters our GUI component.
    // Start tracking the mouse
    public void OnBeginDrag(PointerEventData eventData)
    {
        goPosition = transform.position;
        goPositionZ = transform.position.z;

        #if UNITY_EDITOR
        Debug.Log("WaveVR_EventHandler::OnBeginDrag, position: " + goPosition);
        #endif
        Log.d (LOG_TAG, "OnBeginDrag() position: " + goPosition);

        StartCoroutine( "TrackPointer" );
    }

    public void OnDrag(PointerEventData eventData)
    {
        Camera _cam = eventData.enterEventCamera;
        goPosition = _cam.ScreenToWorldPoint (new Vector3 (eventData.position.x, eventData.position.y, goPositionZ));
        //Log.d (LOG_TAG, "OnDrag() camera: " + c + ", position: " + goPosition);
    }

    // Called when the pointer exits our GUI component.
    // Stop tracking the mouse
    public void OnEndDrag(PointerEventData eventData)
    {
        #if UNITY_EDITOR
        Debug.Log("WaveVR_EventHandler::OnEndDrag, position: " + goPosition);
        #endif
        Log.d (LOG_TAG, "OnEndDrag() position: " + goPosition);

        StopCoroutine( "TrackPointer" );
    }

    public void OnDrop(PointerEventData eventData)
    {
        Camera c = eventData.enterEventCamera;
        goPosition = c.ScreenToWorldPoint (new Vector3 (eventData.position.x, eventData.position.y, goPositionZ));

        #if UNITY_EDITOR
        Debug.Log("WaveVR_EventHandler::OnDrop, position: " + goPosition);
        #endif
    }

    public void OnPointerHover (PointerEventData eventData)
    {
        transform.Rotate (0, 12 * (10 * Time.deltaTime), 0);
    }
    #endregion

    IEnumerator TrackPointer()
    {
        while (true)
        {
            yield return waitForEndOfFrame;

            transform.position = goPosition;
        }
    }
}