using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public static class WaveVR_ExecuteEvents
{
    #region Event Executor of Hover
    /// Use ExecuteEvents.Execute (GameObject, BaseEventData, WaveVR_ExecuteEvents.pointerHoverHandler)
    private static void HoverExecutor(IPointerHoverHandler handler, BaseEventData eventData)
    {
        handler.OnPointerHover (ExecuteEvents.ValidateEventData<PointerEventData> (eventData));
    }

    public static ExecuteEvents.EventFunction<IPointerHoverHandler> pointerHoverHandler
    {
        get{ return HoverExecutor; }
    }
    #endregion
}
