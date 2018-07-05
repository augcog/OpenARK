using UnityEngine.EventSystems;

/// Handler for pointer is hovering over GameObject.
public interface IPointerHoverHandler : IEventSystemHandler
{
    void OnPointerHover (PointerEventData eventData);
}
