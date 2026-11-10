using UnityEngine;
using UnityEngine.EventSystems;

public class DialKnob : MonoBehaviour,
    IPointerDownHandler,
    IPointerUpHandler,
    IDragHandler
{
    // public 
    public  MainHandler mainHandler;

    // Private 
    private string value         = "";
    private float  previousAngle = 0f;
    private float  accumulDelta  = 0f;
    private float  fineStep      = 3f; // Degrees or mm
    
    private RectTransform knob;

    void Start() {
        knob = GetComponent<RectTransform>();
    }


    // Private Methods 
    private float GetAngle(PointerEventData eventData) {
        
        Vector2 knobPos = RectTransformUtility.WorldToScreenPoint(
            eventData.pressEventCamera,
            knob.position
        );

        Vector2 dir = eventData.position - knobPos;

        float angle = Mathf.Atan2(dir.y, dir.x) * Mathf.Rad2Deg;

        angle -= 180f;

        return angle;
    }


    // Public Methods
    public void OnPointerDown(PointerEventData eventData) {
        previousAngle = GetAngle(eventData);
    }
    
    public void OnPointerUp(PointerEventData eventData) {
        mainHandler.StopNavRoutine();
    }

    public void OnDrag(PointerEventData eventData) {

        float currentAngle = GetAngle(eventData);
        float delta        = Mathf.DeltaAngle(previousAngle, currentAngle);

        accumulDelta -= delta;

        // One step every 5 degrees
        if(accumulDelta >= fineStep) {
            value = "+";
            accumulDelta = 0f;
        }

        if(accumulDelta <= -fineStep) {
            value = "-";
            accumulDelta = 0f;
        }

        knob.localEulerAngles = new Vector3(0, 0, currentAngle);
        previousAngle = currentAngle;
        
        SendToArduino(value);
        value = "";
    }

    public void SendToArduino(string value) {

        if(value != "+" && value != "-") return;

        string axis = mainHandler.lastAxisName;

        mainHandler.NavMove($"nav>{axis}{value}:{fineStep}", axis);
    }

}


