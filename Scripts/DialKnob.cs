using UnityEngine;
using UnityEngine.EventSystems;

public class DialKnob : MonoBehaviour,
    IPointerDownHandler,
    IDragHandler
{
    // public 
    public  MainHandler mainHandler;
    public  string      axisName; // Axis Cap Letter

    // Private 
    private int   value         = 0;
    private float previousAngle = 0f;
    private float accumulDelta  = 0f;
    private float step          = 5f; // Degrees
    
    private RectTransform knob;

    void Start() {
        knob = GetComponent<RectTransform>();
    }

    public void OnPointerDown(PointerEventData eventData) {
        previousAngle = GetAngle(eventData);
    }

    public void OnDrag(PointerEventData eventData) {

        float currentAngle = GetAngle(eventData);
        float delta        = Mathf.DeltaAngle(previousAngle, currentAngle);

        accumulDelta -= delta;

        // One step every 5 degrees
        if(accumulDelta >= step) {
            value++;
            accumulDelta = 0f;
        }

        if(accumulDelta <= -step) {
            value--;
            accumulDelta = 0f;
        }

        knob.localEulerAngles = new Vector3(0, 0, currentAngle);
        previousAngle = currentAngle;
        
        SendToArduino(value);
    }

    public void SendToArduino(int value) {

        string coord = (axisName + value.ToString());

        if(axisName == "X") mainHandler.prev_X = value;
        if(axisName == "Y") mainHandler.prev_Y = value;

        mainHandler.serial.WriteLine(coord);
        mainHandler.UpdatePosText();
        mainHandler.ArduinoResponse(" > " +coord);
    }


    private float GetAngle(PointerEventData eventData) {
        
        Vector2 knobPos = RectTransformUtility.WorldToScreenPoint(
            eventData.pressEventCamera,
            knob.position
        );

        Vector2 dir = eventData.position - knobPos;

        float angle = Mathf.Atan2(dir.y, dir.x) * Mathf.Rad2Deg;

        angle -= 90f;

        return angle;
    }

}