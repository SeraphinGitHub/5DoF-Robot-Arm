using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using TMPro;
using System.IO.Ports;


public class MainHandler : MonoBehaviour {

    [HideInInspector] public SerialPort serial = new SerialPort("COM5", 115200);
    
    // Public
    public GameObject     msg_Running;
    public GameObject     msg_CurPos;

    public TMP_InputField X_ValueField;
    public TMP_InputField Y_ValueField;
    public TMP_InputField Z_ValueField;
    public TMP_InputField ProgWindowField;

    // Private
    private string homeCoord;
    private float  nextSendTime = 0f;
    private float  moveStep     = 10f;

    [HideInInspector] public float  prev_X;
    [HideInInspector] public float  prev_Y;
    [HideInInspector] public float  prev_Z;
    

    void Start() {

        OpenSerial();
        msg_Running.SetActive(true);
    }
    

    // Var Methods
    private float StringToFloat(string inputValue) {
        
        float value;

        if(float.TryParse(inputValue, out value)) {

            return value;
        }

        else return 0f;
    }

    private bool isSerial() {
        
        if(serial != null && serial.IsOpen) return true;
        
        else return false;
    }

    // Coroutines
    IEnumerator RunProgram(List<string> GCodeLines) {

        while(GCodeLines.Count > 0) {

            string curLine = GCodeLines[0];
            serial.WriteLine(curLine);

            bool arrived = false;

            while(!arrived) {

                if(serial.BytesToRead > 0) {
                    string response = serial.ReadLine().Trim();

                    if(response == "RES:ARRIVED") {
                        arrived = true;
                        Debug.Log("ATPOS: " + curLine);
                    }
                }

                yield return null;
            }

            GCodeLines.RemoveAt(0);
        }

        Debug.Log("STATUS: PROGRAM DONE !");
    }
    

    // Private
    private void OnDisable() {
        StopAllCoroutines();
        CloseSerial();
    }

    private void OnApplicationQuit() {
        CloseSerial();
    }

    private void CloseSerial() {
        
        try {
            if(isSerial()) serial.Close();
        }

        catch(System.Exception error) {
            Debug.LogWarning("Serial closing error: " + error.Message);
        }
    }

    private void SendNextCmd(string cmd) {

        if(Time.time >= nextSendTime) {
            serial.WriteLine(cmd);
            nextSendTime = Time.time +0.1f;
        }
    }


    // Public
    public void OpenSerial() {

        try {
            if(serial == null) return;

            if(!serial.IsOpen) {
                serial.ReadTimeout  = 2500;
                serial.WriteTimeout = 500;

                serial.DtrEnable = true;
                serial.Open();
            }
        }

        catch(System.Exception error) {
            Debug.LogError("Serial connection error: " + error.Message);
        }
    }

    public void Connect() {

        if(isSerial()) {

            serial.WriteLine("init");

            try {
                string response = serial.ReadLine().Trim();
                Debug.Log(response);
                if(response == "RES:INITILIZED") serial.WriteLine("home");
            }

            catch(System.TimeoutException) {
                Debug.LogWarning("Arduino initialization timed out !");
            }
        }
    }

    public void ArduinoResponse(string msg) {

        if(serial == null || !serial.IsOpen) return;

        try {
            string response = serial.ReadLine();
            Debug.Log("Arduino => " + response + msg);
        }

        catch (System.TimeoutException) {
            Debug.LogWarning("Arduino response timeout");
        }
        
        catch (System.Exception error) {
            Debug.LogError("Serial reading error: " + error.Message);
        }
    }

    public void Home() {
        
        if(isSerial()) serial.WriteLine("home");
    }

    public void SendCoord() {
        
        string coord = (
            "X" + X_ValueField.text + " " +
            "Y" + Y_ValueField.text + " " +
            "Z" + Z_ValueField.text
        );

        prev_X = StringToFloat(X_ValueField.text);
        prev_Y = StringToFloat(Y_ValueField.text);
        prev_Z = StringToFloat(Z_ValueField.text);

        serial.WriteLine(coord);
        Debug.Log(coord);
    }

    public void SendProg() {

        string programText      = ProgWindowField.text;
        List<string> GCodeLines =  new List<string>( programText.Replace("\r", "").Split('\n') );
        
        GCodeLines.RemoveAll(codeLine => string.IsNullOrWhiteSpace(codeLine));
        
        StartCoroutine( RunProgram(GCodeLines) );
    }


    // Axis move
    public void X_Plus() {

        // if(isSerial()) serial.WriteLine("nav_X:10");
        
        prev_X += moveStep;
        string coord = ("X" + prev_X.ToString());

        serial.WriteLine(coord);
        // ArduinoResponse(" > " +coord);
    }

    public void X_Minus() {
        
        // if(isSerial()) serial.WriteLine("nav_X:-10");

        prev_X -= moveStep;
        string coord = ("X" + prev_X.ToString());

        serial.WriteLine(coord);
        // ArduinoResponse(" > " +coord);
    }

    public void Y_Plus() {
        
        // if(isSerial()) serial.WriteLine("nav_Y:10");

        prev_Y += moveStep;
        string coord = ("Y" + prev_Y.ToString());

        serial.WriteLine(coord);
        // ArduinoResponse(" > " +coord);
    }

    public void Y_Minus() {
        
        // if(isSerial()) serial.WriteLine("nav_Y:-10");

        prev_Y -= moveStep;
        string coord = ("Y" + prev_Y.ToString());

        serial.WriteLine(coord);
        // ArduinoResponse(" > " +coord);
    }

    public void Z_Plus() {
        
        prev_Z += moveStep;
        string coord = ("Z" + prev_Z.ToString());

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
    }

    public void Z_Minus() {
        
        prev_Z -= moveStep;
        string coord = ("Z" + prev_Z.ToString());

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
    }
    
    public void RecordPos() {

        string coord = (
           " X: " + prev_X.ToString() +
           " Y: " + prev_Y.ToString() +
           " Z: " + prev_Z.ToString()
        );

        ProgWindowField.text += coord;
    }

    public void UpdatePosText() {

        msg_CurPos.GetComponent<TMP_Text>().text = 
           " X: " + prev_X.ToString() +
           " Y: " + prev_Y.ToString() +
           " Z: " + prev_Z.ToString()
        ;
    }
}
