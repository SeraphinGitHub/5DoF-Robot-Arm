using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO.Ports;


public class UI_handler : MonoBehaviour {

    SerialPort serial = new SerialPort("COM5", 115200);
     
    float  nextSendTime = 0f;
    float  delay        = 0.1f; // 100 ms
    float  moveStep     = 10f;

    float  home_X       = 0f;
    float  home_Y       = 150f;
    float  home_Z       = 120f;

    float  prev_X;
    float  prev_Y;
    float  prev_Z;

    string homeCoord;

    public GameObject     msg_Running;
    public TMP_InputField X_Value;
    public TMP_InputField Y_Value;
    public TMP_InputField Z_Value;
    public TMP_InputField ProgWindow;

    
    void Start() {
        msg_Running.SetActive(true);

        homeCoord = (
            "X" + home_X.ToString() + " " +
            "Y" + home_Y.ToString() + " " +
            "Z" + home_Z.ToString()
        );
    }
    

    // Coroutine
    IEnumerator DelayArduinoRes(float delay) {
        
        yield return new WaitForSeconds(delay);

        ArduinoResponse("");
    }

    IEnumerator RunProgram(List<string> GCodeLines) {

        while(GCodeLines.Count > 0) {

            string curLine = GCodeLines[0];
            serial.WriteLine(curLine);

            while(serial.BytesToRead == 0)
            
            yield return null;

            string resp = serial.ReadLine();

            if(resp == "RES:ARRIVED") {
                Debug.Log("ATPOS: " + curLine);
                GCodeLines.RemoveAt(0);
            }
        }

        Debug.Log("STATUS: PROGRAM DONE !");
    }
    

    // Private
    private void  ArduinoResponse(string msg) {

        string response = serial.ReadLine();
        Debug.Log("Arduino => " +response +msg);
    }

    private float StringToFloat(string inputValue) {
        
        float value;

        if(float.TryParse(inputValue, out value)) {

            return value;
        }

        else return 0f;
    }

    private void  SendNextCmd(string cmd) {

        if(Time.time >= nextSendTime) {
            serial.WriteLine(cmd);
            nextSendTime = Time.time +delay;
        }
    }


    // Public
    public void Connect() {

        serial.Open();
        serial.DtrEnable = true;

        StartCoroutine( DelayArduinoRes(2f) );
    }

    public void CoordMode() {

        serial.WriteLine("CMD:COORD");
        ArduinoResponse("");
    }

    public void ProgMode() {

        serial.WriteLine("CMD:PROG");
        ArduinoResponse("");
    }

    public void PlayProgram() {

        serial.WriteLine("CMD:PLAY");
        ArduinoResponse("");
    }

    public void Home() {

        CoordMode();
        SendNextCmd(homeCoord);

        prev_X = home_X;
        prev_Y = home_Y;
        prev_Z = home_Z;

        ArduinoResponse(" > HOME");
    }
    
    public void EditHome() {
        
        homeCoord = (
            "X" + X_Value.text + " " +
            "Y" + Y_Value.text + " " +
            "Z" + Z_Value.text
        );

        Debug.Log("HOME SET TO : " + homeCoord);
    }

    public void SendCoord() {
        
        string coord = (
            "X" + X_Value.text + " " +
            "Y" + Y_Value.text + " " +
            "Z" + Z_Value.text
        );

        prev_X = StringToFloat(X_Value.text);
        prev_Y = StringToFloat(Y_Value.text);
        prev_Z = StringToFloat(Z_Value.text);

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
    }

    public void SendProg() {

        string programText      = ProgWindow.text;
        List<string> GCodeLines =  new List<string>( programText.Replace("\r", "").Split('\n') );
        
        GCodeLines.RemoveAll(codeLine => string.IsNullOrWhiteSpace(codeLine));
        
        StartCoroutine( RunProgram(GCodeLines) );
    }

    // Axis move
    public void X_Plus() {
        
        prev_X += moveStep;
        string coord = ("X" + prev_X.ToString());

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
    }

    public void X_Minus() {
        
        prev_X -= moveStep;
        string coord = ("X" + prev_X.ToString());

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
    }

    public void Y_Plus() {
        
        prev_Y += moveStep;
        string coord = ("Y" + prev_Y.ToString());

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
    }

    public void Y_Minus() {
        
        prev_Y -= moveStep;
        string coord = ("Y" + prev_Y.ToString());

        serial.WriteLine(coord);
        ArduinoResponse(" > " +coord);
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

}
