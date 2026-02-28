using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO.Ports;


public class UI_handler : MonoBehaviour {

    SerialPort serial = new SerialPort("COM5", 115200);
     
    float  moveStep     = 10f;

    float  home_X       = 0f;
    float  home_Y       = 150f;
    float  home_Z       = 120f;

    float  prev_X;
    float  prev_Y;
    float  prev_Z;

    string homeCoord;

    float  nextSendTime = 0f;
    float  delay        = 0.1f; // 100 ms

    public TMP_InputField X_Value;
    public TMP_InputField Y_Value;
    public TMP_InputField Z_Value;

    
    void Start() {
        homeCoord = (
            "X" + home_X.ToString() + " " +
            "Y" + home_Y.ToString() + " " +
            "Z" + home_Z.ToString()
        );
    }
    

    // Private
    private float StringToFloat(string inputValue) {
        
        float value;

        if(float.TryParse(inputValue, out value)) {

            return value;
        }

        else return 0f;
    }

    private void SendNextCmd(string cmd) {

        if(Time.time >= nextSendTime) {
            serial.WriteLine(cmd);
            nextSendTime = Time.time +delay;
        }
    }


    // Public
    public void Connect() {

        serial.Open();
        serial.DtrEnable = true;
        Debug.Log("Connected !");
    }

    public void CoordMode() {

        serial.WriteLine("coord");
        Debug.Log("Coord mode !");
    }

    public void ProgMode() {

        serial.WriteLine("prog");
        Debug.Log("Prog mode !");
    }

    public void PlayProgram() {

        serial.WriteLine("play");
        Debug.Log("Playing program !");
    }

    public void Home() {

        CoordMode();
        SendNextCmd(homeCoord);

        prev_X = home_X;
        prev_Y = home_Y;
        prev_Z = home_Z;

        Debug.Log("Homing !");
    }

    public void GoTo_Gcode() {
        
        string coord = (
            "X" + X_Value.text + " " +
            "Y" + Y_Value.text + " " +
            "Z" + Z_Value.text
        );

        prev_X = StringToFloat(X_Value.text);
        prev_Y = StringToFloat(Y_Value.text);
        prev_Z = StringToFloat(Z_Value.text);

        serial.WriteLine(coord);
        Debug.Log("Move to : " + coord);
    }

    // Axis move
    public void X_Plus() {
        
        prev_X += moveStep;
        string coord = ("X" + prev_X.ToString());

        serial.WriteLine(coord);
        Debug.Log("X + " +moveStep);
    }

    public void X_Minus() {
        
        prev_X -= moveStep;
        string coord = ("X" + prev_X.ToString());

        serial.WriteLine(coord);
        Debug.Log("X - " +moveStep);
    }

    public void Y_Plus() {
        
        prev_Y += moveStep;
        string coord = ("Y" + prev_Y.ToString());

        serial.WriteLine(coord);
        Debug.Log("Y + " +moveStep);
    }

    public void Y_Minus() {
        
        prev_Y -= moveStep;
        string coord = ("Y" + prev_Y.ToString());

        serial.WriteLine(coord);
        Debug.Log("Y - " +moveStep);
    }

    public void Z_Plus() {
        
        prev_Z += moveStep;
        string coord = ("Z" + prev_Z.ToString());

        serial.WriteLine(coord);
        Debug.Log("Z + " +moveStep);
    }

    public void Z_Minus() {
        
        prev_Z -= moveStep;
        string coord = ("Z" + prev_Z.ToString());

        serial.WriteLine(coord);
        Debug.Log("Z - " +moveStep);
    }
}
