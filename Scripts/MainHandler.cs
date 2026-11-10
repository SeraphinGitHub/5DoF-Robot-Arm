using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;
using System.Globalization;
using TMPro;
using System.IO.Ports;


public class MainHandler : MonoBehaviour {

    [HideInInspector] public SerialPort serial   = new SerialPort("COM5", 115200);
    [HideInInspector] public string lastAxisName = "Z";
    
    // Public
    public TMP_Text       msg_TargetPos;
    public TMP_Text       msg_CurPos;
    public TMP_Text       msg_MoveSpeed;

    public TMP_InputField X_ValueField;
    public TMP_InputField Y_ValueField;
    public TMP_InputField Z_ValueField;
    public TMP_InputField ProgWindowField;

    // Private
    private float  speedStep     = 5f;
    private float  navLineStep   = 30f;
    private float  navAngleStep  = 15f;
    private float  navInterval   = 0.1f;

    private string livePos_str   = "";
    private string moveSpeed_str = "";
    private string serialBuffer  = "";

    private List <string> ProgList       = new List <string>();
    private Queue<string> serialMessages = new Queue<string>();

    private Coroutine     progCoroutine;
    private Coroutine     navCoroutine;


    void Start() {

        OpenSerial();
    }

    void Update() {

        updateDataDisplay();
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
    IEnumerator RunProgram(List<string> ProgList) {

        while(ProgList.Count > 0) {
            
            // Send command
            string curLine = ProgList[0];
            serialMessages.Clear();
            serial.WriteLine("coord>" +curLine);

            msg_TargetPos.text = curLine;

            bool arrived = false;

            while(!arrived) {

                if(serialMessages.Count > 0) {

                    string response = serialMessages.Dequeue();

                    // Debug.Log("DEQUEUED: [" + response + "]");
                    // Debug.Log("QUEUE REMAINING: " + serialMessages.Count);

                    if(response == "RES:ARRIVED") {
                        arrived = true;
                        Debug.Log("ARRIVED: " +curLine);
                    }
                }

                yield return null;
            }

            ProgList.RemoveAt(0);
        }

        Debug.Log("PROGRAM DONE!");
    }
    
    IEnumerator NavRoutine(string navAxis) {
       
        while(true) {
            if(!isSerial()) yield break;

            serial.WriteLine(navAxis);

            yield return new WaitForSeconds(navInterval);
        }
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

    private void updateDataDisplay() {

        if(!isSerial()) return;

        serialBuffer += serial.ReadExisting();

        while(serialBuffer.Contains("\n")) {

            int    index    = serialBuffer.IndexOf("\n");
            string response = serialBuffer.Substring(0, index).Trim();
            serialBuffer    = serialBuffer.Substring(index +1);

            if(string.IsNullOrEmpty(response)) continue;

            // Debug.Log("SERIAL RECEIVED: [" + response + "]");

            updateDisplay_MoveSpeed(response);

            if(response.StartsWith("POS:")) {
                msg_CurPos.text = livePos_str = response.Substring(4);
            }

            else {
                serialMessages.Enqueue(response);
                // Debug.Log("QUEUE SIZE: " + serialMessages.Count);
            }
        }
    }

    private void updateDisplay_MoveSpeed(string response) {

        if(response.StartsWith("maxSpeed:")) {
            moveSpeed_str      = response.Substring(9);
            msg_MoveSpeed.text = moveSpeed_str + " mm/s";
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
                // Debug.Log(response);
                
                // Update display data after Init
                updateDisplay_MoveSpeed(response);
            }

            catch(System.TimeoutException) {
                Debug.LogWarning("Arduino initialization timed out !");
            }
        }
    }

    public void ArduinoResponse(string msg) {

        if(!isSerial()) return;

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

    public void SendProg() {

        string programText    = ProgWindowField.text;
        ProgList = new List<string>( programText.Replace("\r", "").Split('\n') );
        
        ProgList.RemoveAll(codeLine => string.IsNullOrWhiteSpace(codeLine));
        
        if(progCoroutine != null) return;
        
        progCoroutine = StartCoroutine( RunProgram(ProgList) );
    }

    public void StopProgRoutine() {

        if(progCoroutine != null) {
            StopCoroutine(progCoroutine);
            progCoroutine = null;
        }
    }


    // Navigate Axis
    public void StopNavRoutine() {

        if(navCoroutine != null) {
            StopCoroutine(navCoroutine);
            navCoroutine = null;
        }
    }

    public void NavMove(string paramCMD, string lastAxis) {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine( NavRoutine(paramCMD) );
        lastAxisName = lastAxis;
    }
    
    public void X_Plus() {

        NavMove("nav>X+:" +navLineStep, "X");
    }

    public void X_Minus() {

        NavMove("nav>X-:" +navLineStep, "X");
    }

    public void Y_Plus() {
        
        NavMove("nav>Y+:" +navLineStep, "Y");
    }

    public void Y_Minus() {
        
        NavMove("nav>Y-:" +navLineStep, "Y");
    }

    public void Z_Plus() {

        NavMove("nav>Z+:" +navLineStep, "Z");
    }

    public void Z_Minus() {

        NavMove("nav>Z-:" +navLineStep, "Z");
    }

    public void P_Plus() {
        
        NavMove("nav>P+:" +navAngleStep, "P");
    }

    public void P_Minus() {
        
        NavMove("nav>P-:" +navAngleStep, "P");
    }

    public void R_Plus() {

        NavMove("nav>R+:" +navAngleStep, "R");
    }

    public void R_Minus() {

        NavMove("nav>R-:" +navAngleStep, "R");
    }
    

    // Record position
    public void RecordPos() {
        ProgWindowField.text += livePos_str + "\n";
    }

    
    // Change move speed
    public void ChangeSpeed(int value) {

        float moveSpeed = float.Parse(
            moveSpeed_str.Trim(),
            System.Globalization.CultureInfo.InvariantCulture
        );

        if(value > 0) moveSpeed += speedStep;
        else          moveSpeed -= speedStep;

        if(moveSpeed < 1f || moveSpeed > 200f) return;

        if(isSerial()) serial.WriteLine("set>maxSpeed:" +moveSpeed);
    }

    public void StopProg() {

        ProgList.Clear();
        StopProgRoutine();

        if(isSerial()) serial.WriteLine("coord>" + livePos_str);
    }

}
