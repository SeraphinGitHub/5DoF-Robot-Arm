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
    public TMP_Text       msg_CurPos;

    public TMP_InputField X_ValueField;
    public TMP_InputField Y_ValueField;
    public TMP_InputField Z_ValueField;
    public TMP_InputField ProgWindowField;

    // Private
    private float  navInterval  = 0.1f;
    private string curPos_str   = "";
    private string serialBuffer = "";

    private Queue<string> serialMessages = new Queue<string>();
    private Coroutine     navCoroutine;


    void Start() {

        OpenSerial();
        msg_Running.SetActive(true );
    }

    void Update() {

        updateCurPosDisplay();
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

    private void updateCurPosDisplay() {

        if(!isSerial()) return;

        serialBuffer += serial.ReadExisting();

        while(serialBuffer.Contains("\n")) {

            int    index    = serialBuffer.IndexOf("\n");
            string response = serialBuffer.Substring(0, index).Trim();
            serialBuffer    = serialBuffer.Substring(index +1);

            if(string.IsNullOrEmpty(response)) continue;

            // Debug.Log("SERIAL RECEIVED: [" + response + "]");

            if(response.StartsWith("POS:")) {
                msg_CurPos.text = curPos_str = response.Substring(4);
            }

            else {
                serialMessages.Enqueue(response);
                // Debug.Log("QUEUE SIZE: " + serialMessages.Count);
            }
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
        
        if(isSerial()) {
            serial.WriteLine("home");
        }
    }

    public void SendProg() {

        string programText    = ProgWindowField.text;
        List<string> ProgList = new List<string>( programText.Replace("\r", "").Split('\n') );
        
        ProgList.RemoveAll(codeLine => string.IsNullOrWhiteSpace(codeLine));
        
        StartCoroutine( RunProgram(ProgList) );
    }


    // Navigate Axis
    public void StopNavRoutine() {

        if(navCoroutine != null) {
            StopCoroutine(navCoroutine);
            navCoroutine = null;
        }
    }

    public void X_Plus() {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>X+"));
    }

    public void X_Minus() {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>X-"));
    }

    public void Y_Plus() {
        
        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>Y+"));
    }

    public void Y_Minus() {
        
        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>Y-"));
    }

    public void Z_Plus() {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>Z+"));
    }

    public void Z_Minus() {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>Z-"));
    }

    public void P_Plus() {
        
        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>P+"));
    }

    public void P_Minus() {
        
        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>P-"));
    }

    public void R_Plus() {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>R+"));
    }

    public void R_Minus() {

        if(navCoroutine != null) StopCoroutine(navCoroutine);
        navCoroutine = StartCoroutine(NavRoutine("nav>R-"));
    }
    

    // Record position
    public void RecordPos() {
        ProgWindowField.text += curPos_str + "\n";
    }

}
