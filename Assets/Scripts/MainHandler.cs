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
    [Header("Main Page")]
    public GameObject     mainPage;
    public GameObject     savePage;
    public GameObject     loadPage;

    public TMP_Text       msg_TargetPos;
    public TMP_Text       msg_CurPos;
    public TMP_Text       msg_MoveSpeed;
    public TMP_Text       msg_ProgName;
    
    public TMP_InputField ProgWindowField;

    // Private
    private bool   isRunning     = false;

    private float  speedStep     = 5f;
    private float  navLineStep   = 30f;
    private float  navAngleStep  = 15f;
    private float  navInterval   = 0.1f;

    private string livePos_str   = "";
    private string moveSpeed_str = "";
    private string serialBuffer  = "";

    Dictionary<string, string> Programs  = new Dictionary<string, string>();

    private List <string> GCodeLineList  = new List <string>();
    private Queue<string> serialMessages = new Queue<string>();

    private Coroutine     progCoroutine;
    private Coroutine     navCoroutine;


    void Start() {
        
        LoadData();

        ShowPage_MainPage();
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
                        arrived   = true;
                        Debug.Log("ARRIVED: " +curLine);
                    }
                }

                yield return null;
            }

            ProgList.RemoveAt(0);
        }

        isRunning = false;
        Debug.Log("PROGRAM DONE!");
    }
    
    IEnumerator NavRoutine(string navAxis) {
       
        while(true) {
            if(!isSerial()) yield break;

            serial.WriteLine(navAxis);

            yield return new WaitForSeconds(navInterval);
        }
    }


    // =====================================================================================
    // Private
    // =====================================================================================
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


    // =====================================================================================
    // Public
    // =====================================================================================
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
        
        if(!isRunning && isSerial()) serial.WriteLine("home");
    }


    // Prog window
    public void SendProg() {

        if(isRunning) return;

        isRunning = true;
        
        string programText = ProgWindowField.text;
        GCodeLineList      = new List<string>( programText.Replace("\r", "").Split('\n') );
        
        GCodeLineList.RemoveAll(codeLine => string.IsNullOrWhiteSpace(codeLine));
        
        progCoroutine = StartCoroutine( RunProgram(GCodeLineList) );
    }

    public void StopProgRoutine() {

        if(progCoroutine != null) {
            StopCoroutine(progCoroutine);
            progCoroutine = null;
        }
    }

    public void DeleteLastCMD() {

        if(isRunning) return;

        string progText = ProgWindowField.text.TrimEnd('\r', '\n');
        int    lastLine = progText.LastIndexOf('\n');

        if(lastLine >= 0) ProgWindowField.text = ProgWindowField.text.Substring(0, lastLine +1);
        else              ProgWindowField.text = "";
    }

    public void ClearProgWindow() {

        if(isRunning) return;

        ProgWindowField.text = "";
    }


    // Navigate Axis
    public void StopNavRoutine() {

        if(navCoroutine != null) {
            StopCoroutine(navCoroutine);
            navCoroutine = null;
        }
    }

    public void NavMove(string paramCMD, string lastAxis) {
       
        if(isRunning) return;

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

        if(isRunning) return;

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

        GCodeLineList.Clear();
        StopProgRoutine();

        if(isSerial()) serial.WriteLine("coord>" + livePos_str);
    }


    // Swap page
    public void ShowPage_MainPage() {

        mainPage.SetActive(true);
        savePage.SetActive(false);
        loadPage.SetActive(false);

        saveProg_ErrorMsg  .SetActive(false);
        saveProg_SuccessMsg.SetActive(false);
        loadProg_SuccessMsg.SetActive(false);
    }

    public void ShowPage_SaveProg() {

        mainPage.SetActive(false);
        savePage.SetActive(true);
    }

    public void ShowPage_LoadProg() {

        mainPage.SetActive(false);
        loadPage.SetActive(true);

        LoadData();
        RefreshProgramList();
    }


    // Save & Load progs in PlayerPrefabs
    public void SaveData() {

        PlayerPrefs.SetInt("ProgramCount", Programs.Count);

        int i = 0;

        foreach (var p in Programs) {
            PlayerPrefs.SetString("ProgramName_"    + i, p.Key  );
            PlayerPrefs.SetString("ProgramContent_" + i, p.Value);
            i++;
        }

        PlayerPrefs.Save();

        // Debug.Log("SAVED: " + Programs.Count);
    }

    public void LoadData() {
        
        Programs.Clear();

        int count = PlayerPrefs.GetInt("ProgramCount", 0);

        for (int i = 0; i < count; i++) {
            string name    = PlayerPrefs.GetString("ProgramName_"    + i, "");
            string content = PlayerPrefs.GetString("ProgramContent_" + i, "");

            if (!string.IsNullOrEmpty(name))  Programs[name] = content;
        }

        // Debug.Log("LOADED: " + Programs.Count);

        RefreshProgramList();
    }


    // =====================================================================================
    // Save Prog Page
    // =====================================================================================
    [Header("Save Prog Page")]
    public GameObject     saveProg_ErrorMsg;
    public TMP_Text       msg_ErrorSaveProg;

    public GameObject     saveProg_SuccessMsg;
    public TMP_Text       msg_SuccessSaveProg;

    public TMP_InputField NewProgNameField;

    private int    progNameLimit   = 20;
    private string charLimit_msg   = "Name need to be under";
    

    public void SaveNewProg() {

        string progName  = NewProgNameField.text;
        int    charCount = progName.Length;

        if(charCount > progNameLimit) {
            
            saveProg_ErrorMsg.SetActive(true);
            msg_ErrorSaveProg.text = $"{charLimit_msg} {progNameLimit}";
            
            Debug.Log("Save failed !");
            return;
        }

        Programs[ progName ]  = ProgWindowField.text;
        NewProgNameField.text = "";

        SaveData();
        RefreshProgramList();

        saveProg_SuccessMsg.SetActive(true);
        msg_SuccessSaveProg.text = "Program saved successfully";

        Debug.Log("Save success !");
    }


    // =====================================================================================
    // Load Prog Page
    // =====================================================================================
    [Header("Load Prog Page")]
    public Transform      Prog_ListContent;
    public GameObject     Prog_ButtonPrefab;

    public GameObject     loadProg_SuccessMsg;
    public TMP_Text       msg_SuccessLoadProg;

    private string selectedProgram = "";


    IEnumerator HideSuccessMsg() {

        yield return new WaitForSeconds(3f);

        loadProg_SuccessMsg.SetActive(false);
    }


    public void RefreshProgramList() {

        // Remove previous buttons
        foreach (Transform child in Prog_ListContent) {

            if(child.gameObject != Prog_ButtonPrefab) Destroy(child.gameObject);
        }

        // Create one button per saved program
        foreach (string progName in Programs.Keys) {

            GameObject button = Instantiate(Prog_ButtonPrefab, Prog_ListContent);
            button.SetActive(true);

            TMP_Text buttonText = button.GetComponentInChildren<TMP_Text>();
            buttonText.text     = progName;

            UnityEngine.UI.Button btn = button.GetComponent<UnityEngine.UI.Button>();

            btn.onClick.AddListener(() => SelectProgram(progName));
        }
    }

    public void SelectProgram(string progName) {

        selectedProgram = progName;

        Debug.Log("Selected: " + selectedProgram);
    }

    public void LoadSelectedProgram() {

        if(string.IsNullOrEmpty(selectedProgram)) return;

        if(Programs.ContainsKey(selectedProgram)) {

            ProgWindowField.text = Programs[selectedProgram];
            msg_ProgName   .text = selectedProgram;

            loadProg_SuccessMsg.SetActive(true);
            msg_SuccessLoadProg.text = $"< {selectedProgram} > Loaded";

            StartCoroutine(HideSuccessMsg());

            Debug.Log("Loaded: " + selectedProgram);
        }
    }

    public void DeleteSelectedProgram() {

        if(string.IsNullOrEmpty(selectedProgram)) return;

        if(Programs.Remove(selectedProgram)) {
            selectedProgram = "";

            SaveData();
            RefreshProgramList();

            Debug.Log("Deleted: " + selectedProgram);
        }
    }


}
