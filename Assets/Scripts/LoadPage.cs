// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using TMPro;

// public class LoadPage : MonoBehaviour {

//     public Transform      Prog_ListContent;
//     public GameObject     Prog_ButtonPrefab;

//     public GameObject     loadProg_SuccessMsg;
//     public TMP_Text       msg_SuccessLoadProg;

//     private string selectedProgram = "";

//     private Dictionary<string, string> Programs;


//     void Start() {
//         Programs = GetComponent<RectTransform>();
//     }


//     IEnumerator HideSuccessMsg() {

//         yield return new WaitForSeconds(3f);

//         loadProg_SuccessMsg.SetActive(false);
//     }


//     public void RefreshProgramList() {

//         // Remove previous buttons
//         foreach(Transform child in Prog_ListContent) {

//             if(child.gameObject != Prog_ButtonPrefab) Destroy(child.gameObject);
//         }

//         // Create one button per saved program
//         foreach(string progName in Programs.Keys) {

//             GameObject button = Instantiate(Prog_ButtonPrefab, Prog_ListContent);
//             button.SetActive(true);

//             TMP_Text buttonText = button.GetComponentInChildren<TMP_Text>();
//             buttonText.text     = progName;

//             UnityEngine.UI.Button btn = button.GetComponent<UnityEngine.UI.Button>();

//             btn.onClick.AddListener(() => SelectProgram(progName));
//         }
//     }

//     public void SelectProgram(string progName) {

//         selectedProgram = progName;

//         // Debug.Log("Selected: " + selectedProgram);
//     }

//     public void LoadSelectedProgram() {

//         if(string.IsNullOrEmpty(selectedProgram)) return;

//         if(Programs.ContainsKey(selectedProgram)) {

//             ProgWindowField.text = Programs[selectedProgram];
//             msg_ProgName   .text = selectedProgram;

//             loadProg_SuccessMsg.SetActive(true);
//             msg_SuccessLoadProg.text = $"< {selectedProgram} > Loaded";

//             StartCoroutine( HideSuccessMsg() );

//             // Debug.Log("Loaded: " + selectedProgram);
//         }
//     }

//     public void DeleteSelectedProgram() {

//         if(string.IsNullOrEmpty(selectedProgram)) return;

//         if(Programs.Remove(selectedProgram)) {
//             selectedProgram = "";

//             SaveData();
//             RefreshProgramList();

//             // Debug.Log("Deleted: " + selectedProgram);
//         }
//     }

// }
