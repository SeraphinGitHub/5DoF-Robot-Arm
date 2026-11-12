// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using TMPro;


// public class SavePage : MonoBehaviour {
    
//     public GameObject     saveProg_ErrorMsg;
//     public TMP_Text       msg_ErrorSaveProg;

//     public GameObject     saveProg_SuccessMsg;
//     public TMP_Text       msg_SuccessSaveProg;

//     public TMP_InputField NewProgNameField;

//     private int    minChar_Count    = 5;
//     private int    maxChar_Count    = 20;
    
//     private string minChar_msg = "Need at least";
//     private string maxChar_msg = "Enter a name under";


//     IEnumerator HideErrorMsg() {

//         yield return new WaitForSeconds(3f);

//         saveProg_ErrorMsg.SetActive(false);
//     }


//     public void SaveNewProg() {

//         string progName  = NewProgNameField.text;
//         int    charCount = progName.Length;

//         // Name too short
//         if(charCount < minChar_Count) {

//             saveProg_ErrorMsg.SetActive(true);
//             msg_ErrorSaveProg.text = $"{minChar_msg} {minChar_Count} chars";

//             StartCoroutine( HideErrorMsg() );
//             // Debug.Log("Save failed !");

//             return;
//         }

//         // Name too long
//         if(charCount > maxChar_Count) {
            
//             saveProg_ErrorMsg.SetActive(true);
//             msg_ErrorSaveProg.text = $"{maxChar_msg} {maxChar_Count} chars";

//             StartCoroutine( HideErrorMsg() );
//             // Debug.Log("Save failed !");

//             return;
//         }

//         Programs[ progName ]  = ProgWindowField.text;
//         NewProgNameField.text = "";

//         SaveData();
//         RefreshProgramList();

//         saveProg_SuccessMsg.SetActive(true);
//         msg_SuccessSaveProg.text = "Program saved successfully";

//         // Debug.Log("Save success !");
//     }


// }
