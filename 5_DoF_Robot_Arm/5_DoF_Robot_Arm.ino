
#include <Arduino.h>

#include "headers/GcodeParser.h"
#include "headers/ForwardKinematics.h"
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"

char cmd_Buff[64];


void setup() {
  Serial.begin(115200);
  delay(200);
}


void loop() {

  if(Serial.available()) {
    int cmdLength = Serial.readBytesUntil('\n', cmd_Buff, sizeof(cmd_Buff) -1);
    cmd_Buff[cmdLength] = '\0';

    // remove '\r' if present
    if(cmdLength > 0 && cmd_Buff[cmdLength -1] == '\r')  cmd_Buff[cmdLength -1] = '\0';

    // Serial.print("Received: [");
    // Serial.print(cmd_Buff);
    // Serial.println("]");

    // Initialize servos at physical current pos
    if(isMatch(cmd_Buff, "init") && !hasInit) { initController();        return; }
    if(isMatch(cmd_Buff, "home")            ) { setTargetTo   (homePos); return; }
    if(isMatch(cmd_Buff, "demo")            ) { startDemo     ();        return; }
    if(isMatch(cmd_Buff, "end" )            ) { endDemo       ();        return; }

    char* delimiter = strchr(cmd_Buff, '>');
    if(delimiter == nullptr) return;

    *delimiter    = '\0';
    char* varName = cmd_Buff;
    char* value   = delimiter +1;

    if(isMatch(cmd_Buff, "set"  )) { changeVarValue  (value); return; }
    if(isMatch(cmd_Buff, "nav"  )) { navigate        (value); return; }
    if(isMatch(cmd_Buff, "coord")) { processTargetPos(value); return; }
    // manuAngleMove (cmd_Buff);

  }

  linearInterpolation();
  updateDemo();
  sendPos_OnChange();

}

