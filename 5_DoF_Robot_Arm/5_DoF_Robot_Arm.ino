
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
    if(strcmp(cmd_Buff, "init") == 0 && !hasInit) { initController();        return; }
    if(strcmp(cmd_Buff, "home") == 0            ) { setTargetTo   (homePos); return; }
    if(strcmp(cmd_Buff, "demo") == 0            ) { startDemo     ();        return; }
    if(strcmp(cmd_Buff, "end" ) == 0            ) { endDemo       ();        return; }

    char* delimiter = strchr(cmd_Buff, '>');
    if(delimiter == nullptr) return;

    *delimiter    = '\0';
    char* varName = cmd_Buff;
    char* value   = delimiter +1;

    if(strcmp(cmd_Buff, "set"  ) == 0) { changeVarValue  (value); return; }
    if(strcmp(cmd_Buff, "nav"  ) == 0) { navigate        (value); return; }
    if(strcmp(cmd_Buff, "coord") == 0) { processTargetPos(value); return; }
    // manuAngleMove (cmd_Buff);

  }

  linearInterpolation();
  updateDemo();
  sendPos_OnChange();

}

