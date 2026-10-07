
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

    // **********************
    changeVarValue(cmd_Buff);
    // manuAngleMove (cmd_Buff);
    // **********************

    Position newPos = parseGcodeLine(cmd_Buff);
    setTargetTo(newPos);

    Serial.print(F("RES:GO TO "));
    Serial.println(cmd_Buff);

  }

  linearInterpolation();
  updateDemo();

}

