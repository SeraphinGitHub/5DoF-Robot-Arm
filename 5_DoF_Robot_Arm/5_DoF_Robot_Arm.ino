
#include <Arduino.h>

#include "headers/GcodeParser.h"
#include "headers/ForwardKinematics.h"
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


bool hasInit = false;
char cmd_Buff[64];


void setup() {

  Serial.begin(115200);

  delay(200);
}


void loop() {

  if(Serial.available()) {

    String cmd = Serial.readStringUntil('\n');

    if(cmd == "init" && !hasInit) {
      init_Motion();
      hasInit = true;
      return;
    }

    // **********************
    ManuTestServo(cmd);
    // **********************

    int cmdLength = Serial.readBytesUntil('\n', cmd_Buff, sizeof(cmd_Buff) -1);
    cmd_Buff[cmdLength] = '\0';

    // remove '\r' if present
    if(cmdLength > 0 && cmd_Buff[cmdLength -1] == '\r') {
      cmd_Buff[cmdLength -1] = '\0';
    }

    // Serial.print("Received: [");
    // Serial.print(cmd_Buff);
    // Serial.println("]");
    
    // CartesianPos coords = parseGcodeLine(cmd_Buff);
    // setTargetTo(coords);

    // Serial.print(F("RES:GO TO "));
    // Serial.println(cmd_Buff);

  }

  // linear_Interpolation();
}

