
#include <Arduino.h>

#include "headers/GcodeParser.h"
#include "headers/ForwardKinematics.h"
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


char cmd_Buff[64];

int userMode = 0;

void setup() {

  Serial.begin(115200);
  while(!Serial);
  Serial.println(F("RES:CONNECTED"));

  init_Motion();
}

void loop() {

  if(Serial.available()) {

    int cmdLength = Serial.readBytesUntil('\n', cmd_Buff, sizeof(cmd_Buff) -1);
    cmd_Buff[cmdLength] = '\0';

    // remove '\r' if present
    if(cmdLength > 0 && cmd_Buff[cmdLength -1] == '\r') {
      cmd_Buff[cmdLength -1] = '\0';
    }

    // Serial.print("Received: [");
    // Serial.print(cmd_Buff);
    // Serial.println("]");
    
    if(strcmp(cmd_Buff, "CMD:PROG") == 0 ) {
      userMode = 1;
      Serial.println(F("MODE:PROG"));
      return;
    }

    switch(userMode) {
      case 0:   // Coord mode
        isProgMode = false;
        CartesianPos coords = parseGcodeLine(cmd_Buff);
        setTargetTo(coords);

        // Serial.print(F("RES:GO TO "));
        // Serial.println(cmd_Buff);
      break;

      case 1:   // Prog mode
        isProgMode = true;
        if(strcmp(cmd_Buff, "CMD:PLAY") == 0 ) runProgram();
        Serial.println(F("RES:PLAY"));
      break;
    }
  }

  linear_Interpolation();
}