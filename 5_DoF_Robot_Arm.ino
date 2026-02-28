
#include <Arduino.h>

#include "headers/GcodeParser.h"
#include "headers/ForwardKinematics.h"
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


// const int btn_save = A1;
// const int btn_play = A2;

String userMode = "prog";
// String userMode = "coord";

void setup() {
  Serial.begin(115200);
  
  init_Motion();

  home();

  // pinMode(btn_save, INPUT_PULLUP);
  // pinMode(btn_play, INPUT_PULLUP);

}

void loop() {

  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // VERY IMPORTANT

    if(cmd == "coord" || cmd == "prog") userMode = cmd;

    if(cmd == "home") home();

    if(userMode == "coord" && cmd != "coord") {

      Serial.println(F("UserMode : coord"));
      isProgMode = false;
      CartesianPos coords = parseGcodeLine(cmd);
      setTargetTo(coords);
    }

    if(userMode == "prog" && cmd != "prog") {

      Serial.println(F("UserMode : prog"));
      isProgMode = true;
      if(cmd == "play") runProgram();
    }
  }

  normalizedMove();
}