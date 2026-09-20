
#include <Arduino.h>

#include "headers/GcodeParser.h"
#include "headers/ForwardKinematics.h"
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


bool demoRunning = false;
int demoStep = 0;

void startDemo() {
  demoRunning = true;
  demoStep = 0;

  setTargetTo(parseGcodeLine("X150 Y200 Z300"));
}

void updateDemo() {

  if (!demoRunning) return;

  if (isMoving) return;

  demoStep++;

  if (demoStep >= 6) {
    demoStep = 0;   // repeat forever
  }

  switch (demoStep) {

    case 0:
      setTargetTo(parseGcodeLine("X150 Y200 Z300"));
      break;

    case 1:
      setTargetTo(parseGcodeLine("X-100 Y150 Z180"));
      wrist_roll_angle = 45;
      break;

    case 2:
      setTargetTo(parseGcodeLine("X-200 Y100 Z300"));
      wrist_roll_angle = 60;
      break;

    case 3:
      setTargetTo(parseGcodeLine("X150 Y200 Z300"));
      wrist_roll_angle = 90;
      break;

    case 4:
      setTargetTo(parseGcodeLine("Y100 Z100"));
      wrist_roll_angle = 135;
      break;

    case 5:
      setTargetTo(parseGcodeLine("X150 Y200 Z300"));
      wrist_roll_angle = 90;
      break;
  }
}





bool hasInit = false;
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
    if(cmdLength > 0 && cmd_Buff[cmdLength -1] == '\r') {
      cmd_Buff[cmdLength -1] = '\0';
    }

    // Serial.print("Received: [");
    // Serial.print(cmd_Buff);
    // Serial.println("]");

    // Initialize servos at physical current pos
    if(strcmp(cmd_Buff, "init") == 0 && !hasInit) {
      initController();
      hasInit = true;
      return;
    }

    if(strcmp(cmd_Buff, "home") == 0) {
      setTargetTo(homeCoords);
      return;
    }

    if(strcmp(cmd_Buff, "demo") == 0) {
      startDemo();
      return;
    }

    if(strcmp(cmd_Buff, "end") == 0) {
      demoRunning = false;
      setTargetTo(homeCoords);
      return;
    }

    // **********************
    // manuAngleMove(cmd_Buff);
    // **********************

    CartesianPos coords = parseGcodeLine(cmd_Buff);
    setTargetTo(coords);

    Serial.print(F("RES:GO TO "));
    Serial.println(cmd_Buff);

  }

  linear_Interpolation();
  updateDemo();

}

