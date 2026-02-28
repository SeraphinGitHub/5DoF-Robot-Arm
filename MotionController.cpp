
#include <Servo.h>
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"
#include "headers/InverseKinematics.h"
#include "headers/GCodePrograms.h"


String       home_Gcode = "G1 X0 Y0 Z150";
CartesianPos homeCoords = { 0, 0, 150, 1 };
CartesianPos currentPos = homeCoords;
CartesianPos targetPos;

float offsetPos[] = { 0, 0, -8 };


// ====================================================
// Robot dimensions (mm)
// ====================================================
const Rig rig = {
  78,   // c - Base height
  140,  // l - Arm lengths (both the same size)
  10,   // g - Y tool's offset
  50,   // f - Z tool's offset

  8,    // Base offset in degrees
  30,   // Shoulder offset in degrees
  25,   // Elbow offset in degrees
  25,   // Wrist offset in degrees
};


// ====================================================
// Constants
// ====================================================
Servo servo_base;
Servo servo_shoulder_L;
Servo servo_shoulder_R;
Servo servo_elbow_L;
Servo servo_elbow_R;
Servo servo_wrist_L;
Servo servo_wrist_R;
Servo servo_wrist_roll;

const float reachRange   = 1;
const float moveSpeed    = 80.0;  // mm/s (smaller more precise)
const float stepInterval = 5.0;   // ms   (ms between steps > smaller more precise)


// ====================================================
// Vars
// ====================================================
bool isHome          = false;
bool isMoving        = false;
bool isProgMode      = false;

int  progIndex       = 0;
unsigned long lastStepTime = 0;

// Previous position in Degrees
int prevBase         = -1;
int prevShoulder     = -1;
int prevElbow        = -1;
int prevWrist        = -1;
int prevWristRoll    = -1;

// Start position in Degrees
int start_Base       = 98;
int start_Shoulder   = 180;
int start_Elbow      = 11;
int start_Wrist      = 25;
int start_Wrist_roll = 90;


// ====================================================
// Setup
// ====================================================
void init_Motion() {

  servo_base       .attach(3);
  servo_shoulder_R .attach(4);
  servo_elbow_L    .attach(5);
  servo_elbow_R    .attach(6);
  servo_wrist_R    .attach(7);

  delay(100);

  servo_base       .write(start_Base);
  servo_shoulder_R .write(180 -start_Shoulder);
  servo_elbow_L    .write(start_Elbow);
  servo_elbow_R    .write(180 -start_Elbow);
  servo_wrist_R    .write(start_Wrist);
  

  // servo_base       .attach(3);

  // servo_shoulder_L .attach(4);
  // servo_shoulder_R .attach(5);

  // servo_elbow_L    .attach(6);
  // servo_elbow_R    .attach(7);

  // servo_wrist_L    .attach(8);
  // servo_wrist_R    .attach(9);
  
  // servo_wrist_roll .attach(10);

  // delay(100);

  // servo_base       .write(start_Base);

  // servo_shoulder_L .write(start_Shoulder);
  // servo_shoulder_R .write(180 -start_Shoulder);

  // servo_elbow_L    .write(start_Elbow);
  // servo_elbow_R    .write(180 -start_Elbow);

  // servo_wrist_L    .write(start_Wrist);
  // servo_wrist_R    .write(180 -start_Wrist);

  // servo_wrist_roll .write(start_Wrist_roll);

  delay(100);
}


// ====================================================
// Program modes
// ====================================================
void home() {

  if(isHome) return;

  isHome = true;

  currentPos.x        = homeCoords.x;
  currentPos.y        = homeCoords.y;
  currentPos.z        = homeCoords.z;
  currentPos.moveType = homeCoords.moveType;

  // setTargetTo(homeCoords);
}

void runProgram() {
  
  // String* program        = nullptr;
  // int     program_Length = 0;
  // progIndex              = 0;

  // switch(progID) {
  //   case "play-01":
  //     program        = program_001;
  //     program_Length = program_001_Length;
  //   break;

  //   case "play-02":
  //     program        = program_002;
  //     program_Length = program_002_Length;
  //   break;

  //   case "play-03":
  //     program        = program_003;
  //     program_Length = program_003_Length;
  //   break;
  // }

  // if(program != nullptr && progIndex < program_Length) {
  //   setTargetTo(parseGcodeLine(program[progIndex]));
  //   progIndex++;
  // }

  if(progIndex < program_002_Length) {
    setTargetTo( parseGcodeLine( program_002[progIndex] ) );
    progIndex++;
  }
  
  else {
    progIndex = 0;
    Serial.println(F("Program complete !"));
  }
}

void setTargetTo(CartesianPos coords) {

  targetPos    = coords;
  isMoving     = true;
  lastStepTime = millis(); // reset step timer
}

bool isNewValue(int &prevAngle, int newAngle) {

  if(prevAngle == newAngle) return false;

  prevAngle = newAngle;
  return true;
}

void moveServosTo(CartesianPos coords) {

  JointAngles angles = inverseKinematics( coords.x, coords.y, coords.z );

  // Safe limit
  if(  isnan(angles.tau)
    || isnan(angles.gamma)
    || isnan(angles.lambda)
    || isnan(angles.epsilon)
  ) {
    return;
  }

  int ang_Base     = (int)rig.offset_0 +(int)angles.epsilon;
  int min_Base     = rig.offset_0;
  int max_Base     = 180;

  int ang_Shoulder = (int)rig.offset_1 +(int)angles.tau;
  int min_Shoulder = (int)rig.offset_1;
  int max_Shoulder = 180;

  int ang_Elbow    = (int)angles.gamma -(int)rig.offset_2;
  int min_Elbow    = 0;
  int max_Elbow    = 180 -(int)rig.offset_2;

  int ang_Wrist    = (int)angles.lambda -(int)rig.offset_3;
  int min_Wrist    = 0;
  int max_Wrist    = 180;


  // Joint angles bounderies
  int safeBase     = constrain( ang_Base,      min_Base,      max_Base     );
  int safeShoulder = constrain( ang_Shoulder,  min_Shoulder,  max_Shoulder );
  int safeElbow    = constrain( ang_Elbow,     min_Elbow,     max_Elbow    );
  int safeWrist    = constrain( ang_Wrist,     min_Wrist,     max_Wrist    );


  if(isNewValue(prevBase, safeBase)) {
    servo_base.write(safeBase);
  }

  if(isNewValue(prevShoulder, safeShoulder)) {
    // servo_shoulder_L  .write(     safeShoulder);
    servo_shoulder_R  .write(180 -safeShoulder);
  }
  
  if(isNewValue(prevElbow, safeElbow)) {
    servo_elbow_L     .write(     safeElbow);
    servo_elbow_R     .write(180 -safeElbow);
  }

  if(isNewValue(prevWrist, safeWrist)) {
    // servo_wrist_L     .write(     safeWrist);
    servo_wrist_R     .write(180 -safeWrist);
  }


  // Serial.print("epsilon (safeBase) : ");
  // Serial.print(angles.epsilon);
  // Serial.print("°, ");
  // // Serial.print(safeBase);

  // Serial.print("°, tau (safeShoulder) : ");
  // Serial.print(angles.tau);
  // Serial.print("°, ");
  // // Serial.print(safeShoulder);
  
  // Serial.print("°, gamma (safeElbow) : ");
  // Serial.print(angles.gamma);
  // Serial.print("°, ");
  // // Serial.print(safeElbow);
  
  // Serial.print("°, lambda (safeWrist) : ");
  // Serial.print(angles.lambda);
  // Serial.print("°, ang_Wrist (safeWrist) : ");
  // Serial.print(ang_Wrist);
  // // Serial.print("°, ");
  // // Serial.print(safeWrist);
  // Serial.println("°, ");

  // Serial.print("°, moveType : ");
  // Serial.println(angles.moveType);

  // Joint1: 0° to 180° (base rotate)
  // Joint2: 0° to 180°
  // Joint3: 0° to 180°
  // Sample every 10°:
  // for (int a1 = 0; a1 <= 180; a1 += 10) {
  //   for (int a2 = 0; a2 <= 180; a2 += 10) {
  //     for (int a3 = 0; a3 <= 180; a3 += 10) {
  //       CartesianPos pos = forwardKinematics(a1, a2, a3);
  //       if (isValid(pos)) {
  //         storeInWorkspace(pos); // Add to reachable point list
  //       }
  //     }
  //   }
  // }
}

void normalizedMove() {

  unsigned long now = millis();

  if(!isMoving || now -lastStepTime < stepInterval) return;

  lastStepTime = now;

  float dx = targetPos.x - currentPos.x + offsetPos[0];
  float dy = targetPos.y - currentPos.y + offsetPos[1];
  float dz = targetPos.z - currentPos.z + offsetPos[2];

  float dist = sqrt(dx*dx + dy*dy + dz*dz);
  
  // Serial.print("Dist : "); Serial.println(dist);
  // Serial.print("Targ X : "); Serial.println(targetPos.x);
  // Serial.print("Targ Y : "); Serial.println(targetPos.y);
  // Serial.print("Targ Z : "); Serial.println(targetPos.z);

  if(isnan(dist)) return;

  if(dist <= reachRange) {
    
    isMoving  = false;

    Serial.print  (F("Arrived at :   X "));
    Serial.print  ( targetPos.x  );
    Serial.print  (F("   Y "));
    Serial.print  ( targetPos.y  );
    Serial.print  (F("   Z "));
    Serial.println( targetPos.z  );
    
    if(isProgMode) runProgram();
    
    return;
  }

  // Normalize direction
  float invDist = 1.0 / dist;
  float ux = dx * invDist;
  float uy = dy * invDist;
  float uz = dz * invDist;

  // Move by step size
  float stepSize = moveSpeed * (stepInterval / 1000.0); // mm per frame

  currentPos.x += ux * stepSize;
  currentPos.y += uy * stepSize;
  currentPos.z += uz * stepSize;

  moveServosTo(currentPos);
}



