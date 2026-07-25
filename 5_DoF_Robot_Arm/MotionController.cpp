
#include <Servo.h>
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"
#include "headers/InverseKinematics.h"


String home_Gcode = "G1 X0 Y0 Z150";
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

  0,  // ofst_base       offset in degrees
  0,  // ofst_shoulder   offset in degrees
  0,  // ofst_elbow      offset in degrees
  0,  // ofst_wrist      offset in degrees
  0,  // ofst_wrist_roll offset in degrees

  4,  // sho_R_ofst      mirror offset in degrees
  6,  // elb_R_ofst      mirror offset in degrees
  0,  // wri_R_ofst      mirror offset in degrees
};

const Pot basePot     = { A0, 201, 920, 30, 155 };
const Pot shoulderPot = { A1, 902, 188,  0, 180 };
const Pot elbowPot    = { A2, 905, 225,  0, 180 };
const Pot wristPot    = { A3, 925, 189,  0, 180 };


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
const float moveSpeed    = 50.0; // mm/s (smaller more precise)
const float stepInterval = 5.0;  // ms   (ms between steps > smaller more precise)


// ====================================================
// Vars
// ====================================================
unsigned long lastStepTime = 0;

bool isHome   = false;
bool isMoving = false;

// Previous position in Degrees
int prev_base        = -1;
int prev_shoulder    = -1;
int prev_elbow       = -1;
int prev_wrist       = -1;
int prev_wrist_roll  = -1;

int start_wrist_roll = 90;



// ***********************************
String lastCMD = "";

void TestServo(String cmd) {
  
  if(cmd == lastCMD) return;
  lastCMD = cmd;

  int angle = cmd.toInt();
  if(isnan(angle)) return;

  Serial.print(F("CMD: "));
  Serial.println(cmd);

  Serial.print(F("Old potValue: "));
  Serial.println( analogRead(basePot.pin) );        // <<<<<<<<<<

  delay(100);

  servo_base .write(angle);                       // <<<<<<<<<<
  delay(1500);

  Serial.print(F("New potValue: "));
  Serial.println( analogRead(basePot.pin) );        // <<<<<<<<<<
  Serial.println(F("*****************"));
}

// ***********************************



// ====================================================
// Setup
// ====================================================
void init_Motion() {
 
  delay(200);

  int base_angle     = readAngle(basePot);
  int shoulder_angle = readAngle(shoulderPot);
  int elbow_angle    = readAngle(elbowPot);
  int wrist_angle    = readAngle(wristPot);

  servo_base       .attach(3);
  servo_shoulder_L .attach(4);
  servo_shoulder_R .attach(5);
  servo_elbow_L    .attach(6);
  servo_elbow_R    .attach(7);
  servo_wrist_L    .attach(8);
  servo_wrist_R    .attach(9);
  servo_wrist_roll .attach(10);

  servo_base       .write(     base_angle);
  servo_shoulder_L .write(     shoulder_angle);
  servo_shoulder_R .write(180 -shoulder_angle +rig.sho_R_ofst);
  servo_elbow_L    .write(     elbow_angle);
  servo_elbow_R    .write(180 -elbow_angle    +rig.elb_R_ofst);
  servo_wrist_L    .write(     wrist_angle);
  servo_wrist_R    .write(180 -wrist_angle    +rig.wri_R_ofst);
  servo_wrist_roll .write(     start_wrist_roll);

  Serial.println(F("RES:CONNECTED"));
}


// ====================================================
// Methods
// ====================================================
int readAngle(const Pot& pot) {

  return map( analogRead(pot.pin), pot.min, pot.max, pot.minRange, pot.maxRange );
}

void setTargetTo(CartesianPos coords) {

  targetPos    = coords;
  isMoving     = true;
  lastStepTime = millis();  // reset step timer
}

bool isNewValue(int &prevAngle, int newAngle) {

  if (prevAngle == newAngle) return false;

  prevAngle = newAngle;
  return true;
}

void moveServosTo(CartesianPos coords) {

  JointAngles angles = inverseKinematics(coords.x, coords.y, coords.z);

  // Safe limit
  if (isnan(angles.tau)
      || isnan(angles.gamma)
      || isnan(angles.lambda)
      || isnan(angles.epsilon)) {
    return;
  }

  int ang_base     = rig.ofst_base + (int)angles.epsilon;
  int min_base     = rig.ofst_base;
  int max_base     = 180;

  int ang_shoulder = rig.ofst_shoulder + (int)angles.tau;
  int min_shoulder = rig.ofst_shoulder;
  int max_shoulder = 180;

  int ang_elbow    = (int)angles.gamma - rig.ofst_elbow;
  int min_elbow    = 0;
  int max_elbow    = 180 - rig.ofst_elbow;

  int ang_wrist    = (int)angles.lambda - rig.ofst_wrist;
  int min_wrist    = 0;
  int max_wrist    = 180;


  // Joint angles bounderies
  int safe_base     = constrain(ang_base,     min_base,     max_base    );
  int safe_shoulder = constrain(ang_shoulder, min_shoulder, max_shoulder);
  int safe_elbow    = constrain(ang_elbow,    min_elbow,    max_elbow   );
  int safe_wrist    = constrain(ang_wrist,    min_wrist,    max_wrist   );


  if(isNewValue(prev_base, safe_base)) {
    servo_base.write(safe_base);
  }

  if(isNewValue(prev_shoulder, safe_shoulder)) {
    servo_shoulder_L.write(safe_shoulder);
    servo_shoulder_R.write(180 - safe_shoulder + rig.sho_R_ofst);
  }

  if(isNewValue(prev_elbow, safe_elbow)) {
    servo_elbow_L.write(safe_elbow);
    servo_elbow_R.write(180 - safe_elbow + rig.elb_R_ofst);
  }

  if(isNewValue(prev_wrist, safe_wrist)) {
    servo_wrist_L.write(safe_wrist);
    servo_wrist_R.write(180 - safe_wrist + rig.wri_R_ofst);
  }
}

void linear_Interpolation() {

  unsigned long now = millis();

  if(!isMoving || now - lastStepTime < stepInterval) return;

  lastStepTime = now;

  float dx = targetPos.x - currentPos.x + offsetPos[0];
  float dy = targetPos.y - currentPos.y + offsetPos[1];
  float dz = targetPos.z - currentPos.z + offsetPos[2];

  float dist = sqrt(dx * dx + dy * dy + dz * dz);

  // Serial.print("Dist : ");   Serial.println(dist);
  // Serial.print("Targ X : "); Serial.println(targetPos.x);
  // Serial.print("Targ Y : "); Serial.println(targetPos.y);
  // Serial.print("Targ Z : "); Serial.println(targetPos.z);

  if(isnan(dist)) return;

  if(dist <= reachRange) {

    isMoving = false;

    Serial.println(F("RES:ARRIVED"));

    // Serial.print  (F("Arrived at :   X "));
    // Serial.print  ( targetPos.x  );
    // Serial.print  (F("   Y "));
    // Serial.print  ( targetPos.y  );
    // Serial.print  (F("   Z "));
    // Serial.println( targetPos.z  );

    return;
  }

  // Normalize direction
  float invDist = 1.0 / dist;
  float ux = dx * invDist;
  float uy = dy * invDist;
  float uz = dz * invDist;

  // Move by step size
  float stepSize = moveSpeed * (stepInterval / 1000.0);  // mm per frame

  currentPos.x += ux * stepSize;
  currentPos.y += uy * stepSize;
  currentPos.z += uz * stepSize;

  moveServosTo(currentPos);
}
