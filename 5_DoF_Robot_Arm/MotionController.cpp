
#include <Servo.h>
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"
#include "headers/InverseKinematics.h"
#include "headers/ForwardKinematics.h"


String       home_Gcode = "G1 X0 Y100 Z200";
CartesianPos homeCoords = { 0, 100, 200, 1 };
CartesianPos currentPos;
CartesianPos targetPos;


// ====================================================
// Robot dimensions (mm)
// ====================================================
const Rig rig = {
  100,  // c - Base height
  200,  // l - Arm lengths (both the same size)
  37,   // g - Y tool's offset
  80,   // f - Z tool's offset

  4,    // ofst_base   offset in degrees
  28,   // ofst_sho    offset in degrees
  9,    // ofst_elb    offset in degrees
  9,    // ofst_wri    offset in degrees
  0,    // ofst_wriRol offset in degrees

  6,    // sho_R_ofst      mirror offset in degrees
  6,    // elb_R_ofst      mirror offset in degrees
  0,    // wri_R_ofst      mirror offset in degrees
};

const Pot basePot     = { A0, 201, 920, 24, 160 };
const Pot shoulderPot = { A1, 902, 188, 19, 180 };
const Pot elbowPot    = { A2, 905, 225, 20, 180 };
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
const float moveSpeed    = 60.0; // mm/s (smaller more precise)
const float stepInterval = 6.5;  // ms   (ms between steps > smaller more precise)


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

int start_wristRoll  = 90;



// ***********************************
void ManuTestServo(char* cmd_Buff) {
  
  // CMD examples :
  // base:90
  // shoulder:135
  // elbow:87
  
  char* colon = strchr(cmd_Buff, ':');
  
  if(colon == nullptr) return;

  *colon = '\0'; // Split the string into two parts

  char* axisName = cmd_Buff;
  int angle      = atoi(colon + 1);

  if(strcmp(axisName, "base") == 0) {
    int safe_base    = limitRange(angle  +rig.ofst_base, basePot    );
    servo_base       .write(safe_base);
  }
  
  else if(strcmp(axisName, "shoulder") == 0) {
    int safe_shoulder = limitRange(angle,  shoulderPot);
    servo_shoulder_L .write(     safe_shoulder);
    servo_shoulder_R .write(180 -safe_shoulder +rig.mir_sho_ofst    );
  }
  
  else if(strcmp(axisName, "elbow") == 0) {
    int safe_elbow    = limitRange(angle,   elbowPot  );
    servo_elbow_L    .write(180 -safe_elbow    +rig.mir_elb_ofst    );
    servo_elbow_R    .write(     safe_elbow);
  }
  
  else if(strcmp(axisName, "wrist") == 0) {
    int safe_wrist    = limitRange(angle,   wristPot  );
    servo_wrist_L    .write(     safe_wrist);
    servo_wrist_R    .write(180 -safe_wrist    +rig.mir_wri_ofst    );
  }

  else if(strcmp(axisName, "wristRoll") == 0) {
    servo_wrist_roll .write(angle);
  }

  delay(200);
}

// ***********************************



// ====================================================
// Setup
// ====================================================
void init_Motion() {
 
  delay(200);

  int base_potAngle     = readAngle(basePot);
  int shoulder_potAngle = readAngle(shoulderPot);
  int elbow_potAngle    = readAngle(elbowPot);
  int wrist_potAngle    = readAngle(wristPot);

  servo_base       .attach(3);
  servo_shoulder_L .attach(4);
  servo_shoulder_R .attach(5);
  servo_elbow_L    .attach(6);
  servo_elbow_R    .attach(7);
  servo_wrist_L    .attach(8);
  servo_wrist_R    .attach(9);
  servo_wrist_roll .attach(10);

  servo_base       .write(     base_potAngle                      );
  servo_shoulder_L .write(     shoulder_potAngle                  );
  servo_shoulder_R .write(180 -shoulder_potAngle +rig.mir_sho_ofst);
  
  servo_elbow_L    .write(     elbow_potAngle                     );
  servo_elbow_R    .write(180 -elbow_potAngle    +rig.mir_elb_ofst);
  
  servo_wrist_L    .write(     wrist_potAngle                     );
  servo_wrist_R    .write(180 -wrist_potAngle    +rig.mir_wri_ofst);
  
  servo_wrist_roll .write(     start_wristRoll                    );

  currentPos = forwardKinematics(
    base_potAngle,
    shoulder_potAngle   -rig.ofst_sho,
    180 -elbow_potAngle +rig.ofst_elb
  );

  Serial.print  (F("RES:CONNECTED > at : X"));
  Serial.print  ( floor(currentPos.x) );
  Serial.print  (F(" Y"));
  Serial.print  ( floor(currentPos.y) );
  Serial.print  (F(" Z"));
  Serial.println( floor(currentPos.z) );
}


// ====================================================
// Vars Methods
// ====================================================
bool isNewValue(int &prevAngle, int newAngle) {

  if(prevAngle == newAngle) return false;

  prevAngle = newAngle;
  return true;
}

int  readAngle (const Pot& pot) {

  return map( analogRead(pot.pin), pot.min, pot.max, pot.minRange, pot.maxRange );
}

int  limitRange(int angle, const Pot& pot) {

  return constrain(angle, pot.minRange, pot.maxRange);
}


// ====================================================
// Methods
// ====================================================
void setTargetTo(CartesianPos coords) {

  targetPos    = coords;
  isMoving     = true;
  lastStepTime = millis();  // reset step timer
}

void moveServosTo(CartesianPos coords) {

  JointAngles angles = inverseKinematics(coords.x, coords.y, coords.z);

  // Safe limit
  if(  isnan(angles.tau    )
    || isnan(angles.gamma  )
    || isnan(angles.lambda )
    || isnan(angles.epsilon)) {
    return;
  }

  int base_angle     = (int)angles.epsilon + rig.ofst_base;
  int shoulder_angle = (int)angles.tau     + rig.ofst_sho ;
  int elbow_angle    = (int)angles.gamma   - rig.ofst_elb ;
  int wrist_angle    = (int)angles.lambda  + rig.ofst_wri ;

  int safe_base      = limitRange(base_angle,     basePot       );
  int safe_shoulder  = limitRange(shoulder_angle, shoulderPot   );
  int safe_elbow     = limitRange(elbow_angle,    elbowPot      );
  int safe_wrist     = limitRange(wrist_angle,    wristPot      );
  
  if(isNewValue(prev_base, safe_base)) {
    servo_base       .write(     safe_base                      );
  }

  if(isNewValue(prev_shoulder, safe_shoulder)) {
    servo_shoulder_L .write(     safe_shoulder                  );
    servo_shoulder_R .write(180 -safe_shoulder +rig.mir_sho_ofst);
  }

  if(isNewValue(prev_elbow, safe_elbow)) {
    servo_elbow_L    .write(180 -safe_elbow    +rig.mir_elb_ofst);
    servo_elbow_R    .write(     safe_elbow                     );
  }

  if(isNewValue(prev_wrist, safe_wrist)) {
    servo_wrist_L    .write(180 -safe_wrist    +rig.mir_wri_ofst);
    servo_wrist_R    .write(     safe_wrist                     );
  }
}

void linear_Interpolation() {

  unsigned long now = millis();

  if(!isMoving || now - lastStepTime < stepInterval) return;

  lastStepTime = now;

  float dx = targetPos.x - currentPos.x;
  float dy = targetPos.y - currentPos.y;
  float dz = targetPos.z - currentPos.z;

  float dist = sqrt(dx*dx + dy*dy + dz*dz);

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
  float invDist = 1.0 /dist;
  float ux = dx *invDist;
  float uy = dy *invDist;
  float uz = dz *invDist;

  // Move by step size
  float stepSize = moveSpeed * (stepInterval /1000.0);  // mm per frame

  currentPos.x += ux *stepSize;
  currentPos.y += uy *stepSize;
  currentPos.z += uz *stepSize;

  moveServosTo(currentPos);
}
