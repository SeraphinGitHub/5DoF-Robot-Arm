
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

  4,    // ofst_base    offset in degrees
  10,   // ofst_MG995   offset in degrees
  24,   // ofst_sho     offset in degrees
  -3,   // ofst_elb     offset in degrees
  -7,   // ofst_wri     offset in degrees
  0,    // ofst_wriRol  offset in degrees

  6,    // mir_sho_ofst mirror offset in degrees
  6,    // mir_elb_ofst mirror offset in degrees
  0,    // mir_wri_ofst mirror offset in degrees
};


// **********************************
// base potAngle:
//   0° >  31
//  45° > 285 ==> mesured
//  90° > 540
// 135° > 794 ==> mesured
// 180° > 1048
// **********************************
// shoulder potAngle:
//   0° > 798
//  45° > 628 ==> mesured > 63°
//  90° > 458
// 135° > 287 ==> mesured > 141°
// 180° > 117
// **********************************
// elbow potAngle:
//   0° > 211
//  45° > 372 ==> mesured > 63°
//  90° > 534
// 135° > 695 ==> mesured > 141°
// 180° > 856
// **********************************
// wrist potAngle:
//   0° > 940
//  45° > 766 ==> mesured > 45°
//  90° > 592
// 135° > 418 ==> mesured > 135°
// 180° > 244
// **********************************

const Pot basePot     = { A0,  0, 1023, 24, 160 };
// const Pot basePot     = { A0, 31, 1048, 24, 160 };
const Pot shoulderPot = { A1, 798, 117, 24, 180 };
const Pot elbowPot    = { A2, 211, 856, 24, 180 };
const Pot wristPot    = { A3, 940, 244,  0, 180 };


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


// ====================================================
// Setup
// ====================================================
void initController() {
 
  delay(200);

  int base_potAngle     = readAngle( basePot     );
  int shoulder_potAngle = readAngle( shoulderPot );
  int elbow_potAngle    = readAngle( elbowPot    );
  int wrist_potAngle    = readAngle( wristPot    );
  
  servo_base       .attach(3);
  servo_shoulder_L .attach(4);
  servo_shoulder_R .attach(5);
  servo_elbow_L    .attach(6);
  servo_elbow_R    .attach(7);
  servo_wrist_L    .attach(8);
  servo_wrist_R    .attach(9);
  servo_wrist_roll .attach(10);

  servo_base       .write(     base_potAngle     -rig.ofst_base                   );

  servo_shoulder_L .write(     shoulder_potAngle +rig.ofst_MG995                  );
  servo_shoulder_R .write(180 -shoulder_potAngle -rig.ofst_MG995 +rig.mir_sho_ofst);
  
  servo_elbow_L    .write(180 -elbow_potAngle    +rig.ofst_MG995 +rig.mir_elb_ofst);
  servo_elbow_R    .write(     elbow_potAngle    -rig.ofst_MG995                  );
  
  servo_wrist_L    .write(     wrist_potAngle    -rig.ofst_wri                    );
  servo_wrist_R    .write(180 -wrist_potAngle    +rig.ofst_wri   +rig.mir_wri_ofst);
  
  servo_wrist_roll .write(     start_wristRoll                                    );

  delay(1500);

  base_potAngle     = readAngle( basePot     );
  shoulder_potAngle = readAngle( shoulderPot );
  elbow_potAngle    = readAngle( elbowPot    );
  wrist_potAngle    = readAngle( wristPot    );

  // Serial.print  (F("base : "));
  // Serial.println( base_potAngle );
  // Serial.print  (F("shoulder : "));
  // Serial.println( shoulder_potAngle );
  // Serial.print  (F("elbow : "));
  // Serial.println( elbow_potAngle );
  // Serial.print  (F("wrist : "));
  // Serial.println( wrist_potAngle );

  currentPos = forwardKinematics(
    base_potAngle,
    shoulder_potAngle,
    elbow_potAngle,
    wrist_potAngle
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
void manuAngleMove(char* cmd_Buff) {
  
  // CMD examples :
  // base:90
  // shoulder:135
  // elbow:87
  
  char* colon = strchr(cmd_Buff, ':');
  
  if(colon == nullptr) return;

  *colon = '\0'; // Split the string into two parts

  char* axisName = cmd_Buff;
  int angle      = atoi(colon + 1);


  // =======================================================
  // Base
  // =======================================================
  if(strcmp(axisName, "base") == 0) {
    int safe_base = limitRange(angle +rig.ofst_base, basePot);

    servo_base.write(safe_base);
    
    delay(1500);
    Serial.print  (F("base potAngle : "));
    Serial.println( readAngle(basePot)  );
  }


  // =======================================================
  // Shoulder
  // =======================================================
  else if(strcmp(axisName, "shoulder") == 0) {
    int safe_shoulder = limitRange(angle, shoulderPot);

    servo_shoulder_L.write(     safe_shoulder +rig.ofst_sho);
    servo_shoulder_R.write(180 -safe_shoulder -rig.ofst_sho +rig.mir_sho_ofst);
        
    delay(1500);
    Serial.print  (F("shoulder potAngle : "));
    Serial.println( readAngle(shoulderPot)  );
  }

  
  // =======================================================
  // Elbow
  // =======================================================
  else if(strcmp(axisName, "elbow") == 0) {
    int safe_elbow = limitRange(angle, elbowPot);

    servo_elbow_L.write(180 -safe_elbow -rig.ofst_elb +rig.mir_elb_ofst);
    servo_elbow_R.write(     safe_elbow +rig.ofst_elb);
            
    delay(1500);
    Serial.print  (F("elbow potAngle : "));
    Serial.println( readAngle(elbowPot)  );
  }

  
  // =======================================================
  // Wrist
  // =======================================================
  else if(strcmp(axisName, "wrist") == 0) {
    int safe_wrist = limitRange(angle, wristPot);

    servo_wrist_L.write(     safe_wrist +rig.ofst_wri);
    servo_wrist_R.write(180 -safe_wrist -rig.ofst_wri +rig.mir_wri_ofst);
                
    delay(1500);
    Serial.print  (F("wrist potAngle : "));
    Serial.println( readAngle(wristPot)  );
  }


  // =======================================================
  // WristRoll
  // =======================================================
  else if(strcmp(axisName, "wristRoll") == 0) {
    
    servo_wrist_roll.write(angle);
  }
}

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

  int base_angle     = (int)angles.epsilon ;
  int shoulder_angle = (int)angles.tau      ;
  int elbow_angle    = (int)angles.gamma   ;
  int wrist_angle    = (int)angles.lambda  ;

  int safe_base      = limitRange(base_angle,     basePot       );
  int safe_shoulder  = limitRange(shoulder_angle, shoulderPot   );
  int safe_elbow     = limitRange(elbow_angle,    elbowPot      );
  int safe_wrist     = limitRange(wrist_angle,    wristPot      );
  
  if(isNewValue(prev_base, safe_base)) {
    servo_base       .write(     safe_base    + rig.ofst_base                  );
  }

  if(isNewValue(prev_shoulder, safe_shoulder)) {
    servo_shoulder_L .write(     safe_shoulder  + rig.ofst_sho                );
    servo_shoulder_R .write(180 -safe_shoulder  -rig.ofst_sho +rig.mir_sho_ofst);
  }

  if(isNewValue(prev_elbow, safe_elbow)) {
    servo_elbow_L    .write(180 -safe_elbow    - rig.ofst_elb  +rig.mir_elb_ofst);
    servo_elbow_R    .write(     safe_elbow    + rig.ofst_elb                  );
  }

  if(isNewValue(prev_wrist, safe_wrist)) {
    servo_wrist_L    .write(     safe_wrist     + rig.ofst_wri                );
    servo_wrist_R    .write(180 -safe_wrist     - rig.ofst_wri +rig.mir_wri_ofst);
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
