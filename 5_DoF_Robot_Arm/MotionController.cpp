
#include <Servo.h>
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"
#include "headers/InverseKinematics.h"
#include "headers/ForwardKinematics.h"

Servo servo_base;
Servo servo_shoulder_L;
Servo servo_shoulder_R;
Servo servo_elbow_L;
Servo servo_elbow_R;
Servo servo_wrist_L;
Servo servo_wrist_R;
Servo servo_wrist_roll;

// ====================================================
// Robot dimensions (mm)
// ====================================================
const Rig rig = {
  100,  // c - Base height
  200,  // l - Arm lengths (both the same size)
  37,   // g - Y tool's offset
  80,   // f - Z tool's offset

  4,    // ofst_base    offset in degrees
  24,   // ofst_sho     offset in degrees
  -3,   // ofst_elb     offset in degrees
  -7,   // ofst_wri     offset in degrees
  0,    // ofst_wriRol  offset in degrees

  6,    // mir_sho_ofst mirror offset in degrees
  6,    // mir_elb_ofst mirror offset in degrees
  0,    // mir_wri_ofst mirror offset in degrees
};

const Pot basePot     = { A0,  0, 1023, 24, 160 };
const Pot shoulderPot = { A1, 798, 117,  0, 180 };
const Pot elbowPot    = { A2, 211, 856,  0, 180 };
const Pot wristPot    = { A3, 940, 244,  0, 180 };

// ====================================================
// Vars
// ====================================================
CartesianPos homeCoords = { 0, 100, 200, 1 };
CartesianPos currentPos;
CartesianPos targetPos;

unsigned long lastStepTime = 0;

const float reachRange   =    1;
const float stepInterval =  6.0;  // ms   (ms between steps > smaller more precise)
const float maxSpeed     = 60.0;  // mm/s (smaller more precise)
const float accelDist    = 20.0;  // mm
const float decelDist    = 15.0;  // mm
float       currentSpeed =  0.0;

const float acceleration = (maxSpeed *maxSpeed) / (2.0f *accelDist);
const float deceleration = (maxSpeed *maxSpeed) / (2.0f *decelDist);

bool isHome     = false;
bool isMoving   = false;
bool isPrevInit = false;

int prev_base        = -1;
int prev_shoulder    = -1;
int prev_elbow       = -1;
int prev_wrist       = -1;
int prev_wrist_roll  = -1;

int wrist_roll_angle = 90;


CalibrationTable base_table[] = {
  {   0,  46 },
  {  45,  74 },
  {  90,  96 },
  { 135, 122 },
  { 180, 147 }
};

CalibrationTable elbow_table[] = {
  {   5,   2 },
  {  10,   5 },
  {  15,  10 },
  {  20,  17 },
  {  25,  23 },
  {  30,  28 },
  {  35,  33 },
  {  40,  39 },
  {  45,  44 },
  {  50,  50 },
  {  55,  55 },
  {  60,  60 },
  {  65,  65 },
  {  70,  70 },
  {  75,  75 },
  {  80,  80 },
  {  85,  85 },
  {  90,  90 },
  {  95,  95 },
  { 100, 100 },
  { 105, 105 },
  { 110, 110 },
  { 115, 115 },
  { 120, 120 },
  { 125, 125 },
  { 130, 130 },
  { 135, 135 },
  { 140, 141 },
  { 145, 147 },
  { 150, 152 },
  { 155, 158 },
  { 160, 164 },
  { 165, 171 },
  { 170, 178 },
  { 175, 184 },
  { 180, 190 }
};

CalibrationTable shoulder_table[] = {
  { 160, 162 },
  { 155, 159 },
  { 150, 153 },
  { 145, 147 },
  { 140, 142 },
  { 135, 136 },
  { 130, 130 },
  { 125, 125 },
  { 120, 119 },
  { 115, 114 },
  { 110, 108 },
  { 105, 103 },
  { 100,  98 },
  {  95,  93 },
  {  90,  88 },
  {  85,  82 },
  {  80,  77 },
  {  75,  71 },
  {  70,  68 },
  {  65,  64 },
  {  60,  59 },
  {  55,  54 },
  {  50,  50 },
  {  45,  45 },
  {  40,  40 },
  {  35,  35 },
  {  30,  30 },
  {  25,  24 },
  {  20,  19 },
  {  15,  13 },
  {  10,   7 },
  {   5,   1 },
  {   0,  -3 }
};


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

int servo_correction_IK(float IK_angle, CalibrationTable servoTable[]) {
  int tableSize = sizeof(servoTable) / sizeof(servoTable[0]);

  for(int i = 0; i < tableSize -1; i++) {

    float real_down = servoTable[i   ].real;
    float real_up   = servoTable[i +1].real;

    float meas_down = servoTable[i   ].measured;
    float meas_up   = servoTable[i +1].measured;

    if(IK_angle >= real_down
    && IK_angle <= real_up) {

      // Interpolate measured angle [ex: IK_angle = 163°]
      float lerp_measure = meas_down + (IK_angle -real_down) * (meas_up -meas_down) / (real_up -real_down);
      //                        164  +      (163 -160)       *    (171 -164)        /     (165 -160)
      //                      = 168.2

      // Correction
      float error = lerp_measure -IK_angle;
      // 168.2 -163 = 5.2
      // 163 -(168.2 -163) = 5.2

      return round( IK_angle -error );
      // 163 -5.2 = 157.8 ==> 158°
    }
  }

  return IK_angle; // outside calibration range
}

int servo_correction_FK(int potAngle, CalibrationTable servoTable[]) {
  int tableSize = sizeof(servoTable) / sizeof(servoTable[0]);

  for(int i = 0; i < tableSize -1; i++) {

    int real_down = servoTable[i   ].real;
    int real_up   = servoTable[i +1].real;

    int meas_down = servoTable[i   ].measured;
    int meas_up   = servoTable[i +1].measured;

    if(potAngle >= meas_down
    && potAngle <= meas_up) {

      // [ex: potAngle = 167°]
      float ratio = (potAngle -meas_down) / (meas_up -meas_down);
      //   0.43 =      (167 -164)       /     (171 -164)

      int correctedAngle = round( real_down + ratio * (real_up -real_down) );
      //           160 +  0.43 *     (165 -160)

      return correctedAngle;
    }
  }

  return potAngle;
}


// ====================================================
// Setup
// ====================================================
void initController() {
 
  delay(200);

  isPrevInit = false;

  servo_base       .attach(3);
  servo_shoulder_L .attach(4);
  servo_shoulder_R .attach(5);
  servo_elbow_L    .attach(6);
  servo_elbow_R    .attach(7);
  servo_wrist_L    .attach(8);
  servo_wrist_R    .attach(9);
  servo_wrist_roll .attach(10);

  int baseFKAngle       = servo_correction_FK(readAngle( basePot     ), base_table    );
  int shoulder_potAngle = servo_correction_FK(readAngle( shoulderPot ), shoulder_table);
  int elbow_potAngle    = servo_correction_FK(readAngle( elbowPot    ), elbow_table   );
  int wrist_potAngle    = readAngle( wristPot );

  int baseServoAngle    = round( (baseFKAngle -basePot.minRange) * 180.0f / (float)(basePot.maxRange -basePot.minRange) ); // for 270° servo

  // ****************************************************************************
  servo_base       .write(     baseServoAngle                                    );
  // ****************************************************************************
  servo_shoulder_L .write(     shoulder_potAngle +rig.ofst_sho                  );
  servo_shoulder_R .write(180 -shoulder_potAngle -rig.ofst_sho +rig.mir_sho_ofst);
  // ****************************************************************************
  servo_elbow_L    .write(180 -elbow_potAngle    +rig.ofst_elb +rig.mir_elb_ofst);
  servo_elbow_R    .write(     elbow_potAngle    -rig.ofst_elb                  );
  // ****************************************************************************
  servo_wrist_L    .write(     wrist_potAngle    +rig.ofst_wri                  );
  servo_wrist_R    .write(180 -wrist_potAngle    -rig.ofst_wri +rig.mir_wri_ofst);
  // ****************************************************************************
  servo_wrist_roll .write(     wrist_roll_angle                                 );
  // ****************************************************************************

  delay(1000);

  baseFKAngle       = servo_correction_FK(readAngle( basePot     ), base_table    );
  shoulder_potAngle = servo_correction_FK(readAngle( shoulderPot ), shoulder_table);
  elbow_potAngle    = servo_correction_FK(readAngle( elbowPot    ), elbow_table   );
  wrist_potAngle    = readAngle( wristPot );

  int dirMultiplier = baseFKAngle < 95 ? -1 : 1;

  currentPos = forwardKinematics(
    baseFKAngle + basePot.minRange *dirMultiplier,
    shoulder_potAngle,
    elbow_potAngle,
    wrist_potAngle
  );

  targetPos = currentPos;

  setTargetTo(homeCoords);
  
  Serial.print  (F("RES:CONNECTED > at : X"));
  Serial.print  ( floor(currentPos.x) );
  Serial.print  (F(" Y"));
  Serial.print  ( floor(currentPos.y) );
  Serial.print  (F(" Z"));
  Serial.println( floor(currentPos.z) );
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
  int   angle    = atoi(colon + 1);


  // =======================================================
  // Base
  // =======================================================
  if(strcmp(axisName, "base") == 0) {
    int safe_base = limitRange(angle +rig.ofst_base, basePot);

    servo_base.write(safe_base);
    
    delay(1500);
    Serial.print  (F("base potAngle : "));
    Serial.print  (angle);
    Serial.print  (F(" > "));
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
    Serial.print  (angle);
    Serial.print  (F(" > "));
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
    Serial.print  (angle);
    Serial.print  (F(" > "));
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
    Serial.print  (angle);
    Serial.print  (F(" > "));
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
  
  // Serial.println("*********************");
  // Serial.print("x: ");
  // Serial.print(coords.x);
  // Serial.print(", y:");
  // Serial.print(coords.y);
  // Serial.print(", z:");
  // Serial.println(coords.z);
  
  JointAngles angles = inverseKinematics(coords.x, coords.y, coords.z);

  // Safe limit
  if(  isnan(angles.tau    )
    || isnan(angles.gamma  )
    || isnan(angles.lambda )
    || isnan(angles.epsilon)) {
    return;
  }

  int base_angle     = round( basePot.minRange + (angles.epsilon * (float)(basePot.maxRange -basePot.minRange) / 180.0f) ); // for 270° servo
  int shoulder_angle = servo_correction_IK((int)angles.tau,   shoulder_table);
  int elbow_angle    = servo_correction_IK((int)angles.gamma, elbow_table   );
  int wrist_angle    = (int)angles.lambda;

  int safe_base      = limitRange(base_angle,     basePot    );
  int safe_shoulder  = limitRange(shoulder_angle, shoulderPot);
  int safe_elbow     = limitRange(elbow_angle,    elbowPot   );
  int safe_wrist     = limitRange(wrist_angle,    wristPot   );

  // Initialize prev values (Only once)
  if(!isPrevInit) {
    prev_base       = safe_base;
    prev_shoulder   = safe_shoulder;
    prev_elbow      = safe_elbow;
    prev_wrist      = safe_wrist;
    prev_wrist_roll = wrist_roll_angle;
    
    isPrevInit = true;
  }
  
  // ****************************************************************************
  if(isNewValue(prev_base, safe_base)) {
    servo_base       .write(     safe_base                                       );
  } // **************************************************************************
  if(isNewValue(prev_shoulder, safe_shoulder)) {
    servo_shoulder_L .write(     safe_shoulder  + rig.ofst_sho                   );
    servo_shoulder_R .write(180 -safe_shoulder  - rig.ofst_sho +rig.mir_sho_ofst );
  }// ***************************************************************************
  if(isNewValue(prev_elbow, safe_elbow)) {
    servo_elbow_L    .write(180 -safe_elbow     - rig.ofst_elb  +rig.mir_elb_ofst);
    servo_elbow_R    .write(     safe_elbow     + rig.ofst_elb                   );
  }// ***************************************************************************
  if(isNewValue(prev_wrist, safe_wrist)) {
    servo_wrist_L    .write(     safe_wrist     + rig.ofst_wri                   );
    servo_wrist_R    .write(180 -safe_wrist     - rig.ofst_wri +rig.mir_wri_ofst );
  }// ***************************************************************************
  if(isNewValue(prev_wrist_roll, wrist_roll_angle)) {
    servo_wrist_roll .write(     wrist_roll_angle                                );
  } // **************************************************************************
}


void linear_Interpolation() {

  unsigned long now = millis();

  if(!isMoving || now -lastStepTime < stepInterval) return;

  float deltaTime = (now -lastStepTime) /1000.0f;
  lastStepTime    = now;

  float distX = targetPos.x -currentPos.x;
  float distY = targetPos.y -currentPos.y;
  float distZ = targetPos.z -currentPos.z;

  float distance_3D = sqrt(distX*distX + distY*distY + distZ*distZ);

  // Serial.print("Dist : ");   Serial.println(dist);
  // Serial.print("Targ X : "); Serial.println(targetPos.x);
  // Serial.print("Targ Y : "); Serial.println(targetPos.y);
  // Serial.print("Targ Z : "); Serial.println(targetPos.z);

  if(isnan(distance_3D)) return;


  // ==============================================
  // Arrived at targetPos
  // ==============================================
  if(distance_3D <= reachRange) {

    currentSpeed = 0.0f;
    isMoving     = false;

    Serial.println(F("RES:ARRIVED"));

    Serial.print  (F("Arrived at :   X "));
    Serial.print  ( targetPos.x  );
    Serial.print  (F("   Y "));
    Serial.print  ( targetPos.y  );
    Serial.print  (F("   Z "));
    Serial.println( targetPos.z  );

    return;
  }


  // ==============================================
  // Acceleration / Deceleration
  // ==============================================
  float stoppingDist = (currentSpeed *currentSpeed) / (2.0f *deceleration);

  // Decelerate
  if(distance_3D <= stoppingDist) {
    currentSpeed -= deceleration *deltaTime;
    if(currentSpeed < 0.0f) currentSpeed = 0.0f;
  }
  
  // Accelerate
  else {
    currentSpeed += acceleration *deltaTime;
    if(currentSpeed > maxSpeed) currentSpeed = maxSpeed;
  }


  // ==============================================
  // Direction
  // ==============================================
  float invRemainDist = 1.0 /distance_3D;
  float ux = distX *invRemainDist;
  float uy = distY *invRemainDist;
  float uz = distZ *invRemainDist;


  // ==============================================
  // Step move
  // ==============================================
  float stepSize = currentSpeed *deltaTime;

  currentPos.x += ux *stepSize;
  currentPos.y += uy *stepSize;
  currentPos.z += uz *stepSize;


  moveServosTo(currentPos);
}
