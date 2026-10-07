
#include <Servo.h>
#include <Arduino.h>
#include "headers/Calibration.h"
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
  100, // c - Base height
  200, // l - Arm lengths (both the same size)
   37, // g - Y tool's offset
   80, // f - Z tool's offset

    4, // ofst_base    offset in degrees
   24, // ofst_sho     offset in degrees
    3, // ofst_elb     offset in degrees
   -7, // ofst_wri     offset in degrees
    0, // ofst_wriRol  offset in degrees

    6, // mir_sho_ofst mirror offset in degrees
    6, // mir_elb_ofst mirror offset in degrees
    0, // mir_wri_ofst mirror offset in degrees
};

// ====================================================
// Vars
// ====================================================
// Position  pos    = {   X,   Y,   Z,   P,   R, MT };
Position homePos    = {   0, 100, 250,   0,   0,  1 };
Position currentPos = { NAN, NAN, NAN, NAN, NAN, -1 };
Position targetPos  = { NAN, NAN, NAN, NAN, NAN, -1 };
Coords   offsetPos  = {   0,   0,   0  };

unsigned long lastStepTime = 0;

float stepInterval   =   2.0; // ms   (ms between steps > smaller more precise)
float currentSpeed   =   0.0; // mm/s
float maxSpeed       = 100.0; // mm/s (smaller more precise)
float accelDist      =  10.0; // mm
float decelDist      =  10.0; // mm

float acceleration   = (maxSpeed *maxSpeed) / (2.0f *accelDist);
float deceleration   = (maxSpeed *maxSpeed) / (2.0f *decelDist);

bool hasInit         = false;
bool isHome          = false;
bool isMoving        = false;
bool isPrevInit      = false;

int prev_Base        = -1;
int prev_Shoulder    = -1;
int prev_Elbow       = -1;
int prev_Wrist       = -1;
int prev_WristRoll   = -1;



// ****************************  Tempory Demo  ****************************
// String demoProg[] = {
//   "X100 Y60 Z215 A45 B45",
//   "X-100 Y150 Z150 A-45 B-30",
//   "X-150 Y60 Z300 A30 B0",
//   "X100 Y120 Z380 A-60 B20",
//   "X50 Y80 Z100 A-20",
//   "X-100 Y120 Z350 A-45 B-60"
// };

// String demoProg[] = {
//   "X100 Y60 Z215 P45 R45",
//   "X-40 Y57 Z215 P-20 R-15",
//   "X-110 Y70 Z225 P-40 R30",
//   "X-80 Y195 Z245 P0 R45",
//   "X100 P20",
//   "X100 Y60 Z215 P45 R45"
// };

String demoProg[] = {
  "X100 Y200 Z250",
  "Z200",
  "X0",
  "Y100",
  "X100",
  "Y200",
  "Z250"
};

int  demoSize    = sizeof(demoProg) / sizeof(demoProg[0]);
int  demoStep    = 0;
bool demoRunning = false;

void startDemo() {
  demoRunning = true;
  demoStep    = 0;
  setTargetTo(parseGcodeLine( demoProg[0] ));
}

void endDemo() {
  demoRunning = false;
  setTargetTo(homePos);
}

void updateDemo() {

  if(!demoRunning || isMoving) return;
  demoStep++;

  if(demoStep >= demoSize) demoStep = 0; // repeat forever
  setTargetTo(parseGcodeLine( demoProg[demoStep] ));
}
// ****************************  Tempory Demo  ****************************



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
void changeVarValue(char* cmd_Buff) {
  
  // CMD examples :
  // maxSpeed:60
  
  char* colon = strchr(cmd_Buff, ':');
  if(colon == nullptr) return;

  *colon = '\0'; // Split the string into two parts
  char* varName = cmd_Buff;
  float value   = atof(colon + 1);

  bool isNewSpeed = false;


  // ============================================
  // Step Interval (ms)
  // ============================================
  if(strcmp(varName, "stepInterval") == 0) {

    if(value >= 1.0 && value <= 20.0) {
      stepInterval = value;
      Serial.print(F("stepInterval:"));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }


  // ============================================
  // Max Speed (mm/s)
  // ============================================
  if(strcmp(varName, "maxSpeed")     == 0) {
    
    if(value >= 1.0 && value <= 200.0) {
      maxSpeed   = value;
      isNewSpeed = true;
      Serial.print(F("maxSpeed : "));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }


  // ============================================
  // Acceleration Distance (mm)
  // ============================================
  if(strcmp(varName, "accelDist")    == 0) {
    
    if(value >= 5.0 && value <= 100.0) {
      accelDist  = value;
      isNewSpeed = true;
      Serial.print(F("accelDist : "));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }


  // ============================================
  // Deceleration Distance (mm)
  // ============================================
  if(strcmp(varName, "decelDist")    == 0) {
    
    if(value >= 5.0 && value <= 100.0) {
      decelDist  = value;
      isNewSpeed = true;
      Serial.print(F("decelDist : "));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }


  // ============================================
  // OffsetPos coords (mm)
  // ============================================
  if(strcmp(varName, "offsetX")    == 0) {
    
    if(!isnan(value)) {
      offsetPos.x = value;
      Serial.print(F("offsetX : "));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }

  if(strcmp(varName, "offsetY")    == 0) {
    
    if(!isnan(value)) {
      offsetPos.y = value;
      Serial.print(F("offsetY : "));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }

  if(strcmp(varName, "offsetZ")    == 0) {
    
    if(!isnan(value)) {
      offsetPos.z = value;
      Serial.print(F("offsetZ : "));
      Serial.println(value);
    }
    else Serial.println(F("Unexpected value"));
  }


  // ============================================
  // Angle Speed (deg /s)
  // ============================================
  // if(strcmp(varName, "angleSpeed")   == 0) {
    
  //   if(value >= 1.0 && value <= 50.0) {
  //     angleSpeed = value;
  //     Serial.print(F("angleSpeed : "));
  //     Serial.println(value);
  //   }
  //   else Serial.println(F("Unexpected value"));
  // }

  if(isNewSpeed) {
    acceleration = (maxSpeed *maxSpeed) / (2.0f *accelDist);
    deceleration = (maxSpeed *maxSpeed) / (2.0f *decelDist);
  }
}

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
    int safe_base = limitRange(angle +rig.ofst_base, pot_Base);

    servo_base.write(safe_base);
    
    delay(1500);
    Serial.print  (F("base potAngle : "));
    Serial.print  (angle);
    Serial.print  (F(" > "));
    Serial.println( readAngle(pot_Base)  );
  }


  // =======================================================
  // Shoulder
  // =======================================================
  else if(strcmp(axisName, "shoulder") == 0) {
    int safe_shoulder = limitRange(angle, pot_Shoulder) +rig.ofst_sho;

    servo_shoulder_L.write(     safe_shoulder                  );
    servo_shoulder_R.write(180 -safe_shoulder +rig.mir_sho_ofst);
        
    delay(1500);
    Serial.print  (F("shoulder potAngle : "));
    Serial.print  (angle);
    Serial.print  (F(" > "));
    Serial.println( readAngle(pot_Shoulder)  );
  }

  
  // =======================================================
  // Elbow
  // =======================================================
  else if(strcmp(axisName, "elbow") == 0) {
    int safe_elbow = limitRange(angle, pot_Elbow) +rig.ofst_elb;

    servo_elbow_L.write(180 -safe_elbow +rig.mir_elb_ofst);
    servo_elbow_R.write(     safe_elbow                  );
            
    delay(1500);
    Serial.print  (F("elbow potAngle : "));
    Serial.print  (angle);
    Serial.print  (F(" > "));
    Serial.println( readAngle(pot_Elbow)  );
  }

  
  // =======================================================
  // Wrist
  // =======================================================
  else if(strcmp(axisName, "wrist") == 0) {
    int safe_wrist = limitRange(angle, pot_Wrist) +rig.ofst_wri;

    servo_wrist_L.write(     safe_wrist                  );
    servo_wrist_R.write(180 -safe_wrist +rig.mir_wri_ofst);
                
    delay(1500);
    Serial.print  (F("wrist potAngle : "));
    Serial.print  (angle);
    Serial.print  (F(" > "));
    Serial.println( readAngle(pot_Wrist)  );
  }


  // =======================================================
  // WristRoll
  // =======================================================
  else if(strcmp(axisName, "wristRoll") == 0) {
    
    servo_wrist_roll.write(angle);
  }
}

void setTargetTo(Position newPos) {

  targetPos    = newPos;
  isMoving     = true;
  currentSpeed = 0.0f;
  lastStepTime = millis();  // reset step timer
}

void arrivedAtPos() {

  currentSpeed = 0.0;
  isMoving     = false;

  Serial.println(F("RES:ARRIVED"));

  Serial.print  (F("Arrived at:  X "));
  Serial.print  ( targetPos.x    );
  Serial.print  (F("  Y "));
  Serial.print  ( targetPos.y    );
  Serial.print  (F("  Z "));
  Serial.print  ( targetPos.z    );
  Serial.print  (F("  P "));
  Serial.print  ( targetPos.pitch);
  Serial.print  (F("  R "));
  Serial.print  ( targetPos.roll );
  Serial.print  (F("  MT "));
  Serial.println( targetPos.moveType );
}


// ====================================================
// Setup
// ====================================================
void initController() {
 
  delay(200);

  hasInit = true;

  servo_base       .attach(3);
  servo_shoulder_L .attach(4);
  servo_shoulder_R .attach(5);
  servo_elbow_L    .attach(6);
  servo_elbow_R    .attach(7);
  servo_wrist_L    .attach(8);
  servo_wrist_R    .attach(9);
  servo_wrist_roll .attach(10);

  int potAngle_Base       = servoAdjust_FK(readAngle( pot_Base     ), table_Base    );
  int potAngle_Shoulder   = servoAdjust_FK(readAngle( pot_Shoulder ), table_Shoulder);
  int potAngle_Elbow      = servoAdjust_FK(readAngle( pot_Elbow    ), table_Elbow   );
  int potAngle_Wrist      = readAngle( pot_Wrist );

  int moveAngle_Base      = round( (potAngle_Base -pot_Base.minRange) * 180.0f / (float)(pot_Base.maxRange -pot_Base.minRange) ); // for 270° servo
  int moveAngle_Shoulder  = potAngle_Shoulder +rig.ofst_sho;
  int moveAngle_Elbow     = potAngle_Elbow    +rig.ofst_elb;
  int moveAngle_Wrist     = potAngle_Wrist    +rig.ofst_wri;
  int moveAngle_WristRoll = 90                +homePos.roll;

  // ****************************************************************
  servo_base       .write(     moveAngle_Base                      );
  // ****************************************************************
  servo_shoulder_L .write(     moveAngle_Shoulder                  );
  servo_shoulder_R .write(180 -moveAngle_Shoulder +rig.mir_sho_ofst);
  // ****************************************************************
  servo_elbow_L    .write(180 -moveAngle_Elbow    +rig.mir_elb_ofst);
  servo_elbow_R    .write(     moveAngle_Elbow                     );
  // ****************************************************************
  servo_wrist_L    .write(     moveAngle_Wrist                     );
  servo_wrist_R    .write(180 -moveAngle_Wrist    +rig.mir_wri_ofst);
  // ****************************************************************
  servo_wrist_roll .write(     moveAngle_WristRoll                 );
  // ****************************************************************

  delay(1000);

  potAngle_Base     = servoAdjust_FK(readAngle( pot_Base     ), table_Base    );
  potAngle_Shoulder = servoAdjust_FK(readAngle( pot_Shoulder ), table_Shoulder);
  potAngle_Elbow    = servoAdjust_FK(readAngle( pot_Elbow    ), table_Elbow   );
  potAngle_Wrist    = readAngle( pot_Wrist );

  currentPos = forwardKinematics(
    potAngle_Base,
    potAngle_Shoulder,
    potAngle_Elbow,
    potAngle_Wrist
  );

  setTargetTo(homePos);
  
  // Serial.print  (F("RES:CONNECTED > at:  X "));
  // Serial.print  ( floor(currentPos.x) );
  // Serial.print  (F("  Y "));
  // Serial.print  ( floor(currentPos.y) );
  // Serial.print  (F("  Z "));
  // Serial.print  ( floor(currentPos.z) );
  // Serial.print  (F("  P "));
  // Serial.print  ( floor(currentPos.pitch) );
  // Serial.print  (F("  R "));
  // Serial.print  ( floor(currentPos.roll) );
  // Serial.print  (F("  MT "));
  // Serial.println( floor(currentPos.moveType) );
}


// ====================================================
// MovePos
// ====================================================
void moveServosTo(Position newPos) {
  
  JointAngles angles = inverseKinematics(newPos);

  // Safe limit
  if(  isnan(angles.tau    )
    || isnan(angles.gamma  )
    || isnan(angles.lambda )
    || isnan(angles.epsilon)) {
    return;
  }

  int potAngle_Base       = servoAdjust_IK(angles.epsilon, table_Base    );

  int moveAngle_Base      = round( pot_Base.minRange + (potAngle_Base * (float)(pot_Base.maxRange -pot_Base.minRange) / 180.0) ); // for 270° servo
  int moveAngle_Shoulder  = servoAdjust_IK((int)angles.tau,   table_Shoulder) +rig.ofst_sho;
  int moveAngle_Elbow     = servoAdjust_IK((int)angles.gamma, table_Elbow   ) +rig.ofst_elb;
  int moveAngle_Wrist     =                (int)angles.lambda                 +rig.ofst_wri;
  int moveAngle_WristRoll =     90.0      +(int)newPos.roll;

  int safe_Base       = limitRange(moveAngle_Base,     pot_Base    );
  int safe_Shoulder   = limitRange(moveAngle_Shoulder, pot_Shoulder);
  int safe_Elbow      = limitRange(moveAngle_Elbow,    pot_Elbow   );
  int safe_Wrist      = limitRange(moveAngle_Wrist,    pot_Wrist   );

  // Initialize prev values (Only once)
  if(!isPrevInit) {
    prev_Base         = safe_Base;
    prev_Shoulder     = safe_Shoulder;
    prev_Elbow        = safe_Elbow;
    prev_Wrist        = safe_Wrist;
    prev_WristRoll    = 90;
    
    isPrevInit = true;
  }
  
  // **************************************************************
  if(isNewValue(prev_Base,       safe_Base)) {
    servo_base       .write(     safe_Base                       );
  } // ************************************************************
  if(isNewValue(prev_Shoulder,   safe_Shoulder)) {
    servo_shoulder_L .write(     safe_Shoulder                   );
    servo_shoulder_R .write(180 -safe_Shoulder +rig.mir_sho_ofst );
  }// *************************************************************
  if(isNewValue(prev_Elbow,      safe_Elbow)) {
    servo_elbow_L    .write(180 -safe_Elbow    +rig.mir_elb_ofst );
    servo_elbow_R    .write(     safe_Elbow                      );
  }// *************************************************************
  if(isNewValue(prev_Wrist,      safe_Wrist)) {
    servo_wrist_L    .write(     safe_Wrist                      );
    servo_wrist_R    .write(180 -safe_Wrist    +rig.mir_wri_ofst );
  }// *************************************************************
  if(isNewValue(prev_WristRoll,  moveAngle_WristRoll)) {
    servo_wrist_roll .write(     moveAngle_WristRoll             );
  } // ************************************************************
}


// ====================================================
// Lerp
// ====================================================
void linearInterpolation() {

  unsigned long now = millis();

  if(!isMoving || now -lastStepTime < stepInterval) return;

  float deltaTime   = (now -lastStepTime) /1000.0f;
  lastStepTime      = now;

  float deltaX      = targetPos.x     -currentPos.x;
  float deltaY      = targetPos.y     -currentPos.y;
  float deltaZ      = targetPos.z     -currentPos.z;
  float deltaRoll   = targetPos.roll  -currentPos.roll;
  float deltaPitch  = targetPos.pitch -currentPos.pitch;

  float dist_3D     = sqrt(
    deltaX *deltaX +
    deltaY *deltaY +
    deltaZ *deltaZ
  );

  float angle_3D    = sqrt(
    deltaPitch *deltaPitch +
    deltaRoll  *deltaRoll
  );

  float totalDist_3D = sqrt(
    dist_3D  *dist_3D +
    angle_3D *angle_3D
  );

  // Serial.print("Dist_3D : "     ); Serial.print  (dist_3D     );
  // Serial.print("Angle_3D : "    ); Serial.print  (angle_3D    );
  // Serial.print("TotalDist_3D : "); Serial.println(totalDist_3D);

  if(isnan(totalDist_3D)) return;


  // ==============================================
  // Arrived at targetPos
  // ==============================================
  if(totalDist_3D <= 1.0) { arrivedAtPos(); return; }


  // ==============================================
  // Acceleration / Deceleration
  // ==============================================
  float stoppingDist = (currentSpeed *currentSpeed) / (2.0f *deceleration);

  // Decelerate
  if(totalDist_3D <= stoppingDist) {
    currentSpeed -= deceleration *deltaTime;
    if(currentSpeed < 0.0f) currentSpeed = 0.0f;
  }
  
  // Accelerate
  else {
    currentSpeed += acceleration *deltaTime;
    if(currentSpeed > maxSpeed) currentSpeed = maxSpeed;
  }


  // ==============================================
  // Step move
  // ==============================================
  float stepSize = currentSpeed *deltaTime /totalDist_3D;

  if(stepSize > 1.0) stepSize = 1.0;

  currentPos.x     += deltaX     *stepSize;
  currentPos.y     += deltaY     *stepSize;
  currentPos.z     += deltaZ     *stepSize;
  currentPos.pitch += deltaPitch *stepSize;
  currentPos.roll  += deltaRoll  *stepSize;

  moveServosTo(currentPos);
}
