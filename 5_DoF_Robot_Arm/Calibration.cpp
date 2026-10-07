
#include <Arduino.h>
#include "headers/Calibration.h"

const Pot pot_Base     = { A0,  0, 1023, 24, 160 };
const Pot pot_Shoulder = { A1, 798, 117,  0, 180 };
const Pot pot_Elbow    = { A2, 211, 856,  0, 180 };
const Pot pot_Wrist    = { A3, 940, 244,  0, 180 };


CalibTable table_Base[] = {
  {   0,  46 },
  {  45,  74 },
  {  90,  96 },
  { 135, 122 },
  { 180, 147 }
};

CalibTable table_Shoulder[] = {
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

CalibTable table_Elbow[] = {
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


int servoAdjust_IK(float IK_angle, CalibTable servoTable[]) {
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

int servoAdjust_FK(int potAngle,   CalibTable servoTable[]) {
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



