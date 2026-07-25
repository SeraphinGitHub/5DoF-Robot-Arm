
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

  #include <Arduino.h>
  #include "GcodeParser.h"

  // Robot dimensions in mm
  struct Rig {

    int c; // Base height
    int l; // Arm lengths (both the same size)
    int g; // Y Wrist offset
    int f; // Z Wrist offset

    int ofst_base;       // Base       offset in degrees
    int ofst_shoulder;   // Shoulder   offset in degrees
    int ofst_elbow;      // Elbow      offset in degrees
    int ofst_wrist;      // Wrist      offset in degrees
    int ofst_wrist_roll; // Wrist Roll offset in degrees

    int sho_R_ofst;      // Shoulder_R angle offset (mirror servo)
    int elb_R_ofst;      // Elbow_R    angle offset (mirror servo)
    int wri_R_ofst;      // Wrist_R    angle offset (mirror servo)
  };

  struct Pot {
    int pin;      // Pin name
    int min;      // Pot value at 0°
    int max;      // Pot value at 180°
    int minRange; // Range value between 0° to 270° (Depend on used servo)
    int maxRange; // Range value between 0° to 270°
  };

  extern const  Rig rig;
  
  extern CartesianPos currentPos;

  void TestServo(String cmd); // *************************************************************

  void init_Motion();

  int readAngle(const Pot& pot);
  
  void setTargetTo(CartesianPos coords);

  bool isNewValue(int &prevAngle, int newAngle);
  
  void moveServosTo(CartesianPos coords);
  
  void linear_Interpolation();



#endif