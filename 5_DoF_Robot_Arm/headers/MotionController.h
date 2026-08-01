
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

  #include <Arduino.h>
  #include "GcodeParser.h"

  // Robot dimensions in mm
  struct Rig {

    int c;             // Base height
    int l;             // Arm lengths (both the same size)
    int g;             // Y Wrist offset
    int f;             // Z Wrist offset

    int ofst_base;     // Base       offset in degrees
    int ofst_MG995;    // Sho & Elb  offset in degrees
    int ofst_sho;      // Shoulder   offset in degrees
    int ofst_elb;      // Elbow      offset in degrees
    int ofst_wri;      // Wrist      offset in degrees
    int ofst_wriRoll;  // Wrist Roll offset in degrees

    int mir_sho_ofst;  // Shoulder_R angle offset (mirror servo)
    int mir_elb_ofst;  // Elbow_R    angle offset (mirror servo)
    int mir_wri_ofst;  // Wrist_R    angle offset (mirror servo)
  };

  struct Pot {
    int pin;           // Pin name
    int min;           // Pot value at 0°
    int max;           // Pot value at 180°
    int minRange;      // Range value between 0° to 270° (Depend on used servo)
    int maxRange;      // Range value between 0° to 270°
  };

  extern const  Rig rig;
  
  extern CartesianPos homeCoords;
  extern CartesianPos currentPos;

  bool isNewValue   (int &prevAngle, int newAngle);
  int  readAngle    (const Pot& pot);
  int  limitRange   (int angle, const Pot& pot);
  
  void initController();
  void manuAngleMove(char* cmd_Buff);
  void setTargetTo  (CartesianPos coords);
  void moveServosTo (CartesianPos coords);
  void linear_Interpolation();



#endif