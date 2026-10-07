
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

  #include <Arduino.h>
  #include "Calibration.h"

  // Robot dimensions in mm
  struct Rig {
    int c;             // Base height
    int l;             // Arm lengths (both the same size)
    int g;             // Y Wrist offset
    int f;             // Z Wrist offset

    int ofst_base;     // Base       offset in degrees
    int ofst_sho;      // Shoulder   offset in degrees
    int ofst_elb;      // Elbow      offset in degrees
    int ofst_wri;      // Wrist      offset in degrees
    int ofst_wriRoll;  // Wrist Roll offset in degrees

    int mir_sho_ofst;  // Shoulder_R angle offset (mirror servo)
    int mir_elb_ofst;  // Elbow_R    angle offset (mirror servo)
    int mir_wri_ofst;  // Wrist_R    angle offset (mirror servo)
  };

  struct Position {
    float x;
    float y;
    float z;
    float pitch;
    float roll;

    int moveType;
  };

  struct Coords {
    float x;
    float y;
    float z;
  };

  extern const  Rig rig;
  extern bool   hasInit;
  
  extern Position homePos;
  extern Position programPos;
  extern Position currentPos;
  extern Coords   offsetPos;

  int  readAngle            (const Pot& pot);
  int  limitRange           (int angle, const Pot& pot);
  bool isNewValue           (int &prevAngle, int newAngle);

  // ********** Tempory Demo **********
  void startDemo ();
  void endDemo   ();
  void updateDemo();
  // ********** Tempory Demo **********
  
  void initController       ();
  void changeVarValue       (char* cmd_Buff);
  void manuAngleMove        (char* cmd_Buff);
  void setTargetTo          (Position newPos);
  void moveServosTo         (Position newPos);
  void linearInterpolation  ();

#endif