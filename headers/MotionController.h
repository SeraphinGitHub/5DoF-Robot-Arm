
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

  #include <Arduino.h>
  #include "GcodeParser.h"

  // Robot dimensions in mm
  struct Rig {

    int c; // Base height
    int l; // Arm lengths (both the same size)
    int g; // Y tool's offset
    int f; // Z tool's offset

    float offset_0; // Base offset in degrees
    float offset_1; // Arm1 offset in degrees
    float offset_2; // Arm2 offset in degrees
    float offset_3; // tool offset in degrees
  };

  extern const  Rig rig;
  
  extern bool   isProgMode;
  
  extern CartesianPos currentPos;

  void init_Motion();

  void home();
  
  void runProgram();
  
  void setTargetTo(CartesianPos coords);

  bool isNewValue(int &prevAngle, int newAngle);
  
  void moveServosTo(CartesianPos coords);
  
  void normalizedMove();

#endif