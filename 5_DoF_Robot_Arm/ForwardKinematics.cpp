
#include <Arduino.h>
#include "headers/ForwardKinematics.h"
#include "headers/MotionController.h"


CartesianPos forwardKinematics(
  float  epsilon_Deg,
  float  tau_Deg,
  float  gamma_Deg
) {
  
  CartesianPos result;

  result.x = NAN;
  result.y = NAN;
  result.z = NAN;

  float tau     = radians(     tau_Deg    );
  float gamma   = radians(180 -gamma_Deg  );
  float epsilon = radians(     epsilon_Deg);
  
  // Robot dimensions in mm
  const int c  = rig.c; // Base height
  const int l  = rig.l; // Arms lengths (both the same size)
  const int g  = rig.g; // Y tool's offset
  const int f  = rig.f; // Z tool's offset

  float theta  = tau -radians(180 -gamma_Deg);
  float d      = l *cos(tau) + l *cos(theta);
  float e      = l *sin(tau) + l *sin(theta);

  float radius = d +g;

  result.x = radius *cos(epsilon);
  result.y = radius *sin(epsilon);
  result.z = c +e -f;

  return result;
}