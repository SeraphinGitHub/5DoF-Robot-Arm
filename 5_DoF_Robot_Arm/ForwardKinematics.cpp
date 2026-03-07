
#include <Arduino.h>
#include "headers/ForwardKinematics.h"
#include "headers/MotionController.h"


CartesianPos forwardKinematics(
  float  tau_Deg,
  float  gamma_Deg,
  float  lambda_Deg,
  float  epsilon_Deg,
  int    moveType
) {
  
  CartesianPos result;

  result.x = NAN;
  result.y = NAN;
  result.z = NAN;
  result.moveType = -1;

  float tau     = radians(tau_Deg);
  // float gamma   = radians(gamma_Deg);
  float lambda  = radians(lambda_Deg);
  float epsilon = radians(epsilon_Deg);
  
  // Robot dimensions in mm
  const int c = rig.c; // Base height
  const int l = rig.l; // Arms lengths (both the same size)
  const int g = rig.g; // Y tool's offset
  const int f = rig.f; // Z tool's offset

  float sigma = lambda /2;
  float a     = l * sin(sigma);
  float w     = a *2;

  float ratio = a / l;
  ratio       = constrain(ratio, -1.0, 1.0);
  float alpha = acos(ratio);
  float beta  = abs(tau - alpha);
  bool isTop  = (tau >= alpha);

  float d = w * cos(beta);
  float e = w * sin(beta);

  result.x = (w +g) * cos(epsilon);
  result.y = (w +g) * sin(epsilon);
  // result.y = d +g; // ==> old
  result.z = c +e -f;
  
  result.moveType = moveType;

  if(!isTop) result.z = c -e -f;

  return result;
}