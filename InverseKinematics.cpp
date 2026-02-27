
#include <Arduino.h>
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


JointAngles inverseKinematics(float x ,float y ,float z) {

  JointAngles angles;

  angles.tau     = NAN;
  angles.gamma   = NAN;
  angles.lambda  = NAN;
  angles.epsilon = NAN;
  
  // Robot dimensions in mm
  const int   c = rig.c; // Base height
  const int   l = rig.l; // Arms lengths (both the same size)
  const int   g = rig.g; // Y tool's offset
  const int   f = rig.f; // Z tool's offset
  
  const float d = sqrt(x*x + y*y) -g;
  const float e = z +f -c;
  const float w = sqrt( d*d + e*e );
  const float a = w /2;
  
  float beta    = degrees( acos(d /w) );
  float phi     = degrees( acos(e /w) );
  float alpha   = degrees( acos(a /l) );
  float sigma   = degrees( asin(a /l) );
  
  angles.gamma   = sigma *2;
  angles.lambda  = 270 -alpha -phi;
  angles.epsilon = degrees( atan2(y, x) );
  angles.tau     = alpha + beta;

  // Safe limit
  if(  w > 1.95*l || e < 0.5 ) {
    angles.tau     = NAN;
    angles.gamma   = NAN;
    angles.lambda  = NAN;
    angles.epsilon = NAN;
  }

  return angles;
}
