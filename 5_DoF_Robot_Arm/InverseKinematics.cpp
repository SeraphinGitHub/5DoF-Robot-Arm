
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

  // Unreachable / Singularity
  if(w == 0 || w > 1.95 *l) return angles;

  const float a = w /2;
  
  // Ratio calculations
  float ratioAlpha = a / l;
  float ratioBeta  = d /w;
  float ratioPhi   = e /w;

  // Clamp instead of reject
  ratioAlpha = constrain(ratioAlpha, -1.0, 1.0);
  ratioBeta  = constrain(ratioBeta , -1.0, 1.0);
  ratioPhi   = constrain(ratioPhi  , -1.0, 1.0);

  // Safe trigger
  float beta  = degrees( acos(ratioBeta ) );
  float phi   = degrees( acos(ratioPhi  ) );
  float alpha = degrees( acos(ratioAlpha) );
  float sigma = degrees( asin(ratioAlpha) );
  
  angles.gamma   = sigma *2;
  angles.lambda  = 270 -alpha -phi;
  angles.epsilon = degrees( atan2(y, x) );
  angles.tau     = alpha + beta;

  return angles;
}
