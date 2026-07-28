
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

  const float radius = sqrt(x*x + y*y);
  const float d = radius -g;
  const float e = z +f -c;
  const float w = sqrt(d*d + e*e);

  // Safe limit
  if(w < 0.4 *l || w > 1.95 *l) return angles;

  const float a = w /2;
  
  // Ratio calculations
  float ratioAlpha = a /l;
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
  
  bool isTop = z +f > c;

  angles.epsilon = degrees( atan2(y, x) );
  angles.gamma   = sigma *2;
  angles.tau     = isTop ? alpha +beta : alpha -beta;
  angles.lambda  = 270 -angles.tau -angles.gamma;
  
  // Serial.println("*******************");
  // Serial.print(" Tau : ");
  // Serial.print(angles.tau);
  // Serial.print(", Epsilon : ");
  // Serial.print(angles.epsilon);
  // Serial.print(", Gamma : ");
  // Serial.print(angles.gamma);
  // Serial.print(", Lambda : ");
  // Serial.print(angles.lambda);
  // Serial.print(", Beta : ");
  // Serial.print(beta);
  // Serial.print(", Phi : ");
  // Serial.print(phi);
  // Serial.print(", Alpha : ");
  // Serial.print(alpha);
  // Serial.print(", Sigma : ");
  // Serial.println(sigma);

  return angles;
}
