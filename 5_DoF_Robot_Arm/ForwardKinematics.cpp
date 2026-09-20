
#include <Arduino.h>
#include "headers/ForwardKinematics.h"
#include "headers/MotionController.h"


CartesianPos forwardKinematics(
  float  deg_epsilon,
  float  deg_tau,
  float  deg_gamma,
  float  deg_lambda
) {
  
  CartesianPos coords;

  coords.x = NAN;
  coords.y = NAN;
  coords.z = NAN;

  // Robot dimensions in mm
  float c = rig.c; // Base height
  float l = rig.l; // Arms lengths (both the same size)
  float g = rig.g; // Y tool's offset
  float f = rig.f; // Z tool's offset
  
  float epsilon = radians( deg_epsilon );
  float tau     = radians( deg_tau     );
  float gamma   = radians( deg_gamma   );
  float lambda  = radians( deg_lambda  );
  
  float alpha   = (PI -gamma) *0.5; // PI = 180°
  float beta    = tau -alpha;
  // float phi     = PI *0.5 -beta;
  float phi     = PI -lambda -alpha;
  float w       = l *cos(alpha) *2;
  float d       = w *cos(beta);
  float e       = w *sin(beta);
  float radius  = d +g;

  coords.x = radius *cos(epsilon);
  coords.y = radius *sin(epsilon);
  coords.z = phi < PI *0.5 ? c +e -f : c -e -f; // if phi < 90°


  // X80 Y350 Z150
  // Serial.println("*****************************************");
  // Serial.print  ("X : "   );
  // Serial.print  (coords.x );
  // Serial.print  (", Y : " );
  // Serial.print  (coords.y );
  // Serial.print  (", Z : " );
  // Serial.println(coords.z );

  // Serial.print  ("W : "  );
  // Serial.print  (w       );
  // Serial.print  (", d : ");
  // Serial.print  (d       );
  // Serial.print  (", e : ");
  // Serial.println(e       );

  // Serial.print  ("c : "  );
  // Serial.print  (c       );
  // Serial.print  (", l : ");
  // Serial.print  (l       );
  // Serial.print  (", g : ");
  // Serial.print  (g       );
  // Serial.print  (", f : ");
  // Serial.println(f       );

  // Serial.print  ("Alpha : "       );
  // Serial.print  ( degrees( alpha ));
  // Serial.print  (", Beta : "      );
  // Serial.print  ( degrees( beta  ));
  // Serial.print  (", Phi : "       );
  // Serial.println( degrees( phi   ));

  return coords;
}