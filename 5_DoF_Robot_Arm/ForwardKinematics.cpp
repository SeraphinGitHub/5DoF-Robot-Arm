
#include <Arduino.h>
#include "headers/ForwardKinematics.h"
#include "headers/MotionController.h"


Position forwardKinematics(
  float  deg_epsilon,
  float  deg_tau,
  float  deg_gamma,
  float  deg_lambda
) {
  
  Position newPos;

  newPos.x        = NAN;
  newPos.y        = NAN;
  newPos.z        = NAN;
  newPos.pitch    = homePos.pitch;
  newPos.roll     = homePos.roll;
  newPos.moveType = homePos.moveType;

  // Robot dimensions in mm
  float c       = rig.c; // Base height
  float l       = rig.l; // Arms lengths (both the same size)
  float g       = rig.g; // Y tool's offset
  float f       = rig.f; // Z tool's offset
  
  float epsilon = radians( deg_epsilon );
  float tau     = radians( deg_tau     );
  float gamma   = radians( deg_gamma   );
  float lambda  = radians( deg_lambda  );
  
  float alpha   = (PI -gamma) *0.5; // PI = 180°
  float beta    = tau -alpha;
  float phi     = PI -lambda -alpha;
  float w       = l *cos(alpha) *2;
  float d       = w *cos(beta);
  float e       = w *sin(beta);
  float radius  = d +g;

  newPos.x        = radius *cos(epsilon);
  newPos.y        = radius *sin(epsilon);
  newPos.z        = phi < PI *0.5 ? c +e -f : c -e -f; // if phi < 90°
  
  // X80 Y350 Z150
  // Serial.println("*****************************************");
  // Serial.print  ("X : "   );
  // Serial.print  (newPos.x );
  // Serial.print  (", Y : " );
  // Serial.print  (newPos.y );
  // Serial.print  (", Z : " );
  // Serial.println(newPos.z );

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

  return newPos;
}