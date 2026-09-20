
#include <Arduino.h>
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


JointAngles inverseKinematics(float x ,float y ,float z) {

  JointAngles angles;

  angles.epsilon = NAN;
  angles.tau     = NAN;
  angles.gamma   = NAN;
  angles.lambda  = NAN;
  
  // Robot dimensions in mm
  float c = rig.c; // Base height
  float l = rig.l; // Arms lengths (both the same size)
  float g = rig.g; // Y tool's offset
  float f = rig.f; // Z tool's offset

  // Distance base > tool's center
  float radius  = sqrt(x*x + y*y);
  
  // Wrist position
  float wZ = z +f;
  float d  = radius -g;
  float e  = wZ -c;

  // Arm geometry
  float w  = sqrt(d*d + e*e);
  float a  = w *0.5;

  // ===============================================
  // Safe limit
  // ===============================================
  if(w < f +10 || w > 1.95 *l) return angles;
  // ===============================================

  // Clamp ratios to avoid NAN
  float ratAlpha = constrain( a /l , -1.0, 1.0);

  // Calculate angles in Radians
  float alpha    = acos  (ratAlpha);
  float beta     = atan2 (e, d);
  float phi      = atan2 (d, e);
  float epsilon  = atan2 (y, x);
  float gamma    = PI  - (alpha *2); // PI = 180°
  float lambda   = PI  - alpha -phi;
  float tau      = wZ > c ? alpha +beta : alpha -beta;

  angles.epsilon = degrees( epsilon );
  angles.tau     = degrees( tau     );
  angles.gamma   = degrees( gamma   );
  angles.lambda  = degrees( lambda  );
  
  // X80 Y350 Z150
  // Serial.println("*****************************************");
  // Serial.print  ("Epsilon : "  );
  // Serial.print  (angles.epsilon);
  // Serial.print  (", Gamma : "  );
  // Serial.print  (angles.gamma  );
  // Serial.print  (", Lambda : " );
  // Serial.print  (angles.lambda );
  // Serial.print  (", Tau : "    );
  // Serial.println(angles.tau    );

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

  // *****************************************
  // Epsilon : 77.12, Gamma : 120.12, Lambda : 82.01, Tau : 51.89
  // W : 346.62, d : 321.49, e : 129.59
  // c : 100.00, l : 200.00, g : 37.00, f : 80.00
  // Alpha : 29.94, Beta : 21.95, Phi : 68.05

  return angles;
}
