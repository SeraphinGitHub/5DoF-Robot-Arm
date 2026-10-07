
#include <Arduino.h>
#include "headers/InverseKinematics.h"
#include "headers/MotionController.h"


JointAngles inverseKinematics(Position newPos) {

  JointAngles angles;

  angles.epsilon = NAN;
  angles.tau     = NAN;
  angles.gamma   = NAN;
  angles.lambda  = NAN;

  float x        = newPos.x;
  float y        = newPos.y;
  float z        = newPos.z;
  float pitch    = newPos.pitch;
  float roll     = newPos.roll;
  
  // Robot dimensions in mm
  float c        = rig.c; // Base heights
  float l        = rig.l; // Arms lengths (both the same size)
  float g        = rig.g; // Y tool's offset
  float f        = rig.f; // Z tool's offset

  // Distance from base to tool tip
  float radius   = sqrt (x*x + y*y);

  // Wrist & tool tip projections
  float pitchRad = radians(pitch);
  float rollRad  = radians(roll );

  float rXY      =       sin(rollRad ) *f   ; // roll  angle f opposite side
  float rZ       = fabs( cos(rollRad ) *f  ); // roll  angle f adjacent side
  float gXY      = fabs( cos(pitchRad) *g  ); // pitch angle g adjacent side
  float gZ       = fabs( sin(pitchRad) *g  ); // pitch angle g opposite side
  float fXY      = fabs( sin(pitchRad) *rZ ); // pitch angle f opposite side
  float fZ       = fabs( cos(pitchRad) *rZ ); // pitch angle f adjacent side
  
  // Tool tip keep same Z pos when pitch change
  float wri_Z    = pitch > 0
    ? z +fZ -gZ
    : z +fZ +gZ
  ;
  
  // Tool tip keep same XY pos when pitch change
  float d        = pitch > 0
    ? radius -fXY -gXY
    : radius +fXY -gXY
  ;

  // Tool tip keep same XYZ pos when roll change
  float tempEpsi = atan2  (y, x);
  float tool_X   = x + sin(tempEpsi) *rXY;
  float tool_Y   = y - cos(tempEpsi) *rXY;
  float epsilon  = atan2  (tool_Y, tool_X);

  // Arm geometry
  float e        = wri_Z -c;
  float w        = sqrt(d*d + e*e);
  float a        = w *0.5;

  // ===============================================
  // Safe limit
  // ===============================================
  if(w < f +10.0 || w > 1.95 *l) return angles;
  // ===============================================

  // Clamp ratios to avoid NAN
  float ratAlpha = constrain( a /l , -1.0, 1.0);

  // Calculate angles in Radians
  float alpha    = acos  (ratAlpha);
  float beta     = atan2 (e, d);
  float phi      = atan2 (d, e);
  float gamma    = PI  - (alpha *2); // PI = 180°
  float lambda   = PI  - alpha -phi;
  float tau      = wri_Z > c ? alpha +beta : alpha -beta;

  angles.epsilon = degrees( epsilon );
  angles.tau     = degrees( tau     );
  angles.gamma   = degrees( gamma   );
  angles.lambda  = degrees( lambda  ) -pitch;
  
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

  return angles;
}
