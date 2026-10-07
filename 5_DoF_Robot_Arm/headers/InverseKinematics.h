
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

  #include "GcodeParser.h"

  struct JointAngles {

    float epsilon;
    float tau;
    float gamma;
    float lambda;
  };

  JointAngles inverseKinematics(Position newPos);

#endif