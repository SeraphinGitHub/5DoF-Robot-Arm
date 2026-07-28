
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

  #include "GcodeParser.h"

  CartesianPos forwardKinematics(
    float  epsilon_Deg,
    float  tau_Deg,
    float  gamma_Deg
  );

#endif