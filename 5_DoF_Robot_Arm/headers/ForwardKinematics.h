
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

  #include "GcodeParser.h"

  CartesianPos forwardKinematics(
    float  tau_Deg,
    float  gamma_Deg,
    float  lambda_Deg,
    float  epsilon_Deg,
    int    moveType
  );

#endif