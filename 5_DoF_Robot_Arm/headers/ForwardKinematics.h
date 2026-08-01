
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

  #include "GcodeParser.h"

  CartesianPos forwardKinematics(
    float  deg_epsilon,
    float  deg_tau,
    float  deg_gamma,
    float  deg_lambda
  );

#endif