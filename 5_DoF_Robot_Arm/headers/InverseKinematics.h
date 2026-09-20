
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

  struct JointAngles {

    float epsilon;
    float tau;
    float gamma;
    float lambda;
  };

  JointAngles inverseKinematics(
    float x,
    float y,
    float z
  );

#endif