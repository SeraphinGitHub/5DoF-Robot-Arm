
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

  struct JointAngles {

    float epsilon;
    float gamma;
    float lambda;
    float tau;
  };

  JointAngles inverseKinematics(
    float x,
    float y,
    float z
  );

#endif