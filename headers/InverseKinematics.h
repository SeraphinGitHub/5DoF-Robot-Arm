
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

  struct JointAngles {

    float tau;
    float gamma;
    float lambda;
    float epsilon;
  };

  JointAngles inverseKinematics(
    float x,
    float y,
    float z
  );

#endif