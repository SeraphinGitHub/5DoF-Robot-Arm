
#ifndef CALIBRATION_H
#define CALIBRATION_H

  struct Pot {
    int pin;           // Pin name
    int min;           // Pot value at 0°
    int max;           // Pot value at 180°
    int minRange;      // Range value between 0° to 270° (Depend on used servo)
    int maxRange;      // Range value between 0° to 270°
  };

  struct CalibTable {
    float real;
    float measured;
  };

  extern const Pot pot_Base;
  extern const Pot pot_Shoulder;
  extern const Pot pot_Elbow;
  extern const Pot pot_Wrist;

  extern CalibTable table_Base[];
  extern CalibTable table_Elbow[];
  extern CalibTable table_Shoulder[];

  extern int servoAdjust_IK(float IK_angle, CalibTable servoTable[]);
  extern int servoAdjust_FK(int   potAngle, CalibTable servoTable[]);

  // extern int getBaseFKAngle     ();
  // extern int getShoulderFKAngle ();
  // extern int getElbowFKAngle    ();

#endif