
#include <Arduino.h>
#include "headers/GCodePrograms.h"


// const char* program_001[]


// Shoe
String program_001[] = {
  "G1 X0 Y200 Z100",
  "Z50",
  "X-100",
  "Y70",
  "X100",
  "Y100",
  "X50 Y150",
  "X0",
  "Y200",
  "X-25",
  "Y90",
  "X-50",
  "Y200",
  "X-100",
  "X0 Y100 Z150"
};

int program_001_Length = sizeof(program_001) / sizeof(program_001[0]);

// Chevron niku mori
String program_002[] = {
  "G1 X-50 Y200 Z100",
  "Z50",
  "X50",
  "Y110",
  "X25 Y70",
  "X-25",
  "X-50 Y110",
  "Y200",
  "Z80",
  
  // 1st Chev
  "X-40 Y190",
  "Z50",
  "X-5 Y155",
  "Z80",

  "X40 Y190",
  "Z50",
  "X5 Y155",
  "Z80",

  // 2nd
  "X-40 Y160",
  "Z50",
  "X-5 Y125",
  "Z80",

  "X40 Y160",
  "Z50",
  "X5 Y125",
  "Z80",

  // 3rd
  "X-40 Y130",
  "Z50",
  "X-5 Y95",
  "Z80",

  "X40 Y130",
  "Z50",
  "X5 Y95",
  "Z80",
  
  "X0 Y100 Z150"
};

int program_002_Length = sizeof(program_002) / sizeof(program_002[0]);


String program_003[] = {
  "G1 X50 Y120 Z80",
  "Z0",
  "X-50",
  "Y200",
  "X50",
  "Y120",
  "X-50 Y200",
  "X0 Y240",
  "X50 Y200",
  "X-50 Y120",
  "Z80",
  "X0 Y0 Z150"
};

int program_003_Length = sizeof(program_003) / sizeof(program_003[0]);