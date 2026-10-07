
#ifndef G_CODE_PARSER_H
#define G_CODE_PARSER_H

  #include "MotionController.h"

  Position parseGcodeLine(String gcodeLine);
  
  float extractCoord(String line, int index, String varType);

#endif