
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"

// Read one G-code line at a time from UGS
Position parseGcodeLine(String gcodeLine) {
  
  Position newPos;

  newPos.x        = NAN;
  newPos.y        = NAN;
  newPos.z        = NAN;
  newPos.pitch    = NAN;
  newPos.roll     = NAN;
  newPos.moveType = -1;

  // Trim any extra spaces or newlines
  gcodeLine.trim();

  if(gcodeLine.length() == 0) return newPos;

  int index_X     = gcodeLine.indexOf("X");
  newPos.x        = extractCoord(gcodeLine, index_X, "float");
  
  int index_Y     = gcodeLine.indexOf("Y");
  newPos.y        = extractCoord(gcodeLine, index_Y, "float");

  int index_Z     = gcodeLine.indexOf("Z");
  newPos.z        = extractCoord(gcodeLine, index_Z, "float");

  int index_P     = gcodeLine.indexOf("P");
  newPos.pitch    = extractCoord(gcodeLine, index_P, "float");

  int index_R     = gcodeLine.indexOf("R");
  newPos.roll     = extractCoord(gcodeLine, index_R, "float");

  int index_G     = gcodeLine.indexOf("G");
  newPos.moveType = extractCoord(gcodeLine, index_G, "int");

  if(isnan(newPos.x    )) newPos.x     = currentPos.x;
  if(isnan(newPos.y    )) newPos.y     = currentPos.y;
  if(isnan(newPos.z    )) newPos.z     = currentPos.z;
  if(isnan(newPos.roll )) newPos.roll  = currentPos.roll;
  if(isnan(newPos.pitch)) newPos.pitch = currentPos.pitch;

  newPos.x += offsetPos.x;
  newPos.y += offsetPos.y;
  newPos.z += offsetPos.z;

  // Serial.print   ("MoveType: ");
  // Serial.print   (newPos.moveType);
  // Serial.print   (", x: ");
  // Serial.print   (newPos.x);
  // Serial.print   (", y: ");
  // Serial.print   (newPos.y);
  // Serial.print   (", z: ");
  // Serial.println (newPos.z);

  return newPos;
}


float extractCoord(String line, int index, String varType) {

  if(index >= 0) {
    int endIndex = line.indexOf(' ', index);

    if(endIndex == -1) endIndex = line.length();
    
    String subStr = line.substring(index +1, endIndex);

    if(varType == "float") return subStr.toFloat();
    if(varType == "int"  ) return subStr.toInt();
  }

  return NAN;
}
