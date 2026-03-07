
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"

float old_X = currentPos.x;
float old_Y = currentPos.y;
float old_Z = currentPos.z;

// Read one G-code line at a time from UGS
CartesianPos parseGcodeLine(String gcodeLine) {
  
  CartesianPos result;

  result.x = NAN;
  result.y = NAN;
  result.z = NAN;
  result.moveType = NAN;

  // Trim any extra spaces or newlines
  gcodeLine.trim();

  if(gcodeLine.length() == 0) return result;

  int index_X     = gcodeLine.indexOf("X");
  result.x        = extractCoord(gcodeLine, index_X, "float");
  
  int index_Y     = gcodeLine.indexOf("Y");
  result.y        = extractCoord(gcodeLine, index_Y, "float");

  int index_Z     = gcodeLine.indexOf("Z");
  result.z        = extractCoord(gcodeLine, index_Z, "float");

  int index_G     = gcodeLine.indexOf("G");
  result.moveType = extractCoord(gcodeLine, index_G, "int");

  if(!isnan(result.x)) old_X = result.x;
  if(!isnan(result.y)) old_Y = result.y;
  if(!isnan(result.z)) old_Z = result.z;

  if(isnan(result.x) && !isnan(old_X)) result.x = old_X;
  if(isnan(result.y) && !isnan(old_Y)) result.y = old_Y;
  if(isnan(result.z) && !isnan(old_Z)) result.z = old_Z;

  // Serial.print   ("MoveType: ");
  // Serial.print   (result.moveType);
  // Serial.print   (", x: ");
  // Serial.print   (result.x);
  // Serial.print   (", y: ");
  // Serial.print   (result.y);
  // Serial.print   (", z: ");
  // Serial.println (result.z);

  return result;
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
