
#include <Arduino.h>
#include "headers/GcodeParser.h"
#include "headers/MotionController.h"

float old_X = currentPos.x;
float old_Y = currentPos.y;
float old_Z = currentPos.z;

// Read one G-code line at a time from UGS
CartesianPos parseGcodeLine(String gcodeLine) {
  
  CartesianPos coords;

  coords.x = NAN;
  coords.y = NAN;
  coords.z = NAN;
  coords.moveType = NAN;

  // Trim any extra spaces or newlines
  gcodeLine.trim();

  if(gcodeLine.length() == 0) return coords;

  int index_X     = gcodeLine.indexOf("X");
  coords.x        = extractCoord(gcodeLine, index_X, "float");
  
  int index_Y     = gcodeLine.indexOf("Y");
  coords.y        = extractCoord(gcodeLine, index_Y, "float");

  int index_Z     = gcodeLine.indexOf("Z");
  coords.z        = extractCoord(gcodeLine, index_Z, "float");

  int index_G     = gcodeLine.indexOf("G");
  coords.moveType = extractCoord(gcodeLine, index_G, "int");

  if(!isnan(coords.x)) old_X = coords.x;
  if(!isnan(coords.y)) old_Y = coords.y;
  if(!isnan(coords.z)) old_Z = coords.z;

  if(isnan(coords.x) && !isnan(old_X)) coords.x = old_X;
  if(isnan(coords.y) && !isnan(old_Y)) coords.y = old_Y;
  if(isnan(coords.z) && !isnan(old_Z)) coords.z = old_Z;

  // Serial.print   ("MoveType: ");
  // Serial.print   (coords.moveType);
  // Serial.print   (", x: ");
  // Serial.print   (coords.x);
  // Serial.print   (", y: ");
  // Serial.print   (coords.y);
  // Serial.print   (", z: ");
  // Serial.println (coords.z);

  return coords;
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
