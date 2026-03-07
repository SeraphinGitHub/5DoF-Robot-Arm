
#ifndef G_CODE_PARSER_H
#define G_CODE_PARSER_H

  struct CartesianPos {

    float x;
    float y;
    float z;

    int moveType;
  };

  CartesianPos parseGcodeLine(String gcodeLine);
  
  float extractCoord(String line, int index, String varType);

#endif