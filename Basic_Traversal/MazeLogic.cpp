#include <cmath>
#include <stdlib.h>
#include "MazeLogic.h"



Tree* createTree() {
  pTree ptree = (pTree)malloc(sizeof(Tree));
  return ptree;
}


int CreateVertex(pVertex new_vertex, float target_angle, float L_distance, float F_distance, float R_distance) {

  return 0;
}


int ReturnProperIndex(float angle) {
  // Angle | Index
  // 0/360 |  0
  // 90    |  1
  // 180   |  2
  // 270   |  3
  int return_ind;
  if (abs((std::fmod)((angle), 360)) < Angle_Threshold) {
    return_ind = 0;
  }
  // FINISH DUMBASS

  // ALSO MAKE THIS SMARTER

  return 0;
}