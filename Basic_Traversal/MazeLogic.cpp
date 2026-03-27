#include <cmath>
#include <stdlib.h>
#include "MazeLogic.h"



Graph* createGraph() {
  pGraph pgraph = (pGraph)malloc(sizeof(Graph));
  return pgraph;
}

int CreateEmptyVertex(pVertex *new_vertex, int *count) {
  // Makes a green Vertex will little to no information stored in it
  *new_vertex = (pVertex)malloc(sizeof(Vertex));
  (*new_vertex)->Key = ++(*count);
  (*new_vertex)->Colour = GREEN;

  return 0;
}

// UNFINISHED    
int CreateVertex(pVertex new_vertex, float target_angle, float L_distance, float F_distance, float R_distance, int *count) {
  
  int node_ind = ReturnProperIndex(target_angle);
  // new_vertex is already initialized
  if (L_distance >= MAX_WALL_DIST) { // NODE TO LEFT OF BOT
    node_ind = (node_ind+3)%4;
    CreateEmptyVertex(&(new_vertex->Neighbours[node_ind]), count);
    // make "child" point to "parent"
    new_vertex->Neighbours[node_ind]->Neighbours[(node_ind+2)%4] = new_vertex;
  }

  return 0;
}


int ReturnProperIndex(float angle) {
  // Angle | Index
  // 0/360 |  0
  // 90    |  1
  // 180   |  2
  // 270   |  3
  int return_ind;
  if (abs((std::fmod)((angle), 360)) < Angle_Threshold) { // 0
    return_ind = 0;
  } else if (abs((std::fmod)((angle + 270), 360)) < Angle_Threshold) { // 90
    return_ind = 1;
  } else if (abs((std::fmod)((angle + 180), 360)) < Angle_Threshold) { // 180
    return_ind = 2;
  } else if (abs((std::fmod)((angle + 90), 360)) < Angle_Threshold) { // 270
    return_ind = 3;
  }
  return return_ind;
}

float ReturnProperAngleFromIndex(int ind) {
  return ((float)(ind % 4)) * 90;
}