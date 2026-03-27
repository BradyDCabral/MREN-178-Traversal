#ifndef ASTAR_H
#define ASTAR_H

#include "MazeLogic.h"

// Heuristic h(current, goal). Use AStar_heuristic_zero if row/col are unset.
typedef float (*AStar_Heuristic)(pVertex current, pVertex goal);

float AStar_heuristic_zero(pVertex current, pVertex goal);

// Manhattan distance on (row,col); if any coordinate is VERTEX_GRID_UNSET, returns 0
float AStar_Heuristic_Distance(pVertex current, pVertex goal);

// Uniform edge cost 1 between non-NULL Neighbours[i]. outPath length <= maxOut.
int AStar_Search(pVertex start, pVertex goal, AStar_Heuristic h,
                 pVertex *outPath, int maxOut, int *outLen);

#endif
