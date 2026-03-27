#ifndef ASTAR_H
#define ASTAR_H

#include "MazeLogic.h"

typedef float(AStar_Heuristic)(pVertex Current, pVertex Goal);

float AStar_Heuristic_Distance(pVertex Current, pVertex Goal);

int AStar_Search(pVertex Start, pVertex Goal, AStar_Heuristic h, pVertex *outPath, int maxOut, int *outLen););



#endif