#include "AStar.h"
#include <stdlib.h>
#include <string.h>
/* Heuristic h(current, goal). Use AStar_heuristic_zero if row/col are unset. */
float AStar_heuristic_zero(pVertex current, pVertex goal) {
  (void)current;
  (void)goal;
  return 0.0f;
}

