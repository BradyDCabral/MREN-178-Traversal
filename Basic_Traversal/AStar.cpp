#include "AStar.h"
#include <stdlib.h>
#include <string.h>

float AStar_heuristic_zero(pVertex current, pVertex goal) {
  (void)current;
  (void)goal;
  return 0.0f;
}

