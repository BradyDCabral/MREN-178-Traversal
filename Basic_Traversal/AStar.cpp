#include "AStar.h"
#include <string.h>

#define KEY_IDX (MAX_NODES + 1)

static int key_ok(uint32_t k) {
  return (k >= 1 && k <= (uint32_t)MAX_NODES);
}

/* Heuristic h(current, goal). Use AStar_heuristic_zero if row/col are unset. */
float AStar_heuristic_zero(pVertex current, pVertex goal) {
  (void)current;
  (void)goal;
  return 0.0f;
}

float AStar_Heuristic_Distance(pVertex current, pVertex goal) {
  if (!current || !goal)
    return 0.0f;
  if (current->row == VERTEX_GRID_UNSET || current->col == VERTEX_GRID_UNSET ||
      goal->row == VERTEX_GRID_UNSET || goal->col == VERTEX_GRID_UNSET)
    return 0.0f;
  int dr = (int)current->row - (int)goal->row;
  int dc = (int)current->col - (int)goal->col;
  if (dr < 0)
    dr = -dr;
  if (dc < 0)
    dc = -dc;
  return (float)(dr + dc);
}

int AStar_Search(pVertex start, pVertex goal, AStar_Heuristic h,
                 pVertex *outPath, int maxOut, int *outLen) {
  if (!start || !goal || !outPath || !outLen || !h)
    return -1;
  if (!key_ok(start->Key) || !key_ok(goal->Key))
    return -1;
  if (maxOut < 1)
    return -1;

  static float gScore[KEY_IDX];
  static pVertex parent[KEY_IDX];
  static bool closedSet[KEY_IDX];
  static bool inOpenSet[KEY_IDX];
  static pVertex openList[KEY_IDX];

  memset(closedSet, 0, sizeof(closedSet));
  memset(inOpenSet, 0, sizeof(inOpenSet));
  for (int i = 0; i < KEY_IDX; i++) {
    gScore[i] = INF;
    parent[i] = nullptr;
  }

  uint32_t sk = start->Key;
  gScore[sk] = 0.0f;
  parent[sk] = nullptr;

  int openCount = 0;
  openList[openCount++] = start;
  inOpenSet[sk] = true;

  pVertex current = nullptr;

  while (openCount > 0) {
    int bestIdx = 0;
    float bestF = gScore[openList[0]->Key] + h(openList[0], goal);
    for (int i = 1; i < openCount; i++) {
      uint32_t k = openList[i]->Key;
      float f = gScore[k] + h(openList[i], goal);
      if (f < bestF) {
        bestF = f;
        bestIdx = i;
      }
    }
    current = openList[bestIdx];
    openList[bestIdx] = openList[openCount - 1];
    openCount--;
    inOpenSet[current->Key] = false;

    uint32_t ck = current->Key;
    closedSet[ck] = true;

    if (current == goal)
      break;

    for (int i = 0; i < 4; i++) {
      pVertex nb = current->Neighbours[i];
      if (!nb)
        continue;
      if (!key_ok(nb->Key))
        continue;
      uint32_t nk = nb->Key;
      if (closedSet[nk])
        continue;

      float tentative = gScore[ck] + 1.0f;
      if (tentative < gScore[nk]) {
        parent[nk] = current;
        gScore[nk] = tentative;
        if (!inOpenSet[nk]) {
          if (openCount >= MAX_NODES)
            return -1;
          openList[openCount++] = nb;
          inOpenSet[nk] = true;
        }
      }
    }
  }

  if (current != goal)
    return -1;

  pVertex rev[KEY_IDX];
  int n = 0;
  for (pVertex v = goal; v && n < KEY_IDX; v = parent[v->Key]) {
    rev[n++] = v;
    if (v == start)
      break;
  }
  if (n == 0 || rev[n - 1] != start)
    return -1;
  if (n > maxOut)
    return -1;

  for (int i = 0; i < n; i++)
    outPath[i] = rev[n - 1 - i];
  *outLen = n;
  return 0;
}
