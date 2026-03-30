//Farmer was replaed is suh a good subreddit for astar implementations 

#include "AStar.h"
#include <string.h>

// Arrays are indexed by vertex Key, and Keys are 1..MAX_NODES.
// +1 lets us ignore index 0 and keep key math simple.
#define KEY_IDX (MAX_NODES + 1)

// Quick guard so random/bad keys do not index arrays out of bounds.
static int key_ok(uint32_t k) {
  return (k >= 1 && k <= (uint32_t)MAX_NODES);
}

/* Heuristic h(current, goal). Use AStar_heuristic_zero if row/col are unset. */
float AStar_heuristic_zero(pVertex current, pVertex goal) {
  // Intentionally ignored params: this is Dijkstra mode in disguise.
  (void)current;
  (void)goal;
  return 0.0f;
}

float AStar_Heuristic_Distance(pVertex current, pVertex goal) {
  // If either node is missing, we cannot estimate anything useful.
  if (!current || !goal)
    return 0.0f;
  // If grid coordinates are not assigned yet, fall back to zero heuristic.
  if (current->row == VERTEX_GRID_UNSET || current->col == VERTEX_GRID_UNSET ||
      goal->row == VERTEX_GRID_UNSET || goal->col == VERTEX_GRID_UNSET)
    return 0.0f;
  // Manhattan distance on maze grid (4-neighbour movement).
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
  // Input validation: fail fast, fail loud.
  if (!start || !goal || !outPath || !outLen || !h)
    return -1;
  // Keys must be within the array-backed indexing range.
  if (!key_ok(start->Key) || !key_ok(goal->Key))
    return -1;
  // Caller must provide room for at least one node in result path.
  if (maxOut < 1)
    return -1;

  // Static buffers avoid stack churn on microcontrollers.
  // This is single-run style state (not thread-safe, which is fine here).
  static float gScore[KEY_IDX];
  static pVertex parent[KEY_IDX];
  static bool closedSet[KEY_IDX];
  static bool inOpenSet[KEY_IDX];
  static pVertex openList[KEY_IDX];

  // Reset search state.
  memset(closedSet, 0, sizeof(closedSet));
  memset(inOpenSet, 0, sizeof(inOpenSet));
  for (int i = 0; i < KEY_IDX; i++) {
    gScore[i] = INF;
    parent[i] = nullptr;
  }

  uint32_t sk = start->Key;
  // Start node has zero path cost from itself.
  gScore[sk] = 0.0f;
  parent[sk] = nullptr;

  // Open set starts with exactly one brave little vertex.
  int openCount = 0;
  openList[openCount++] = start;
  inOpenSet[sk] = true;

  pVertex current = nullptr;

  while (openCount > 0) {
    // Pick node in open set with minimum f = g + h.
    // (Linear scan: simple and good enough for small MAX_NODES.)
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
    // Pop best candidate from open set (swap-with-last removal).
    current = openList[bestIdx];
    openList[bestIdx] = openList[openCount - 1];
    openCount--;
    inOpenSet[current->Key] = false;

    // Mark node as fully processed.
    uint32_t ck = current->Key;
    closedSet[ck] = true;

    // Goal reached. 
    if (current == goal)
      break;

    // Expand up to 4 neighbours (N/E/S/W style adjacency).
    for (int i = 0; i < 4; i++) {
      pVertex nb = current->Neighbours[i];
      // Wall / missing edge.
      if (!nb)
        continue;
      // Safety check against malformed graph keys.
      if (!key_ok(nb->Key))
        continue;
      uint32_t nk = nb->Key;
      // Already finalized with lowest known cost.
      if (closedSet[nk])
        continue;

      // Uniform step cost of 1 edge per move.
      float tentative = gScore[ck] + 1.0f;
      if (tentative < gScore[nk]) {
        // Found a better path to neighbour; update parent + cost.
        parent[nk] = current;
        gScore[nk] = tentative;
        if (!inOpenSet[nk]) {
          // Keep open set bounded by MAX_NODES.
          if (openCount >= MAX_NODES)
            return -1;
          openList[openCount++] = nb;
          inOpenSet[nk] = true;
        }
      }
    }
  }

  // If loop ended without landing on goal, no path exists in current graph.
  if (current != goal)
    return -1;

  // Reconstruct path by following parent links from goal back to start.
  pVertex rev[KEY_IDX];
  int n = 0;
  for (pVertex v = goal; v && n < KEY_IDX; v = parent[v->Key]) {
    rev[n++] = v;
    if (v == start)
      break;
  }
  // If we never reached start while backtracking, graph/parent chain is invalid.
  if (n == 0 || rev[n - 1] != start)
    return -1;
  // Caller output buffer too small for computed path.
  if (n > maxOut)
    return -1;

  // Reverse into forward path: start -> ... -> goal.
  for (int i = 0; i < n; i++)
    outPath[i] = rev[n - 1 - i];
  // Path length payload for caller.
  *outLen = n;
  return 0;
  // Zero means success; yes, i think this is correct 
  // ooh 
}
