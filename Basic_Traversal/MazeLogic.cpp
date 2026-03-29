/*
 * Maze graph: vertices = intersections, Neighbours[0..3] = passages in world frame
 * (see ReturnProperIndex: 0=N, 1=E, 2=S, 3=W). NULL = wall / no edge.
 * row/col are for A* heuristic; set separately when you map cells to a grid.
 */

#include <cmath>
#include <stdlib.h>
#include <string.h>
#include "MazeLogic.h"

// Allocate an empty Graph; Root is null until you CreateEmptyVertex(&Root, ...).
Graph* createGraph() {
  pGraph pgraph = (pGraph)malloc(sizeof(Graph));
  if (pgraph)
    pgraph->Root = nullptr;
  return pgraph;
}

// Allocate a new vertex, assign Key from *count, zero neighbours, mark grid unknown.
// *count must stay <= MAX_NODES (A* indexes Keys in that range).
int CreateEmptyVertex(pVertex *new_vertex, int *count) {
  if (!new_vertex || !count)
    return -1;
  if (*count >= MAX_NODES)
    return -1;
  *new_vertex = (pVertex)malloc(sizeof(Vertex));
  if (!*new_vertex)
    return -1;
  (*new_vertex)->Key = ++(*count);
  (*new_vertex)->Colour = GREEN;
  memset((*new_vertex)->Neighbours, 0, sizeof((*new_vertex)->Neighbours));
  (*new_vertex)->row = VERTEX_GRID_UNSET;
  (*new_vertex)->col = VERTEX_GRID_UNSET;

  return 0;
}

// If ToF `dist` is "open" (>= MAX_WALL_DIST), ensure there is a neighbour at world direction `dir`.
// Skips if already linked. Links v<->child on opposite faces: child sits "dir" from v, v sits (dir+2)%4 from child.
static int add_opening(pVertex v, int *count, float dist, int dir) {
  if (!v || !count || dir < 0 || dir > 3)
    return -1;
  if (dist < MAX_WALL_DIST)
    return 0;
  if (*count >= MAX_NODES)
    return -1;
  if (v->Neighbours[dir])
    return 0;

  pVertex child = nullptr;
  if (CreateEmptyVertex(&child, count) != 0 || !child)
    return -1;
  int opp = (dir + 2) % 4;
  v->Neighbours[dir] = child;
  child->Neighbours[opp] = v;
  return 0;
}

// At the current intersection vertex: from sensor readings, add new vertices for open F/L/R.
// target_angle = robot yaw in same convention as ReturnProperIndex (world N/E/S/W).
// Forward edge uses index node_ind; left = (node_ind+3)%4; right = (node_ind+1)%4.
// Does not merge revisits — same physical cell may get duplicate nodes until you add that logic.
int CreateVertex(pVertex new_vertex, float target_angle, float L_distance, float F_distance, float R_distance, int *count) {
  if (!new_vertex || !count)
    return -1;

  int node_ind = ReturnProperIndex(target_angle);
  add_opening(new_vertex, count, F_distance, node_ind);
  add_opening(new_vertex, count, L_distance, (node_ind + 3) % 4);
  add_opening(new_vertex, count, R_distance, (node_ind + 1) % 4);

  return 0;
}

// Map global yaw (degrees) to Neighbours slot: 0=N(0/360), 1=E(90), 2=S(180), 3=W(270).
// Uses Angle_Threshold deg tolerance. Unknown angles default to 0 (north).
int ReturnProperIndex(float angle) {
  int return_ind;
  if (std::fabs(std::fmod(angle, 360.0f)) < Angle_Threshold) {
    return_ind = 0;
  } else if (std::fabs(std::fmod(angle + 270.0f, 360.0f)) < Angle_Threshold) {
    return_ind = 1;
  } else if (std::fabs(std::fmod(angle + 180.0f, 360.0f)) < Angle_Threshold) {
    return_ind = 2;
  } else if (std::fabs(std::fmod(angle + 90.0f, 360.0f)) < Angle_Threshold) {
    return_ind = 3;
  } else {
    return_ind = 0;
  }
  return return_ind;
}

// Neighbours index -> compass heading in degrees (0, 90, 180, 270).
float ReturnProperAngleFromIndex(int ind) {
  return ((float)(ind % 4)) * 90;
}

int NeighborIndexOf(pVertex from, pVertex to) {
  if (!from || !to)
    return -1;
  for (int i = 0; i < 4; i++) {
    if (from->Neighbours[i] == to)
      return i;
  }
  return -1;
}
