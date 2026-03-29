#ifndef MAZELOGIC_H
#define MAZELOGIC_H

#include <Arduino.h>
#include "M_CONSTANTS.h"
#include <stdint.h>

#define GREEN 0
#define BLUE 1
#define RED 2

#define Angle_Threshold 10

// Max vertices in the maze graph; A* uses Key in 1..MAX_NODES
#define MAX_NODES 30
#define INF 9999.0f

// Use until you assign a cell when mapping the explored maze to a grid (for A* heuristic)
#define VERTEX_GRID_UNSET ((int16_t)-1)

// Neighbours[i]: edge in direction i — see ReturnProperIndex / ReturnProperAngleFromIndex (0=N, 1=E, 2=S, 3=W style)
// Each Vertex in graph/Maze
typedef struct vertex {
  uint32_t Key; // Identifier; must stay in 1..MAX_NODES for AStar_Search
  uint8_t Colour; // identifies if this vertex has been visited in searching
  struct vertex *Neighbours[4]; // NULL = no wall passage / no neighbour
  int16_t row; // grid row for A* heuristic (Manhattan); VERTEX_GRID_UNSET if unknown
  int16_t col; // grid column
} Vertex, Vx, *pVertex;

// graph with all nodes in Maze
typedef struct graph {
  pVertex Root; 
} Graph, *pGraph;

// angle Stack element
typedef struct angle_element {
  struct angle_element *Next;
  float Angle;
} Angle_E, *pAngle_E;

// Stack for angles to return
typedef struct angle_stack {
  pAngle_E Head;
} Angle_Stack, *pAngle_Stack;

// Elements in Frontier
typedef struct future_v_element {
  pVertex Destination;
  Angle_Stack Return_Angles;
} Future_V_Element, *pFuture_V_Element;


// Stack for Frontier
typedef struct stack_future_v {
  pFuture_V_Element Head;
} Stack_Future_V, *pStack_Future_V;


// Creates a graph
Graph* createGraph();


// Creates empty green Vertex
int CreateEmptyVertex(pVertex *new_vertex, int *count);

// Creates a Vertex
int CreateVertex(pVertex new_vertex, float target_angle, float L_distance, float F_distance, float R_distance, int *count);

// certain angles will be represented based on position in Neighbours[4] array
int ReturnProperIndex(float angle);

// reverse of above function
float ReturnProperAngleFromIndex(int ind);

// Neighbour slot of `to` as seen from `from`, or -1 if not adjacent
int NeighborIndexOf(pVertex from, pVertex to);








#endif