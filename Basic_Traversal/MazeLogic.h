#include <stdint.h>
#ifndef MAZELOGIC_H
#define MAZELOGIC_H

#include <Arduino.h>
#include "M_CONSTANTS.h"

#define GREEN 0
#define BLUE 1
#define RED 2

#define Angle_Threshold 10


// Each Vertex in tree/Maze
typedef struct vertex {
  uint32_t Key; // Identifier
  uint8_t Colour; // identifies if this vertex has been visited in searching
  struct vertex *Neighbours[4]; // Adjacency list (doesn't need to be a LL because MAX 4 neighbours)
} Vertex, Vx, *pVertex;

// tree with all nodes in Maze
typedef struct tree {
  pVertex Root; 
} Tree, *pTree;

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


// Creates a tree
Tree* createTree();

// Creates a Vertex
int CreateVertex(pVertex new_vertex, float target_angle, float L_distance, float F_distance, float R_distance);

// certain angles will be represented based on position in Neighbours[4] array
int ReturnProperIndex(float angle);








#endif