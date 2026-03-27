#include <stdint.h>
#ifndef MAZELOGIC_H
#define MAZELOGIC_H

#include <Arduino.h>

#define GREEN 0
#define BLUE 1
#define RED 2


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

// angle Stace element
typedef struct angle_element {
  struct angle_element *Next;
  float Angle;
} Angle_E, pAngle_E;

// Stack for angles to return
typedef struct angle_stack {
  pAngle_E Head;
}


// Elements in Frontier
typedef struct future_v_element {
  pVertex Destination;

}


// Stack for Frontier
// typedef struct Stack_


// Creates a tree
Tree* createTree();











#endif