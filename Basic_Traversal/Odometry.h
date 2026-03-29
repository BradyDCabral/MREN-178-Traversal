#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

#define DRIFT_rZ 0.01
#define rZ_DEADZONE 0.001

#define Equal_Threshold 0.01

#define FAIL -1
#define SUCCESS 1


// Wheel odometry information
#define WHEEL_RADIUS 0.1 // in (m)
#define Sl 0.1 // perpendicular distance of wheel from centre for left wheel
#define Sr 0.1 // perpendicular distance of wheel from centre for right wheel


typedef enum stage {
  UNSURE = -1,
  START, // set at start of program run
  FOLLOW_HALLWAY, // moves straight through hallways
  ENTER_NODE_CENTRE, // move to centre of node (centre of fork in road)
  DETERMINE_NEXT_STEP, // does calculation using matrix to figure out next move
  ROTATE_TO_DESTINATION, // rotates to desired angle
  EXIT_NODE_CENTRE, // move forward till walls reached
  FOUND_EXIT, // stop moving as magnet detected
  TEST // just for testing 
} STAGE, *pSTAGE;




// MPU YAW UPDATE return if fail
int UpdateYAW(float Gz, float *zR, float *oR, float Dt);


// Differential-drive odometry: *theta is radians (robot heading in world XY); *x,*y meters.
// Track width = Sl + Sr. Arc length = rps * 2*pi*r * dt.
int Wheel_Tracking(float rpsL, float rpsR, float *theta, float *x, float *y, bool frwrd, float Dt);

#endif