#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

#define DRIFT_rZ 0.01
#define rZ_DEADZONE 0.001
#define FAIL -1
#define SUCCESS 1


typedef enum stage {
  UNSURE = -1,
  START,
  FOLLOW_HALLWAY,
  ENTER_NODE_CENTRE,
  DETERMINE_NEXT_STEP,
  ROTATE_TO_DESTINATION,
  EXIT_NODE_CENTRE,
  FOUND_EXIT,
  TEST
} STAGE, *pSTAGE;




// MPU YAW UPDATE return if fail
int UpdateYAW(float Gz, float *zR, float *oR, float Dt);





#endif