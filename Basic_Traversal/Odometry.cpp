#include "Odometry.h"








int UpdateYAW(float Gz, float *zR, float *oR, float Dt) {
  // if stationary just dont update YAW
  Gz += ((std::abs)(Gz) > rZ_DEADZONE ? 0 : DRIFT_rZ);
  float dz = (*oR + Gz)/2 * Dt;

  *zR += (dz > -rZ_DEADZONE && dz < rZ_DEADZONE ? 0 : dz);
  
  *oR = Gz;
  return SUCCESS;

}

int Wheel_Tracking(float rpsL, float rpsR, float *theta, float *x, float *y, bool frwrd, float Dt) {
  // fill out later
  return SUCCESS;
}