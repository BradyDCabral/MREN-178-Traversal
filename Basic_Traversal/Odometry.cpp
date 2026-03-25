#include <math.h>
#include <numbers>
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

  float D_L = rpsL * Dt * 2 * (std::numbers::pi) * WHEEL_RADIUS;
  float D_R = rpsR * Dt * 2 * (std::numbers::pi) * WHEEL_RADIUS;

  // update prev values encoder do that manually

  // get delta theta 
  float D_theta = (D_L-D_R)/(Sl + Sr);

  float D_Lengths = D_L - D_R;

  if ((D_Lengths < Equal_Threshold) && (D_Lengths > -Equal_Threshold)) {
    *y += D_R;
  } else {
    

    // get local y offset
    float temp_y = 2 * sin((double)(D_theta/2)) * (D_R/D_theta + Sr);

    float theta_m = *theta + (D_theta/2);

    // to polar 
    float p_r = temp_y;
    float p_theta = (std::numbers::pi)/2;

    // rotate polar
    p_theta -= theta_m;

    // to cartestian
    float temp_x = p_r * cos((double)p_theta);
    temp_y = p_r * sin((double)p_theta);

    *x += temp_x;
    *y += temp_y;



    // update absolute theta might need to check sign
    *theta += D_theta;
  }

  return SUCCESS;
}