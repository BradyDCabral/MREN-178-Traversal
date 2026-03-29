#include <cmath>
#include "Odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int UpdateYAW(float Gz, float *zR, float *oR, float Dt) {
  Gz += ((std::abs)(Gz) > rZ_DEADZONE ? 0 : DRIFT_rZ);
  float dz = (*oR + Gz) / 2.0f * Dt;

  *zR += (dz > -rZ_DEADZONE && dz < rZ_DEADZONE ? 0 : dz);

  *oR = Gz;
  return SUCCESS;
}

int Wheel_Tracking(float rpsL, float rpsR, float *theta, float *x, float *y, bool frwrdL, bool frwrdR, float Dt) {
  if (!theta || !x || !y || Dt <= 0.0f)
    return FAIL;

  float d_l = rpsL * Dt * 2.0f * (float)M_PI * WHEEL_RADIUS;
  float d_r = rpsR * Dt * 2.0f * (float)M_PI * WHEEL_RADIUS;
  if (!frwrdL)
    d_l = -d_l;
  if (!frwrdR)
    d_r = -d_r;

  float track = Sl + Sr;
  if (track <= 0.0f)
    return FAIL;

  float d_theta = (d_l - d_r) / track;
  float dist = 0.5f * (d_l + d_r);
  float th = *theta;
  float th_mid = th + 0.5f * d_theta;

  *x += dist * cosf(th_mid);
  *y += dist * sinf(th_mid);
  *theta += d_theta;
  return SUCCESS;
}
