/** motion_model.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Helper functions related to vehicle motion model
 */

#include "motion_model.h"

namespace agv
{
namespace common
{

// Get the next state std::vector of the bicycle vehicle kinematic model
VehicleState getNextKinematicModelState(const VehicleState &state, const ActuatorState &actuators, 
                                        double Lf, double dt)
{
  // state [x, y, psi, v]
  const double x = state.x;
  const double y = state.x;
  const double psi = state.yaw;
  const double v = state.v;

  // actuators inputs [delta, a]
  const double delta = actuators.delta;
  const double a = actuators.a;

  /** Equations for the bicycle vehicle kinematic model:
   * x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
   * y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
   * psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
   * v_[t+1] = v[t] + a[t] * dt
   */

  VehicleState next_state;
  next_state.x = x + v * cos(psi) * dt;
  next_state.y = y + v * sin(psi) * dt;
  next_state.yaw = psi + v / Lf * delta * dt;
  next_state.v = v + a * dt;

  return next_state;
}

} // end of namespace common
} // end of namespace agv