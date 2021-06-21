#include "collision_detector_helper.h"

namespace agv
{
namespace behaviour_planner
{
// project future path in the next <LOOK_AHEAD_TIME> in <DELTA_T> time increments
agv::common::Path projectFuturePath(const agv::common::VehicleState& state, const double steering_angle,
                                    const double SPEED_THRESHOLD, const double LOOK_AHEAD_TIME, const double DELTA_T)
{
  agv::common::VehicleState predicted_state = state;
  predicted_state.v = std::max(state.v, SPEED_THRESHOLD);
  agv::common::ActuatorState actuator(steering_angle, 0);
  agv::common::Path predicted_path;

  predicted_path.clear();

  for (double t = 0; t <= LOOK_AHEAD_TIME; t += DELTA_T)
  {
    agv::common::VehicleState next_state = getNextKinematicModelState(predicted_state, actuator, DELTA_T);

    predicted_path.x.push_back(next_state.x);
    predicted_path.y.push_back(next_state.y);
    predicted_path.yaw.push_back(next_state.yaw);

    predicted_state = next_state;
  }

  return predicted_path;
}

// Get the next state vector of the bicycle vehicle kinematic model
agv::common::VehicleState getNextKinematicModelState(const agv::common::VehicleState& state,
                                                     const agv::common::ActuatorState& actuator, const double DELTA_T)
{
  // Create a new vector for the next state.
  agv::common::VehicleState next_state;

  // state vector [x, y, psi, v]
  double x = state.x;
  double y = state.y;
  double psi = state.yaw;
  double v = state.v;

  // actuators inputs [delta, a]
  double delta = actuator.delta;
  double a = actuator.a;

  /** Equations for the bicycle vehicle kinematic model:
   * x_[t+1] = x[t] + v[t] * cos(psi[t]) * DELTA_T
   * y_[t+1] = y[t] + v[t] * sin(psi[t]) * DELTA_T
   * psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * DELTA_T
   * v_[t+1] = v[t] + a[t] * DELTA_T
   */
  next_state.x = x + v * cos(psi) * DELTA_T;
  next_state.y = y + v * sin(psi) * DELTA_T;
  next_state.yaw = psi + v / agv::common::Vehicle::Lf() * delta * DELTA_T;
  next_state.v = v + a * DELTA_T;

  return next_state;
}

// check for collision given 2 different obstacles
bool checkForCollision(const double obstacle_dist, const double obstacle2_dist, const double safety_margin)
{
  if (obstacle_dist <= safety_margin || obstacle2_dist <= safety_margin)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// check for collision given 1 obstacle
bool checkForCollision(const double obstacle_dist, const double safety_margin)
{
  if (obstacle_dist <= safety_margin)
  {
    return true;
  }
  else
  {
    return false;
  }
}

}  // namespace behaviour_planner
}  // namespace agv