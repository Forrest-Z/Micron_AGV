/** collision_detector_helper.h
 * 
 * Copyright (C) 2019 Brina & Haoren & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 */

#ifndef COLLISION_DETECTOR_HELPER_H_
#define COLLISION_DETECTOR_HELPER_H_

#include "common/vehicle_state.h"
#include "common/vehicle.h"
#include "common/lane.h"
#include "common/obstacle.h"
#include "common/math_utils.h"

namespace agv
{
namespace behaviour_planner
{
agv::common::Path projectFuturePath(const agv::common::VehicleState& state, const double steering_angle,
                                    const double SPEED_THRESHOLD, const double LOOK_AHEAD_TIME, const double DELTA_T);
agv::common::VehicleState getNextKinematicModelState(const agv::common::VehicleState& state,
                                                     const agv::common::ActuatorState& actuator, const double DELTA_T);
bool checkForCollision(const double obstacle_dist, const double obstacle2_dist, const double safety_margin);
bool checkForCollision(const double obstacle_dist, const double safety_margin);
}  // namespace behaviour_planner
}  // namespace agv

#endif  // COLLISION_DETECTOR_HELPER_H_