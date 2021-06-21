#include "vehicle.h"

namespace agv
{
namespace common
{

  double Vehicle::length() { return 3.3; };
  double Vehicle::width() { return 1.4; };
  double Vehicle::L() { return 2.2; };
  double Vehicle::Lf() { return 1.1; };
  double Vehicle::Lr() { return 1.1; };

  double Vehicle::max_steering_angle() { return deg2rad(20); };
  double Vehicle::max_speed() { return 3.33; };
  double Vehicle::max_acceleration() { return 1.0; };
  double Vehicle::max_deceleration() { return -1.0; };
  double Vehicle::max_curvature() { return 1.0; };
  
} // namespace common
} // namespace agv