#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "math_utils.h"

namespace agv {
namespace common {

class Vehicle
{
 public:
  static double length();
  static double width();
  static double L();
  static double Lf();
  static double Lr();

  static double max_steering_angle();
  static double max_speed();
  static double max_acceleration();
  static double max_deceleration();
  static double max_curvature();

 private:
};

} // namespace common
} // namespace agv

#endif //VEHICLE_H_