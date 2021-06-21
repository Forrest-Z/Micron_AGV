/** obstacle.h
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Obstacle class
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "obstacle_detector/CircleObstacle.h"

#include "frenet.h"
#include "vehicle_state.h"

namespace agv
{
namespace common
{

class CircleObstacle
{
 public:
  // Constructors
  CircleObstacle() {};
  CircleObstacle(const obstacle_detector::CircleObstacle& obstacle_msg);
  // Destructor
  virtual ~CircleObstacle(){};

  int class_id;
  VehicleState state;
  FrenetState frenet_state;

  double vx;
  double vy;
  double radius;
  double true_radius;

  int lane_id;
  int quadrant;
  bool is_static;
};

} // namespace common
} // namespace agv

#endif // OBSTACLE_H_