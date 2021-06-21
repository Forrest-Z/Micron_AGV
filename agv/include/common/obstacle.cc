/** obstacle.cc
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Obstacle class
 */

#include "obstacle.h"

namespace agv
{
namespace common
{

CircleObstacle::CircleObstacle(const obstacle_detector::CircleObstacle& obstacle_msg)
{
  this->state.x = obstacle_msg.center.x;
  this->state.y = obstacle_msg.center.y;
  this->state.yaw = atan2(obstacle_msg.velocity.y, obstacle_msg.velocity.x);
  this->state.v = hypot(obstacle_msg.velocity.x, obstacle_msg.velocity.y);

  this->vx = obstacle_msg.velocity.x;
  this->vy = obstacle_msg.velocity.y;
  this->radius = obstacle_msg.radius;
  this->true_radius = obstacle_msg.true_radius;
}

} // namespace common
} // namespace agv