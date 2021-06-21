#include "behaviour_planner_helper.h"

namespace agv
{
namespace behaviour_planner
{
  
obstacle_detector::Obstacles feedObstaclesMsg(const std::vector<agv::common::CircleObstacle>& obstacles, 
                                              const std::vector<agv::common::CircleObstacle>& obstacles2, 
                                              const std::vector<agv::common::CircleObstacle>& obstacles3, 
                                              const int quadrant)
{
  obstacle_detector::Obstacles obstacle_msg;

  obstacle_msg.header.frame_id = "map";  // display on map frame

  for (auto obstacle : obstacles)
  {
    if (obstacle.quadrant == quadrant) // && obstacle.is_static == is_static)
    {
      obstacle_detector::CircleObstacle circle;

      circle.center.x = obstacle.state.x;
      circle.center.y = obstacle.state.y;
      circle.true_radius = obstacle.true_radius;
      circle.velocity.x = obstacle.vx;
      circle.velocity.y = obstacle.vy;

      obstacle_msg.circles.emplace_back(circle);
    }
  }

  for (auto obstacle : obstacles2)
  {
    if (obstacle.quadrant == quadrant) // && obstacle.is_static == is_static)
    {
      obstacle_detector::CircleObstacle circle;

      circle.center.x = obstacle.state.x;
      circle.center.y = obstacle.state.y;
      circle.true_radius = obstacle.true_radius;
      circle.velocity.x = obstacle.vx;
      circle.velocity.y = obstacle.vy;

      obstacle_msg.circles.emplace_back(circle);
    }
  }

  for (auto obstacle : obstacles3)
  {
    if (obstacle.quadrant == quadrant) // && obstacle.is_static == is_static)
    {
      obstacle_detector::CircleObstacle circle;

      circle.center.x = obstacle.state.x;
      circle.center.y = obstacle.state.y;
      circle.true_radius = obstacle.true_radius;
      circle.velocity.x = obstacle.vx;
      circle.velocity.y = obstacle.vy;

      obstacle_msg.circles.emplace_back(circle);
    }
  }

  return obstacle_msg;
}

}  // namespace behaviour_planner
}  // namespace agv