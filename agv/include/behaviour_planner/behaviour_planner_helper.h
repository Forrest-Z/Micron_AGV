/** behaviour_planner_helper.h
 * 
 *  Copyright (C) 2019 Brina & Haoren & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 */

#ifndef BEHAVIOUR_PLANNER_HELPER_H_
#define BEHAVIOUR_PLANNER_HELPER_H_

#include <obstacle_detector/Obstacles.h>

#include "common/frenet.h"
#include "common/obstacle.h"

namespace agv
{
namespace behaviour_planner
{
obstacle_detector::Obstacles feedObstaclesMsg(const std::vector<agv::common::CircleObstacle>& obstacles, 
                                              const std::vector<agv::common::CircleObstacle>& obstacles2, 
                                              const std::vector<agv::common::CircleObstacle>& obstacles3, 
                                              const int quadrant);

}  // namespace behaviour_planner
}  // namespace agv

#endif  // BEHAVIOUR_PLANNER_HELPER_H_