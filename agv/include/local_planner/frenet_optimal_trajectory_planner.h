/* frenet_optimal_trajectory_planner.h

    Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

    Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
    Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

#ifndef FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_
#define FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_

#include <cmath>
#include <vector>
#include <iostream>

#include "common/frenet.h"
#include "common/math_utils.h"
#include "common/obstacle.h"
#include "common/quintic_polynomial.h"
#include "common/quartic_polynomial.h"
#include "common/spline.h"
#include "common/vehicle_state.h"

namespace agv
{
namespace local_planner
{

class FrenetOptimalTrajectoryPlanner
{
public:
  class Setting
  {
  public:
    Setting(){};
    virtual ~Setting(){};

    // parameters
    double max_speed;       // maximum speed [m/s]
    double max_accel;       // maximum acceleration [m/ss]
    double max_decel;       // maximum deceleration [m/ss]
    double max_curvature;   // maximum curvature [rad/m]

    double centre_offset;   // offset from the center of the lane [m]
    double delta_width;     // road width sampling length [m]

    double max_t;     // max prediction time [m]
    double min_t;     // min prediction time [m]
    double delta_t;   // sampling time increment [s]
    double tick_t;    // time tick [s]

    double target_speed;      // target speed [m/s]
    double delta_speed;       // target speed sampling length [m/s]
    double num_speed_sample;  // sampling number of target speed

    double hard_safety_margin;  // circle radius [m]
    double soft_safety_margin;  // vehicle length [m]
    double vehicle_width;       // vehicle width [m]

    // Cost Weights
    double k_jerk;              // jerk cost weight
    double k_time;              // time cost weight
    double k_diff;              // speed and lateral offset cost weight
    double k_lateral;           // lateral overall cost weight
    double k_longitudinal;      // longitudinal overall cost weight
    double k_obstacle;          // obstacle cost weight
  };

  // Result Data Type
  class ResultType
  {
  public:
    // Constructor
    ResultType(){};
    // Destructor
    virtual ~ResultType(){};

    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> ryaw;
    std::vector<double> rk;
    agv::common::Spline2D cubic_spline;
  };

  // Constructors
  FrenetOptimalTrajectoryPlanner(){};
  FrenetOptimalTrajectoryPlanner(Setting settings);

  // Destructor
  virtual ~FrenetOptimalTrajectoryPlanner(){};

  /* Public Functions */
  // Generate reference curve as the frenet s coordinate
  ResultType generateReferenceCurve(const agv::common::Map& map);

  // Plan for the optimal trajectory
  std::vector<agv::common::FrenetPath> frenetOptimalPlanning(agv::common::Spline2D& cubic_spline, const agv::common::FrenetState& frenet_state, 
                                                            double center_offset, double left_width, double right_width,
                                                            const std::vector<agv::common::CircleObstacle>& obstacles, const std::vector<agv::common::CircleObstacle>& obstacles_2);

private:
  Setting settings_;

  /* Private Functions */
  // Sample candidate trajectories
  std::vector<agv::common::FrenetPath> generateFrenetPaths(const agv::common::FrenetState& frenet_state, 
                                                          double center_offset, double left_bound, double right_bound);

  // Convert paths from frenet frame to gobal map frame
  std::vector<agv::common::FrenetPath> calculateGlobalPaths(std::vector<agv::common::FrenetPath>& frenet_paths_list,
                                                            agv::common::Spline2D& cubic_spline);

  // Check for collisions and calculate obstacle cost
  bool checkCollision(agv::common::FrenetPath& frenet_path, const std::vector<agv::common::CircleObstacle>& obstacles);

  // Check for vehicle kinematic constraints
  std::vector<agv::common::FrenetPath> checkPaths(const std::vector<agv::common::FrenetPath>& frenet_paths_list,
                                                  const std::vector<agv::common::CircleObstacle>& obstacles,
                                                  const std::vector<agv::common::CircleObstacle>& obstacles_2);

  // Select the best paths for each lane option
  std::vector<agv::common::FrenetPath> findBestPaths(const std::vector<agv::common::FrenetPath>& frenet_paths_list);

  // Select the path with the minimum cost
  agv::common::FrenetPath findBestPath(const std::vector<agv::common::FrenetPath>& frenet_paths_list, int target_lane_id);
};

}  // namespace local_planner
}  // namespace agv

#endif  // FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_