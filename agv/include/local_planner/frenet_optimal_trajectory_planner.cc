/* frenet_optimal_trajectory_planner.cc

    Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

    Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
    Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/

#include "frenet_optimal_trajectory_planner.h"

namespace agv
{
namespace local_planner
{

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner(FrenetOptimalTrajectoryPlanner::Setting settings)
{
  this->settings_ = settings;
}

FrenetOptimalTrajectoryPlanner::ResultType FrenetOptimalTrajectoryPlanner::generateReferenceCurve(const agv::common::Map& map)
{
  FrenetOptimalTrajectoryPlanner::ResultType result = FrenetOptimalTrajectoryPlanner::ResultType();
  result.cubic_spline = agv::common::Spline2D(map);

  std::vector<double> s;
  for (double i = 0; i < result.cubic_spline.s_.back(); i += 0.1)
  {
    s.push_back(i);
  }

  for (int i = 0; i < s.size(); i++)
  {
    agv::common::VehicleState state = result.cubic_spline.calculatePosition(s.at(i));
    result.rx.push_back(state.x);
    result.ry.push_back(state.y);
    result.ryaw.push_back(result.cubic_spline.calculateYaw(s.at(i)));
    result.rk.push_back(result.cubic_spline.calculateCurvature(s.at(i)));
  }

  return result;
}

std::vector<agv::common::FrenetPath> FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(agv::common::Spline2D& cubic_spline, const agv::common::FrenetState& frenet_state, 
                                                                              double center_offset, double left_width, double right_width,
                                                                              const std::vector<agv::common::CircleObstacle>& obstacles, const std::vector<agv::common::CircleObstacle>& obstacles_2)
{
  // Sample a list of FrenetPaths
  std::vector<agv::common::FrenetPath> frenet_paths_list = generateFrenetPaths(frenet_state, center_offset, left_width, right_width);
  std::cout << "Total Paths Generated: " << frenet_paths_list.size() << std::endl;

  // Convert to global paths
  frenet_paths_list = calculateGlobalPaths(frenet_paths_list, cubic_spline);
  std::cout << "Paths Converted to Global Frame: " << frenet_paths_list.size() << std::endl;

  // Check the constraints
  frenet_paths_list = checkPaths(frenet_paths_list, obstacles, obstacles_2);
  std::cout << "Paths Passed Collision Check: " << frenet_paths_list.size() << std::endl;

  // Find the path with minimum costs
  std::vector<agv::common::FrenetPath> best_path_list = findBestPaths(frenet_paths_list);

  return best_path_list;
}

std::vector<agv::common::FrenetPath> FrenetOptimalTrajectoryPlanner::generateFrenetPaths(const agv::common::FrenetState& frenet_state, 
                                                                                        double center_offset, double left_bound, double right_bound)
{
  // list of frenet paths generated
  std::vector<agv::common::FrenetPath> frenet_paths;
  std::vector<double> goal_ds;

  // generate different goals with a lateral offset
  for (double d = 0.0 + center_offset; d <= left_bound; d += settings_.delta_width)  // left being positive
  {
    goal_ds.push_back(d);
  }
  for (double d = 0.0 + center_offset - settings_.delta_width; d >= right_bound; d -= settings_.delta_width)  // right being negative
  {
    goal_ds.push_back(d);
  }

  // for (double goal_d = right_bound; goal_d <= left_bound; goal_d += settings_.delta_width)
  for (double goal_d : goal_ds)
  {
    // generate d_t polynomials
    int t_count = 0;
    for (double T = settings_.min_t; T <= settings_.max_t; T += settings_.delta_t)
    {
      t_count++;
      // std::cout << T << std::endl;
      agv::common::FrenetPath frenet_path = agv::common::FrenetPath();

      // left lane
      if (goal_d >= -left_bound)
      {
        frenet_path.lane_id = 1;
      }
      // transition area
      else if (goal_d >= right_bound + 2 * left_bound)
      {
        frenet_path.lane_id = 0;
      }
      // right lane
      else if (goal_d >= right_bound)
      {
        frenet_path.lane_id = 2;
      }
      // fault lane
      else
      {
        frenet_path.lane_id = -1;
      }

      // start lateral state [d, d_d, d_dd]
      std::vector<double> start_d;
      start_d.push_back(frenet_state.d);
      start_d.push_back(frenet_state.d_d);
      start_d.push_back(frenet_state.d_dd);

      // end lateral state [d, d_d, d_dd]
      std::vector<double> end_d;
      end_d.push_back(goal_d);
      end_d.push_back(0.0);
      end_d.push_back(0.0);

      // generate lateral quintic polynomial
      agv::common::QuinticPolynomial lateral_quintic_poly = agv::common::QuinticPolynomial(start_d, end_d, T);

      // store the this lateral trajectory into frenet_path
      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        frenet_path.t.push_back(t);
        frenet_path.d.push_back(lateral_quintic_poly.calculatePoint(t));
        frenet_path.d_d.push_back(lateral_quintic_poly.calculateFirstDerivative(t));
        frenet_path.d_dd.push_back(lateral_quintic_poly.calculateSecondDerivative(t));
        frenet_path.d_ddd.push_back(lateral_quintic_poly.calculateThirdDerivative(t));
      }

      // generate longitudinal quintic polynomial
      for (double target_speed = settings_.target_speed - settings_.num_speed_sample * settings_.delta_speed; target_speed <= settings_.max_speed;
           target_speed += settings_.delta_speed)  // settings_.target_speed + settings_.num_speed_sample*settings_.delta_speed
      {
        // copy the longitudinal path over
        agv::common::FrenetPath target_frenet_path = frenet_path;

        // start longitudinal state [s, s_d, s_dd]
        std::vector<double> start_s;
        start_s.push_back(frenet_state.s);
        start_s.push_back(frenet_state.s_d);
        start_s.push_back(0.0);

        // end longitudinal state [s_d, s_dd]
        std::vector<double> end_s;
        end_s.push_back(target_speed);
        end_s.push_back(0.0);

        // generate longitudinal quartic polynomial
        agv::common::QuarticPolynomial longitudinal_quartic_poly = agv::common::QuarticPolynomial(start_s, end_s, T);

        // store the this longitudinal trajectory into target_frenet_path
        for (double t = 0.0; t <= T; t += settings_.tick_t)
        {
          target_frenet_path.s.push_back(longitudinal_quartic_poly.calculatePoint(t));
          target_frenet_path.s_d.push_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
          target_frenet_path.s_dd.push_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
          target_frenet_path.s_ddd.push_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
        }

        // calculate the costs
        double speed_diff = 0.0;
        double jerk_s = 0.0;
        double jerk_d = 0.0;

        // encourage driving according to the target speed
        speed_diff = settings_.target_speed - pow(target_frenet_path.s_d.back(), 2);

        // calculate total squared jerks
        for (int i = 0; i < target_frenet_path.t.size(); i++)
        {
          jerk_s += pow(target_frenet_path.s_ddd.at(i), 2);
          jerk_d += pow(target_frenet_path.d_ddd.at(i), 2);
        }

        // encourage longer planning time
        const double planning_time_cost = settings_.k_time * (1 - T / settings_.max_t);

        target_frenet_path.cd =
            settings_.k_jerk * jerk_d + planning_time_cost + settings_.k_diff * pow(target_frenet_path.d.back() - settings_.centre_offset, 2);
        target_frenet_path.cs = settings_.k_jerk * jerk_s + planning_time_cost + settings_.k_diff * speed_diff;
        target_frenet_path.cf = settings_.k_lateral * target_frenet_path.cd + settings_.k_longitudinal * target_frenet_path.cs;

        frenet_paths.push_back(target_frenet_path);
      }
    }
  }

  return frenet_paths;
}

std::vector<agv::common::FrenetPath> FrenetOptimalTrajectoryPlanner::calculateGlobalPaths(std::vector<agv::common::FrenetPath>& frenet_paths_list,
                                                                                          agv::common::Spline2D& cubic_spline)
{
  for (int i = 0; i < frenet_paths_list.size(); i++)
  {
    // std::cout << "Break 1" << std::endl;
    // calculate global positions
    for (int j = 0; j < frenet_paths_list.at(i).s.size(); j++)
    {
      // std::cout << "Break 1.1" << std::endl;
      agv::common::VehicleState state = cubic_spline.calculatePosition(frenet_paths_list.at(i).s.at(j));
      // std::cout << "Break 1.2" << std::endl;
      double i_yaw = cubic_spline.calculateYaw(frenet_paths_list.at(i).s.at(j));
      // std::cout << "Break 1.3" << std::endl;
      double di = frenet_paths_list.at(i).d.at(j);
      double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
      double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
      frenet_paths_list.at(i).x.push_back(frenet_x);
      frenet_paths_list.at(i).y.push_back(frenet_y);
    }
    // calculate yaw and ds
    for (int j = 0; j < frenet_paths_list.at(i).x.size() - 1; j++)
    {
      double dx = frenet_paths_list.at(i).x.at(j + 1) - frenet_paths_list.at(i).x.at(j);
      double dy = frenet_paths_list.at(i).y.at(j + 1) - frenet_paths_list.at(i).y.at(j);
      frenet_paths_list.at(i).yaw.push_back(atan2(dy, dx));
      frenet_paths_list.at(i).ds.push_back(sqrt(dx * dx + dy * dy));
    }

    frenet_paths_list.at(i).yaw.push_back(frenet_paths_list.at(i).yaw.back());
    frenet_paths_list.at(i).ds.push_back(frenet_paths_list.at(i).ds.back());

    // calculate curvature
    for (int j = 0; j < frenet_paths_list.at(i).yaw.size() - 1; j++)
    {
      double yaw_diff = frenet_paths_list.at(i).yaw.at(j + 1) - frenet_paths_list.at(i).yaw.at(j);
      yaw_diff = agv::common::unifyAngleRange(yaw_diff);
      frenet_paths_list.at(i).c.push_back(yaw_diff / frenet_paths_list.at(i).ds.at(j));
    }
  }

  return frenet_paths_list;
}

bool FrenetOptimalTrajectoryPlanner::checkCollision(agv::common::FrenetPath& frenet_path, const std::vector<agv::common::CircleObstacle>& obstacles)
{
  const double square_soft_margin = pow(settings_.soft_safety_margin, 2);
  const double half_width = settings_.vehicle_width / 2;

  for (int i = 0; i < frenet_path.x.size(); i++)
  {
    double dist_to_obstacle_cost = 0.0;

    for (int j = 0; j < obstacles.size(); j++)
    {
      double distance = sqrt(pow(frenet_path.x.at(i) - obstacles.at(j).state.x, 2) + pow(frenet_path.y.at(i) - obstacles.at(j).state.y, 2));
      distance -= half_width + settings_.hard_safety_margin + obstacles.at(j).radius;

      // Check for hard collision constraints
      if (distance <= 0)
      {
        return false;
      }
      // Check for soft collision constraints and add to cost
      else if (distance <= settings_.soft_safety_margin)
      {
        // cost is unified to 0 ~ 1
        double cost = pow(settings_.soft_safety_margin - distance, 2) / square_soft_margin;
        dist_to_obstacle_cost = dist_to_obstacle_cost >= cost ? dist_to_obstacle_cost : cost;
      }
    }

    frenet_path.cf += settings_.k_obstacle * dist_to_obstacle_cost;
    // frenet_path.cf += settings_.k_obstacle * dist_to_obstacle_cost / count;
  }

  return true;
}

std::vector<agv::common::FrenetPath> FrenetOptimalTrajectoryPlanner::checkPaths(const std::vector<agv::common::FrenetPath>& frenet_paths_list,
                                                                                const std::vector<agv::common::CircleObstacle>& obstacles,
                                                                                const std::vector<agv::common::CircleObstacle>& obstacles_2)
{
  std::vector<agv::common::FrenetPath> safe_paths;

  for (auto frenet_path : frenet_paths_list)
  {
    bool safe = true;
    for (int j = 0; j < frenet_path.c.size(); j++)
    {
      if (frenet_path.s_d.at(j) > settings_.max_speed)
      {
        safe = false;
        // std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
        break;
      }
      else if (frenet_path.s_dd.at(j) > settings_.max_accel || frenet_path.s_dd.at(j) < settings_.max_decel)
      {
        safe = false;
        // std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
        break;
      }
      else if (fabs(frenet_path.c.at(j)) > settings_.max_curvature)
      {
        safe = false;
        // std::cout << "Condition 3: Exceeded Max Curvature" << i << " " << frenet_path.c.at(j) << " " <<
        // frenet_path.yaw.at(j) << std::endl;
        break;
      }
      else if (!checkCollision(frenet_path, obstacles))
      {
        safe = false;
        // std::cout << "Condition 4: Collision with obstacles" << std::endl;
        break;
      }
      else if (!checkCollision(frenet_path, obstacles_2))
      {
        safe = false;
        // std::cout << "Condition 5: Condition with obstacles_2" << std::endl;
        break;
      }
    }

    if (safe)
    {
      safe_paths.push_back(frenet_path);
    }
  }
  // if safe path exists
  if (safe_paths.size() > 0)
  {
    return safe_paths;
  }
  // if safe path doesn't exist
  else
  {
    std::vector<agv::common::FrenetPath> dummy_paths;
    return dummy_paths;
  }
}

std::vector<agv::common::FrenetPath> FrenetOptimalTrajectoryPlanner::findBestPaths(const std::vector<agv::common::FrenetPath>& frenet_paths_list)
{
  std::vector<agv::common::FrenetPath> best_path_list;
  best_path_list.push_back(findBestPath(frenet_paths_list, 0));  // transition area
  best_path_list.push_back(findBestPath(frenet_paths_list, 1));  // left lane
  best_path_list.push_back(findBestPath(frenet_paths_list, 2));  // right lane

  return best_path_list;
}

agv::common::FrenetPath FrenetOptimalTrajectoryPlanner::findBestPath(const std::vector<agv::common::FrenetPath>& frenet_paths_list,
                                                                    int target_lane_id)
{
  // if best paths list isn't empty
  if (!frenet_paths_list.empty())
  {
    double min_cost = 1000000000.0;  // start with a large number
    int best_path_id = 0;
    for (int i = 0; i < frenet_paths_list.size(); i++)
    {
      if (frenet_paths_list.at(i).lane_id == target_lane_id)
      {
        if (min_cost >= frenet_paths_list.at(i).cf)
        {
          min_cost = frenet_paths_list.at(i).cf;
          best_path_id = i;
        }
      }
    }
    // std::cout << "Best Path ID: " << best_path_id << std::endl;

    return frenet_paths_list[best_path_id];
  }

  // if best paths list is empty
  else
  {
    // std::cout << "Best Path Is Empty" << std::endl;
    return agv::common::FrenetPath();
  }
}

}  // namespace local_planner
}  // namespace agv
