/** lane.h
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Classes and functions related to lanes
 */

#ifndef LANE_H_
#define LANE_H_

#include <vector>

#include <agv/LaneInfo.h>

#include "math_utils.h"
#include "vehicle_state.h"

namespace agv
{
namespace common
{
  
class Map
{
 public:
  // constructors
  Map(){};
  Map(const agv::LaneInfo::ConstPtr& lane_info);
  // Destructor
  virtual ~Map(){};

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> dx;
  std::vector<double> dy;
  std::vector<double> s;
  std::vector<double> left_widths;
  std::vector<double> right_widths;
  std::vector<double> far_right_widths;
  std::vector<int> special_points;

  // Clear all contents
  void clear();
};

class Path
{
 public:
  // Constructor
  Path(){};
  // Destructor
  virtual ~Path(){};

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;

  // Clear all contents
  void clear();
};

// Find the ID of the closest waypoint wrt current x, y position
int closestWaypoint(VehicleState current_state, const Path& path);
int closestWaypoint(VehicleState current_state, const Map& map);

// Find the ID of the next waypoint of the closest waypoint wrt current x, y position
int nextWaypoint(VehicleState current_state, const Path& path);
int nextWaypoint(VehicleState current_state, const Map& map);

// Find the ID of the previous waypoint
int lastWaypoint(VehicleState current_state, const Path& path);
int lastWaypoint(VehicleState current_state, const Map& map);

}  // end of namespace common
}  // end of namespace agv

#endif // LANE_H_