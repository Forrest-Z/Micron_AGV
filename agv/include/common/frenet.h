/** frenet.h
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Construction of frenet coordinates
 * Conversion between Frenet frame and Cartesian frame
 */

#ifndef FRENET_H_
#define FRENET_H_

#include <cmath>
#include <vector>
#include <iostream>

#include "lane.h"
#include "math_utils.h"
#include "vehicle_state.h"

namespace agv
{
namespace common
{

class FrenetState
{
 public:
  // Constructor
  FrenetState(){};
  // Destructor
  virtual ~FrenetState(){};

  double s;
  double s_d;
  double s_dd;
  double s_ddd;
  double d;
  double d_d;
  double d_dd;
  double d_ddd;
};

class FrenetPath
{
 public:
  // Constructor
  FrenetPath(){};
  // Destructor
  virtual ~FrenetPath(){};

  int lane_id;
  // time list
  std::vector<double> t;
  // longitudinal
  std::vector<double> s;
  std::vector<double> s_d;
  std::vector<double> s_dd;
  std::vector<double> s_ddd;
  // lateral
  std::vector<double> d;
  std::vector<double> d_d;
  std::vector<double> d_dd;
  std::vector<double> d_ddd;
  // state
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> ds;
  std::vector<double> c;
  // costs
  double cd = 0.0;
  double cs = 0.0;
  double cf = 0.0;
};

// Convert the position in Cartesian coordinates to Frenet frame
FrenetState getFrenet(VehicleState current_state, const Map& map);
FrenetState getFrenet(VehicleState current_state, const Path& path);

}  // end of namespace common
}  // end of namespace agv

#endif  // FRENET_H_