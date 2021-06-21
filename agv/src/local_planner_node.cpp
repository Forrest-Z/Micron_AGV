
/* frenet_optimal_trajectory_planner.cpp

    Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

    Local Planner ROS Node
    Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <obstacle_detector/Obstacles.h>
#include <agv/LaneInfo.h>
#include <agv/Behaviour.h>

#include <dynamic_reconfigure/server.h>
#include <agv/local_planner_Config.h>

#include <cmath>
#include <vector>
#include <iostream>

#include "local_planner/frenet_optimal_trajectory_planner.h"
#include "common/frenet.h"
#include "common/vehicle.h"

namespace agv
{
namespace local_planner
{
// FrenetOptimalTrajectoryPlanner settings
FrenetOptimalTrajectoryPlanner::Setting SETTINGS = FrenetOptimalTrajectoryPlanner::Setting();

// Vehicle Parameters
const double L = agv::common::Vehicle::L();                                       // Wheelbase length, back wheel to front wheel
const double MAX_STEERING_ANGLE = agv::common::Vehicle::max_steering_angle();     // Maximum steering angle
// Constants values used as thresholds (Not for tuning)
const double WP_MAX_SEP = 3.0;                  // Maximum allowable waypoint separation
const double WP_MIN_SEP = 0.01;                 // Minimum allowable waypoint separation
const double HEADING_DIFF_THRESH = M_PI / 2;    // Maximum allowed heading diff between vehicle and path
const double DISTANCE_THRESH = 20.0;            // Maximum allowed distance between vehicle and path
const double MIN_PLANNING_SPEED = 1.0;          // Minimum allowed vehicle speed for planning
const int NUM_LOOK_AHEAD_WP = 1;                // Number of waypoints to look ahead for Stanley
const double ERROR_VALUE = -9999.99;            // Error value for return

/* List of dynamic parameters */
// Hyperparameters for output path
double OUTPUT_PATH_MAX_SIZE;  // Maximum size of the output path
double OUTPUT_PATH_MIN_SIZE;  // Minimum size of the output path
// Stanley gains
double STANLEY_OVERALL_GAIN;  // Stanley overall gain
double TRACK_ERROR_GAIN;      // Cross track error gain
// Turn signal thresholds
double TURN_YAW_THRESH;       // Yaw difference threshold
// Safety margins for collision check
double LEFT_LANE_WIDTH;       // Maximum left road width [m]
double RIGHT_LANE_WIDTH;      // Maximum right road width [m]

// Class for lane id
enum LaneID
{
  BOTH_LANES,
  LEFT_LANE,
  RIGHT_LANE
};

// Dynamic parameter server callback function
void dynamicParamCallback(agv::local_planner_Config& config, uint32_t level)
{
  // Hyperparameters for output path
  OUTPUT_PATH_MAX_SIZE = config.output_path_max_size;
  OUTPUT_PATH_MIN_SIZE = config.output_path_min_size;
  // Safety constraints
  SETTINGS.vehicle_width = agv::common::Vehicle::width();
  SETTINGS.hard_safety_margin = config.hard_safety_margin;
  SETTINGS.soft_safety_margin = config.soft_safety_margin;
  // Stanley gains
  STANLEY_OVERALL_GAIN = config.stanley_overall_gain;
  TRACK_ERROR_GAIN = config.track_error_gain;
  // Sampling parameters (lateral)
  LEFT_LANE_WIDTH = config.left_lane_width;
  RIGHT_LANE_WIDTH = config.right_lane_width;
  SETTINGS.centre_offset = config.center_offset;
  SETTINGS.delta_width = config.delta_width;
  // Sampling parameters (longitudinal)
  SETTINGS.max_t = config.max_t;
  SETTINGS.min_t = config.min_t;
  SETTINGS.delta_t = config.delta_t;
  SETTINGS.tick_t = config.tick_t;
  SETTINGS.target_speed = config.target_speed;
  SETTINGS.delta_speed = config.delta_speed;
  SETTINGS.num_speed_sample = config.num_speed_sample;
  // Constraints
  SETTINGS.max_speed = agv::common::Vehicle::max_speed();
  SETTINGS.max_accel = agv::common::Vehicle::max_acceleration();
  SETTINGS.max_decel = agv::common::Vehicle::max_deceleration();
  SETTINGS.max_curvature = agv::common::Vehicle::max_curvature();
  // Cost Weights
  SETTINGS.k_jerk = config.k_jerk;
  SETTINGS.k_diff = config.k_time;
  SETTINGS.k_diff = config.k_diff;
  SETTINGS.k_lateral = config.k_lateral;
  SETTINGS.k_longitudinal = config.k_longitudinal;
  SETTINGS.k_obstacle = config.k_obstacle;
  // Turn signal thresholds
  TURN_YAW_THRESH = config.turn_yaw_thresh;
}

class LocalPlannerNode
{
public:
  // Constructor
  LocalPlannerNode();

  // Destructor
  virtual ~LocalPlannerNode() {};

private:
  // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Private Variables $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

  ros::NodeHandle nh;

  // Vehicle's current state
  agv::common::VehicleState current_state_;       // State of the vehicle baselink
  agv::common::VehicleState frontaxle_state_;     // State of the vehicle frontaxle
  agv::common::FrenetState start_state_;          // Starting States for sampling

  // obstacles: 2d array like: [x,y,radius,vel_x,vel_y]
  std::vector<agv::common::CircleObstacle> obstacles;     // from /obstacles topic
  std::vector<agv::common::CircleObstacle> obstacles_2;   // from /obstacles2 topic

  // Maps and Paths
  agv::common::Map map_;            // Maps (All the waypoints)
  agv::common::Map local_map_;      // Selected Waypoints
  agv::common::Path ref_spline_;    // Reference Spline
  agv::common::Path output_path_;   // Output Path

  // Regnerate path flag
  bool regenerate_flag_;

  // Lane related variables
  int current_lane_;
  int target_lane_;
  std::vector<double> roi_boundaries_;  //[0] = left boundary length in metre, [1] = right boundary length in metre. roi = region of interest
  int turn_signal_;                     // turn indicator signal, 1 = turn left, -1 = turn right, 0 = not turning

  // output steering angle
  double steering_angle_;

  // Instantiate the Frenet compute object
  FrenetOptimalTrajectoryPlanner frenet_planner_instance;

  // subscriber and publishers
  ros::Subscriber odom_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber obstacle_2_sub;
  ros::Subscriber lane_info_sub;
  ros::Subscriber behaviour_sub;
  ros::Subscriber cmd_sub;

  ros::Publisher output_path_pub;
  ros::Publisher next_path_pub;
  ros::Publisher ref_path_pub;
  ros::Publisher steering_angle_pub;
  ros::Publisher turn_signal_pub;

  // timer
  ros::Timer timer;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  dynamic_reconfigure::Server<agv::local_planner_Config> server;
  dynamic_reconfigure::Server<agv::local_planner_Config>::CallbackType f;

  // ###################################### Private Functions ######################################

  // Main Function in ROS running primary logics
  void mainTimerCallback(const ros::TimerEvent& timer_event);

  // Functions for subscribing
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void laneInfoCallback(const agv::LaneInfo::ConstPtr& lane_info);
  void behaviourCallback(const agv::Behaviour::ConstPtr& behaviour_msg);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  // Functions fo publishing results
  void publishRefSpline(const agv::common::Path& path);
  void publishOutputPath(const agv::common::Path& path);
  void publishNextPath(const agv::common::FrenetPath& frenet_path);
  void publishEmptyPaths();
  void publishSteeringAngle(const double angle);
  void publishTurnSignal(const agv::common::FrenetPath& best_path, const bool change_lane, const double yaw_thresh);

  // Odom Helper Function
  void updateVehicleFrontAxleState();

  // Planner Helper Functions
  bool feedWaypoints();
  void updateStartState();
  std::vector<double> getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width,
                                                     const double left_lane_width, const double right_lane_width);

  agv::common::FrenetPath selectLane(const std::vector<agv::common::FrenetPath>& best_path_list, const int current_lane);

  void concatPath(const agv::common::FrenetPath& frenet_path, const int path_size, 
                  const double wp_max_seperation, const double wp_min_seperation);

  // Stanley Steeing Functions
  double calculateSteeringAngle(const int next_wp_id, const agv::common::VehicleState& frontaxle_state);
};

// Constructor
LocalPlannerNode::LocalPlannerNode() : tf_listener(tf_buffer)
{
  // Initializing states
  regenerate_flag_ = false;
  turn_signal_ = 0;
  target_lane_ = 0;
  
  ros::NodeHandle private_nh("~");

  // topics
  std::string odom_topic_;
  std::string obstacle_topic_;
  std::string obstacle_topic_2_;
  std::string lane_info_topic_;
  std::string behaviour_topic_;
  std::string cmd_topic_;

  std::string output_path_topic_;
  std::string next_path_topic_;
  std::string ref_path_topic_;
  std::string steering_angle_topic_;
  std::string turn_signal_topic_;

  // Hyperparameters
  double planning_frequency_;

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Parameters from launch file: topic names
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle_topic_2", obstacle_topic_2_));
  ROS_ASSERT(private_nh.getParam("lane_info_topic", lane_info_topic_));
  ROS_ASSERT(private_nh.getParam("behaviour_topic", behaviour_topic_));
  ROS_ASSERT(private_nh.getParam("cmd_topic", cmd_topic_));

  ROS_ASSERT(private_nh.getParam("output_path_topic", output_path_topic_));
  ROS_ASSERT(private_nh.getParam("next_path_topic", next_path_topic_));
  ROS_ASSERT(private_nh.getParam("ref_path_topic", ref_path_topic_));
  ROS_ASSERT(private_nh.getParam("steering_angle_topic", steering_angle_topic_));
  ROS_ASSERT(private_nh.getParam("turn_signal_topic", turn_signal_topic_));

  // Hyperparameters
  ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));

  // Instantiate FrenetOptimalTrajectoryPlanner
  frenet_planner_instance = FrenetOptimalTrajectoryPlanner(SETTINGS);

  // Subscribe & Advertise
  odom_sub = nh.subscribe(odom_topic_, 1, &LocalPlannerNode::odomCallback, this);
  obstacle_sub = nh.subscribe(obstacle_topic_, 1, &LocalPlannerNode::obstacleCallback, this);
  obstacle_2_sub = nh.subscribe(obstacle_topic_2_, 1, &LocalPlannerNode::obstacle2Callback, this);
  lane_info_sub = nh.subscribe(lane_info_topic_, 1, &LocalPlannerNode::laneInfoCallback, this);
  behaviour_sub = nh.subscribe(behaviour_topic_, 1, &LocalPlannerNode::behaviourCallback, this);
  cmd_sub = nh.subscribe(cmd_topic_, 1, &LocalPlannerNode::cmdCallback, this);

  ref_path_pub = nh.advertise<nav_msgs::Path>(ref_path_topic_, 1);
  output_path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_, 1);
  next_path_pub = nh.advertise<nav_msgs::Path>(next_path_topic_, 1);
  steering_angle_pub = nh.advertise<std_msgs::Float64>(steering_angle_topic_, 1);
  turn_signal_pub = nh.advertise<std_msgs::Int16>(turn_signal_topic_, 1);

  // timer
  timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &LocalPlannerNode::mainTimerCallback, this);
};

// Local planner main logic
void LocalPlannerNode::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  // Check if all required data are in position
  if (obstacles.empty() && obstacles_2.empty())
  {
    ROS_FATAL("Local Planner: No obstacles received from both topics, No Path Generated");
    publishEmptyPaths();
    return;
  }
  else if (obstacles.empty())
  {
    ROS_WARN("Local Planner: Obstacles List Is Empty");
  }
  else if (obstacles_2.empty())
  {
    ROS_WARN("Local Planner: Obstacles_2 List Is Empty");
  }
  ROS_INFO("Local Planner: Planning Start");

  // Update Waypoints
  if (!feedWaypoints())
  {
    ROS_WARN("Local Planner: Waiting for Waypoints");
    publishEmptyPaths();
    return;
  }
  // Print Waypoints
  // for (int i = 0; i < local_map_.x.size(); i++)
  // {
  // 	std::cout << "waypoint no." << i << ": " << local_map_.x.at(i) << " " << local_map_.y.at(i) << std::endl;
  // }

  // Update start state
  updateStartState();
  // Get the reference lane's centerline as a spline
  FrenetOptimalTrajectoryPlanner::ResultType result = frenet_planner_instance.generateReferenceCurve(local_map_);
  // Store the results into reference spline
  ref_spline_.x = result.rx;
  ref_spline_.y = result.ry;
  ref_spline_.yaw = result.ryaw;

  if (ref_spline_.x.empty())
  {
    ROS_ERROR("Local Planner: Reference Curve Is Empty, No Path Generated");
    publishEmptyPaths();
    return;
  }
  publishRefSpline(ref_spline_);  //publish to RVIZ for visualisation
  ROS_INFO("Local Planner: Reference Curve Generated");

  // Define ROI width for path sampling
  roi_boundaries_ = getSamplingWidthFromTargetLane(target_lane_, SETTINGS.vehicle_width, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH);

  // Get the planning result (best path of each of the 3 regions, 0 = vehicle transition zone (buggy width), 1 = remaining of left lane, 2 = remaining of right lane
  std::vector<agv::common::FrenetPath> best_path_list = frenet_planner_instance.frenetOptimalPlanning(result.cubic_spline, start_state_, SETTINGS.centre_offset, 
                                                                                                      roi_boundaries_.at(0), roi_boundaries_.at(1), 
                                                                                                      obstacles, obstacles_2);
  
  // Find the best path from the 3 candidates from frenetOptimalPlanning, but still using same cost functions. Behaviour can use this 3 options too choose [NOT IMPLEMENTED 20191213]
  agv::common::FrenetPath best_path = selectLane(best_path_list, current_lane_);
  ROS_INFO("Local Planner: Best Paths Selected");

  // Concatenate the best path into output_path
  concatPath(best_path, OUTPUT_PATH_MAX_SIZE, WP_MAX_SEP, WP_MIN_SEP);

  // Publish the best paths
  publishNextPath(best_path);       //publish to RVIZ for visualisation
  publishOutputPath(output_path_);  //publish to RVIZ for visualisation

  // Publish steeing angle
  publishSteeringAngle(steering_angle_);
}

// Update vehicle current state from the tf transform
void LocalPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_state_.v = odom_msg->twist.twist.linear.x;

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
  pose_before_transform.header.frame_id = odom_msg->header.frame_id;
  pose_before_transform.header.stamp = odom_msg->header.stamp;
  pose_before_transform.pose = odom_msg->pose.pose;
  tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

  tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                   pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_state_.yaw);

  // Current XY of robot (map frame)
  current_state_.x = pose_after_transform.pose.position.x;
  current_state_.y = pose_after_transform.pose.position.y;

  updateVehicleFrontAxleState();
}

// Receive 3D lidar obstacles from obstacle detector
void LocalPlannerNode::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles.push_back(obstacle);
  }
}

// Receive 2D lidar obstacles from obstacle detector 2
void LocalPlannerNode::obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles_2.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles_2.push_back(obstacle);
  }
}

// Receive lane info from the lane publisher
void LocalPlannerNode::laneInfoCallback(const agv::LaneInfo::ConstPtr& lane_info)
{
  map_ = agv::common::Map(lane_info);
}

// Receive behaviour command from the behaviour planner (currently not used)
void LocalPlannerNode::behaviourCallback(const agv::Behaviour::ConstPtr& behaviour_msg)
{
  int path_id = behaviour_msg->ID;
  int state = behaviour_msg->state;
}

// Listen to control output
void LocalPlannerNode::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  // regenerate path when braking
  if (cmd_msg->linear.z == 1.0)
  {
    regenerate_flag_ = true;
  }
}

// Publish the reference spline (for Rviz only)
void LocalPlannerNode::publishRefSpline(const agv::common::Path& path)
{
  nav_msgs::Path ref_path_msg;
  ref_path_msg.header.frame_id = "map";

  for (int i = 0; i < path.yaw.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = path.x.at(i);
    pose.pose.position.y = path.y.at(i);
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(path.yaw.at(i));
    pose.pose.orientation = pose_quat;
    ref_path_msg.poses.emplace_back(pose);
  }

  ref_path_pub.publish(ref_path_msg);
}

// Publish the current path (for Rviz and MPC)
void LocalPlannerNode::publishOutputPath(const agv::common::Path& path)
{
  nav_msgs::Path output_path_msg;
  output_path_msg.header.frame_id = "map";

  for (int i = 0; i < path.yaw.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = path.x.at(i);
    pose.pose.position.y = path.y.at(i);
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(path.yaw.at(i));
    pose.pose.orientation = pose_quat;

    output_path_msg.poses.emplace_back(pose);
  }

  output_path_pub.publish(output_path_msg);
}

// Publish the best next path (for Rviz only)
void LocalPlannerNode::publishNextPath(const agv::common::FrenetPath& frenet_path)
{
  nav_msgs::Path output_path_msg;
  output_path_msg.header.frame_id = "map";

  for (int i = 0; i < frenet_path.c.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = frenet_path.x.at(i);
    pose.pose.position.y = frenet_path.y.at(i);
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(frenet_path.yaw.at(i));
    pose.pose.orientation = pose_quat;

    output_path_msg.poses.emplace_back(pose);
  }

  next_path_pub.publish(output_path_msg);
}

// Publish empty paths (for Rviz only)
void LocalPlannerNode::publishEmptyPaths()
{
  // Publish empty paths
  publishRefSpline(agv::common::Path());
  publishOutputPath(agv::common::Path());
  publishNextPath(agv::common::FrenetPath());
}

// Update the vehicle front axle state (used in odomcallback)
void LocalPlannerNode::updateVehicleFrontAxleState()
{
  // Current XY of robot (map frame)
  frontaxle_state_.x = current_state_.x + (L * std::cos(current_state_.yaw));
  frontaxle_state_.y = current_state_.y + (L * std::sin(current_state_.yaw));
  frontaxle_state_.yaw = current_state_.yaw;
  frontaxle_state_.v = current_state_.v;
}

// Feed map waypoints into local map
bool LocalPlannerNode::feedWaypoints()
{
  if (map_.x.empty() || map_.y.empty())
  {
    ROS_WARN("Local Planner: Waiting for Map XY ");
    return false;
  }
  else if (map_.dx.empty() || map_.dy.empty())
  {
    ROS_WARN("Local Planner: Waiting for Map dx dy");
    return false;
  }

  int start_id = agv::common::lastWaypoint(current_state_, map_);

  // if reached the end of the lane, stop
  if (start_id >= map_.x.size() - 2)  //exclude last 2 waypoints for safety, and prevent code crashing
  {
    return false;
  }

  const double dist = agv::common::distance(map_.x[start_id], map_.y[start_id], current_state_.x, current_state_.y);
  const double lane_heading = atan2(map_.dy[start_id], map_.dx[start_id]) + M_PI / 2;
  const double heading_diff = agv::common::unifyAngleRange(current_state_.yaw - lane_heading);

  if (dist > DISTANCE_THRESH)
  {
    ROS_WARN("Local Planner: Vehicle's Location Is Too Far From The Target Lane");
    return false;
  }
  else if (heading_diff > HEADING_DIFF_THRESH || heading_diff < -HEADING_DIFF_THRESH)
  {
    ROS_WARN("Local Planner: Vehicle's Is Heading In A Different Direction");
    return false;
  }
  else
  {
    // clear the old waypoints
    local_map_.clear();

    if (start_id > map_.x.size() - 5)
    {
      start_id = map_.x.size() - 5;
    }

    for (int i = 0; i < 5; i++)
    {
      // feed the new waypoints
      local_map_.x.push_back(map_.x[start_id + i]);
      local_map_.y.push_back(map_.y[start_id + i]);
      local_map_.dx.push_back(map_.dx[start_id + i]);
      local_map_.dy.push_back(map_.dy[start_id + i]);
      // local_map_.s.push_back(map_.s[start_id + i]);
      // local_map_.left_widths.push_back(map_.left_width[start_id + i]);
      // local_map_.right_widths.push_back(map_.right_width[start_id + i]);
      // local_map_.far_right_widths.push_back(map_.far_right_width[start_id + i]);

      // std::cout << "waypoint no:" << start_id + i << std::endl;
    }

    return true;
  }
}

// Update the vehicle start state in frenet
void LocalPlannerNode::updateStartState()
{
  if (!local_map_.x.empty())
  {
    // The new starting state
    // agv::common::FrenetState new_state;
    
    // if the current path size is too small, regenerate
    if (output_path_.x.size() < OUTPUT_PATH_MIN_SIZE)
    {
      regenerate_flag_ = true;
    }

    // if need to regenerate the entire path
    if (regenerate_flag_)
    {
      ROS_INFO("Local Planner: Regenerating The Entire Path...");
      // Update the starting state in frenet (using ref_spline_ can produce a finer result compared to local_map_, but at fringe cases, such as start of code, ref spline might not be available
      start_state_ = ref_spline_.yaw.empty()? agv::common::getFrenet(current_state_, local_map_) : agv::common::getFrenet(current_state_, ref_spline_);

      // Clear the last output path
      output_path_.clear();
      regenerate_flag_ = false;
    }
    // if not regenerating
    else
    {
      ROS_INFO("Local Planner: Continuing From The Previous Path...");
      
      // End of the previous path speed
      const double output_path_last_speed = hypot(output_path_.x.back() - output_path_.x.end()[-2], 
                                                  output_path_.y.back() - output_path_.y.end()[-2]) / SETTINGS.tick_t;
      // End of the previous path state
      agv::common::VehicleState last_state = agv::common::VehicleState(output_path_.x.back(), output_path_.y.back(), 
                                                                      output_path_.yaw.back(), output_path_last_speed);

      start_state_ = ref_spline_.yaw.empty()? agv::common::getFrenet(last_state, local_map_) : agv::common::getFrenet(last_state, ref_spline_);
    }

    // Ensure the speed is above the minimum planning speed
    start_state_.s_d = std::max(start_state_.s_d, MIN_PLANNING_SPEED);

    // Update current lane
    current_lane_ = (start_state_.d >= -LEFT_LANE_WIDTH/2)? LEFT_LANE : RIGHT_LANE;
  }
}

// Calculate the sampling width for the planner
std::vector<double> LocalPlannerNode::getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width,
                                                                     const double left_lane_width,
                                                                     const double right_lane_width)
{
  double left_bound, right_bound;

  switch (lane_id)
  {
    // both lanes
    case 0:
      left_bound = left_lane_width / 2 - vehicle_width / 2;
      right_bound = -left_lane_width / 2 - right_lane_width + vehicle_width / 2;
      ROS_INFO("Local Planner: Sampling On Both Lanes");
      break;

    // stay within left lane
    case 1:
      left_bound = left_lane_width / 2 - vehicle_width / 2;
      right_bound = -left_bound;
      ROS_INFO("Local Planner: Sampling On The Left Lane");
      break;

    // stay within right lane
    case 2:
      left_bound = -left_lane_width / 2 - (vehicle_width / 2);
      right_bound = -left_lane_width / 2 - right_lane_width + vehicle_width / 2;
      ROS_INFO("Local Planner: Sampling On The Right Lane");
      break;
  }

  return { left_bound, right_bound };
}

// Select the ideal lane to proceed
agv::common::FrenetPath LocalPlannerNode::selectLane(const std::vector<agv::common::FrenetPath>& best_path_list, const int current_lane)
{
  agv::common::FrenetPath best_path;
  bool change_lane_flag;
  int keep_lane_id = -1;
  int change_lane_id = -1;
  double keep_lane_cost = 1000000000.0;
  double change_lane_cost = 1000000000.0;

  for (int i = 0; i < best_path_list.size(); i++)
  {
    if (!best_path_list.at(i).x.empty())
    {
      // keep lane option
      if (best_path_list.at(i).lane_id == current_lane || best_path_list.at(i).lane_id == 0)
      {
        if (best_path_list.at(i).cf < keep_lane_cost)
        {
          keep_lane_id = i;
          keep_lane_cost = best_path_list.at(i).cf;
        }
      }
      // change lane option
      else
      {
        change_lane_id = i;
        change_lane_cost = best_path_list.at(i).cf;
      }
    }
  }
  
  // if both lanes available
  if (keep_lane_id != -1 && change_lane_id != -1)
  {
    if (keep_lane_cost <= change_lane_cost)
    {
      ROS_DEBUG("Local Planner: Keeping Lane");
      change_lane_flag = false;
      best_path = best_path_list.at(keep_lane_id);
    }
    else
    {
      ROS_DEBUG("Local Planner: Changing Lane");
      change_lane_flag = true;
      best_path = best_path_list.at(change_lane_id);
    }
  }
  // if only keep lane available
  else if (keep_lane_id != -1 && change_lane_id == -1)
  {
    ROS_DEBUG("Local Planner: Keeping Lane");
    change_lane_flag = false;
    best_path = best_path_list.at(keep_lane_id);
  }
  // if only change lane available
  else if (keep_lane_id == -1 && change_lane_id != -1)
  {
    ROS_DEBUG("Local Planner: Changing Lane");
    change_lane_flag = true;
    best_path = best_path_list.at(change_lane_id);
  }
  // if none available
  else
  {
    ROS_DEBUG("Local Planner: No Path Available");
    change_lane_flag = false;
    // dummy path
    best_path = agv::common::FrenetPath();
  }

  publishTurnSignal(best_path, change_lane_flag, TURN_YAW_THRESH);

  return best_path;
}

// Concatenate the best next path to the current path
void LocalPlannerNode::concatPath(const agv::common::FrenetPath& frenet_path, const int path_size, const double wp_max_seperation,
                                  const double wp_min_seperation)
{
  // Concatenate the best path to the output path
  int diff = std::min(path_size - output_path_.x.size(), frenet_path.x.size());
  // std::cout << "Output Path Size: " << output_path_.x.size() << " Current Size: " << path_size << " Diff: " << diff
  //           << " Next Path Size: " << frenet_path.x.size() << std::endl;

  for (int i = 0; i < diff; i++)
  {
    double wp_seperation;

    // Check if the separation between adjacent waypoint are permitted
    if (!output_path_.x.empty())
    {
      wp_seperation = agv::common::distance(output_path_.x.back(), output_path_.y.back(), frenet_path.x.at(i), frenet_path.y.at(i));
    }
    else
    {
      wp_seperation = agv::common::distance(frenet_path.x.at(i), frenet_path.y.at(i), frenet_path.x.at(i + 1), frenet_path.y.at(i + 1));
    }

    // If the separation is too big/small, reject point onward
    if (wp_seperation >= wp_max_seperation || wp_seperation <= wp_min_seperation)
    {
      // ROS_WARN("Local Planner: waypoint out of bound, rejected");
      // regenerate_flag_ = true;
      break;
    }

    output_path_.x.push_back(frenet_path.x.at(i));
    output_path_.y.push_back(frenet_path.y.at(i));
    output_path_.yaw.push_back(frenet_path.yaw.at(i));

    // std::cout << "Concatenate round " << i << ": Output Path Size: " << output_path_.x.size() << std::endl;
  }

  // Calculate steering angle
  if (!output_path_.x.empty() && !output_path_.y.empty())
  {
    const int next_frontlink_wp_id = agv::common::nextWaypoint(frontaxle_state_, output_path_);
    ROS_INFO("Local Planner: Stanley Start");
    steering_angle_ = calculateSteeringAngle(next_frontlink_wp_id, frontaxle_state_);

    const int next_wp_id = agv::common::nextWaypoint(current_state_, output_path_);

    for (int i = 0; i < next_wp_id; i++)
    {
      output_path_.x.erase(output_path_.x.begin());
      output_path_.y.erase(output_path_.y.begin());
      output_path_.yaw.erase(output_path_.yaw.begin());
    }
  }
  else
  {
    ROS_ERROR("Local Planner: Output Path is Empty, No Steering Angle");
  }
}

// Steeing Help Function
double LocalPlannerNode::calculateSteeringAngle(const int next_wp_id, const agv::common::VehicleState& frontaxle_state)
{
  const double wp_id = next_wp_id + NUM_LOOK_AHEAD_WP;
  // std::cout << "Output Path Size: " << output_path_.x.size() << " Next Waypoint ID: " << wp_id << std::endl;

  // If the current path is too short, return error value
  if (output_path_.x.size() < wp_id + 2)
  {
    ROS_ERROR("Local Planner: Output Path Too Short! No output steering angle");
    // std::cout << "Output Path Size: " << output_path_.x.size() << " Required Size: " << wp_id + 2 << std::endl;
    regenerate_flag_ = true;
    return ERROR_VALUE;
  }
  else
  {
    // First Term
    const double delta_yaw = agv::common::unifyAngleRange(output_path_.yaw.at(wp_id) - current_state_.yaw);

    // Second Term
    const double c = agv::common::distance(output_path_.x.at(wp_id), output_path_.y.at(wp_id), output_path_.x.at(wp_id + 1), output_path_.y.at(wp_id + 1));
    // if two waypoints overlapped, return error value
    if (c <= WP_MIN_SEP)
    {
      regenerate_flag_ = true;
      return ERROR_VALUE;
    }
    const double a = agv::common::distance(frontaxle_state.x, frontaxle_state.y, output_path_.x.at(wp_id), output_path_.y.at(wp_id));
    const double b = agv::common::distance(frontaxle_state.x, frontaxle_state.y, output_path_.x.at(wp_id + 1), output_path_.y.at(wp_id + 1));
    // if the vehicle is too far from the waypoint, return error value
    if (a >= WP_MAX_SEP || b >= WP_MAX_SEP)
    {
      ROS_WARN("Local Planner: Vehicle is too far from the path, Regenerate");
      regenerate_flag_ = true;
      return ERROR_VALUE;
    }

    const double p = (a + b + c) / 2.0;
    const double triangle_area = sqrt(p * (p - a) * (p - b) * (p - c));
    const double x = triangle_area * 2.0 / c;
    const double u = std::max(1.0, current_state_.v);

    // Angle of std::vector vehicle -> waypoint
    const double vectors_angle_diff = atan2(frontaxle_state.y - output_path_.y.at(wp_id), frontaxle_state.x - output_path_.x.at(wp_id)) - output_path_.yaw.at(wp_id);
    const double vectors_angle_diff_unified = agv::common::unifyAngleRange(vectors_angle_diff);
    const int direction = vectors_angle_diff_unified < 0 ? 1 : -1;

    // Final Angle
    steering_angle_ = STANLEY_OVERALL_GAIN * (delta_yaw + direction * atan(TRACK_ERROR_GAIN * x / u));
    // Check if exceeding max steering angle
    steering_angle_ = agv::common::limitWithinRange(steering_angle_, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    std::cout << "Steering Angle: " << agv::common::rad2deg(steering_angle_) << " degrees" << std::endl;

    return steering_angle_;
  }
}

// Publish the resulted steering angle (Stanley)
void LocalPlannerNode::publishSteeringAngle(const double angle)
{
  // If steering angle is an error value, publish nothing
  if (angle <= ERROR_VALUE)
  {
    ROS_ERROR("Local Planner: Output steering angle is NULL");
    return;
  }
  // If steering angle is valid, publish it
  else
  {
    std_msgs::Float64 steering_angle_msg;
    steering_angle_msg.data = angle;
    steering_angle_pub.publish(steering_angle_msg);
  }
}

// Publish the turn signal
void LocalPlannerNode::publishTurnSignal(const agv::common::FrenetPath& best_path, const bool change_lane, const double yaw_thresh)
{
  if (change_lane)
  {
    if (current_lane_ == 1)
    {
      target_lane_ = 2;
    }
    else if (current_lane_ == 2)
    {
      target_lane_ = 1;
    }
  }
  else
  {
    target_lane_ = current_lane_;
  }
  
  if (best_path.yaw.empty())
  {
    ROS_ERROR("Local Planner: No Path Generated");
  }
  else
  {
    // Check for yaw difference
    const double delta_yaw = best_path.yaw.back() - current_state_.yaw;

    // std::cout << "path yaw: " << best_path.yaw.back() << " vehicle yaw: " << best_path.yaw.front() << std::endl;
    // std::cout << "delta_yaw:" << delta_yaw << std::endl;

    if (delta_yaw >= yaw_thresh)
    {
      turn_signal_ = 1;
      ROS_DEBUG("Local Planner: Turning Left");
    }
    else if (delta_yaw <= -yaw_thresh)
    {
      turn_signal_ = -1;
      ROS_DEBUG("Local Planner: Turning Right");
    }
    else
    {
      // turn left
      if (target_lane_ < current_lane_)
      {
        turn_signal_ = 1;
        ROS_DEBUG("Local Planner: Changing Lane Left");
      }
      // turn right
      else if (target_lane_ > current_lane_)
      {
        turn_signal_ = -1;
        ROS_DEBUG("Local Planner: Changing Lane Right");
      }
      else
      {
        turn_signal_ = 0;
      }
    }
  }

  std_msgs::Int16 turn_signal_msg;
  turn_signal_msg.data = turn_signal_;
  turn_signal_pub.publish(turn_signal_msg);
}

} // namespace local_planner
} // namespace agv

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_node");
  agv::local_planner::LocalPlannerNode local_planner_node;
  ros::spin();  // spin the ros node.
  return 0;
}
