/** behaviour_planner_node.cpp
 *
 * Copyright (C) 2019 Haoren & Brina & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 *  1) Gets desired modes from collision_detector_topic and special_waypoint_topic, and
 *     executes the most conservative mode by publishing to nav_cmd_topic
 *  2) Filters out obstacles that are not in the lanes, and
 *     classify static and dynamic obstacles that are within the lanes into 4 sections:
 *     frontLeft, frontRight, backLeft and backRight
 */

// TODO: add documentation of nav cmd out x,y,z, etc

#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <obstacle_detector/Obstacles.h>
#include <agv/LaneInfo.h>

#include <dynamic_reconfigure/server.h>
#include <agv/behaviour_planner_Config.h>

#include "common/frenet.h"
#include "common/obstacle.h"
#include "behaviour_planner/behaviour_planner_helper.h"
#include "local_planner/frenet_optimal_trajectory_planner.h"

namespace agv
{
namespace behaviour_planner
{
// List of dynamic parameters
double MARK_HEIGHT;
double NORMAL_CRUISE_SPEED;
double SLOPE_SLOWDOWN_SPEED;
double NORMAL_SLOWDOWN_SPEED;
double MEDIUM_STEERING_ANGLE_SLOWDOWN_SPEED;
double LARGE_STEERING_ANGLE_SLOWDOWN_SPEED;

double MEDIUM_STEERING_ANGLE;
double LARGE_STEERING_ANGLE;

double DYNAMIC_OBSTACLE_BRAKE;  // between 0 to 1
double STATIC_OBSTACLE_BRAKE;   // between 0 to 1
double ENDPOINT_BRAKE;          // between 0 to 1
double SLOPE_BRAKE;             // between 0 to 1

double DYNAMIC_OBSTACLE_THRESH;
double DYNAMIC_OBSTACLE_WAIT_TIME;

// Dynamic parameter callback function
void dynamicParamCallback(agv::behaviour_planner_Config& config, uint32_t level)
{
  MARK_HEIGHT = config.mark_height;
  NORMAL_CRUISE_SPEED = config.normal_cruise_speed;
  SLOPE_SLOWDOWN_SPEED = config.slope_slowdown_speed;
  NORMAL_SLOWDOWN_SPEED = config.normal_slowdown_speed;
  MEDIUM_STEERING_ANGLE_SLOWDOWN_SPEED = config.medium_steering_angle_slowdown_speed;
  LARGE_STEERING_ANGLE_SLOWDOWN_SPEED = config.large_steering_angle_slowdown_speed;
  MEDIUM_STEERING_ANGLE = config.medium_steering_angle;
  LARGE_STEERING_ANGLE = config.large_steering_angle;
  DYNAMIC_OBSTACLE_BRAKE = config.dynamic_obstacle_brake;
  STATIC_OBSTACLE_BRAKE = config.static_obstacle_brake;
  ENDPOINT_BRAKE = config.endpoint_brake;
  SLOPE_BRAKE = config.slope_brake;
  DYNAMIC_OBSTACLE_THRESH = config.dynamic_obstacle_thresh;
  DYNAMIC_OBSTACLE_WAIT_TIME = config.dynamic_obstacle_wait_time;
}

class BehaviourPlanner
{
public:
  // Constructor
  BehaviourPlanner();
  // Destructor
  virtual ~BehaviourPlanner(){};

private:
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Subscriber desired_steering_angle_sub;
  ros::Subscriber current_steering_angle_sub;
  ros::Subscriber obstacle_sub;   // 3D LiDAR
  ros::Subscriber obstacle2_sub;  // Front 2D LiDAR
  ros::Subscriber obstacle3_sub;  // Left and Right 2D LiDARs
  ros::Subscriber collision_detector_sub;
  ros::Subscriber special_waypoint_sub;
  ros::Subscriber lane_info_sub;

  // Publishers
  ros::Publisher nav_cmd_pub;
  ros::Publisher front_left_obs_pub;
  ros::Publisher front_right_obs_pub;
  ros::Publisher back_left_obs_pub;
  ros::Publisher back_right_obs_pub;
  ros::Publisher stopwall_pub;

  ros::Timer timer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  /* Private variables */
  agv::common::VehicleState current_state_;

  agv::common::Path path; // to obtain path coordinates from local_planner, for now only checking if we are getting the path data

  double desired_steering_angle;
  double current_steering_angle;

  std::vector<agv::common::CircleObstacle> obstacles;   // 3D LiDAR data from /obstacles topic
  std::vector<agv::common::CircleObstacle> obstacles2;  // 2D front LiDAR data from /obstacles2 topic
  std::vector<agv::common::CircleObstacle> obstacles3;  // 2D left and right LiDAR data from /obstacles3 topic

  int collision_mode;  // desired mode as stated in enum desired_mode from collision detector node
  int waypoint_mode;   // desired mode as stated in enum desired_mode from lane publisher node

  geometry_msgs::Twist nav_cmd_msg;  // final message to be published

  enum DesiredMode
  {
    DYNAMIC_OBSTACLE_STOP,  // to stop and wait a while, see code below
    STATIC_OBSTACLE_STOP,   // to stop only
    ENDPOINT_STOP,          // to stop at end of path
    SLOPE_SLOWDOWN,         // to slowdown at slopes
    NORMAL_SLOWDOWN,  // slowdown at turning points specified by landmarks file and when approaching obstacle. TODO: to
                      // slowdown at the right time when overtaking
    NORMAL_SPEED      // all other cases
  };

  enum Lane
  {
    OWN_LANE,
    NEXT_LANE,
    BOTH_LANES,
    NOT_IN_LANES
  };

  double desired_speed_from_final_mode;
  double desired_speed_from_steering_angle;

  bool collision_cb_flag = false;  // to ensure that we are getting the mode from collision detector node
  bool waypoint_cb_flag = false;   // to ensure that we are getting the mode from lane publisher node
  bool near_stop_flag = false;     // flag to check if neat end point of path

  double planning_frequency_;

  double brake_intensity = 0;  // 0 is no brake, 1 is full brake

  int dynamic_obstacle_counter = 0;

  bool stopwall_flag = false;

  // Frenet Planner
  agv::local_planner::FrenetOptimalTrajectoryPlanner frenet_planner_instance;

  agv::common::Map map_;
  agv::common::Map local_map_;

  // Reference Spline
  agv::common::Path ref_spline;

  dynamic_reconfigure::Server<agv::behaviour_planner_Config> server;
  dynamic_reconfigure::Server<agv::behaviour_planner_Config>::CallbackType f;

  // Callbacks
  void mainTimerCallback(const ros::TimerEvent& timer_event);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
  void desiredSteeringAngleCallback(const std_msgs::Float64::ConstPtr& desired_steering_angle_msg);
  void currentSteeringAngleCallback(const std_msgs::Float64::ConstPtr& current_steering_angle_msg);
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void obstacle3Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void collisionDetectorCallback(const std_msgs::Int16::ConstPtr& collision_msg);
  void specialWaypointCallback(const std_msgs::Int16::ConstPtr& waypoint_msg);
  void laneInfoCallback(const agv::LaneInfo::ConstPtr& lane_info);

  // Functions
  void initializeStartingVariables();
  void publishNavCmd(const double brake_intensity);
  void finalModeToNavCmd();
  bool feedWaypoints();
  agv::common::FrenetState getVehicleFrenet(const agv::common::VehicleState& state);
  agv::common::FrenetState getRelativeVehicleFrenet(const agv::common::CircleObstacle& obstacle, const agv::common::FrenetState& vehicle_frenet);
  void processObstacles();
  void filterAndClassifyObstacle(agv::common::CircleObstacle& obstacle, const int closest_wp_id);
  int checkObstacleLane(const agv::common::CircleObstacle& obstacle, const int closest_wp_id);
  void publishObstacles();
  void stopwallVisualization();
};

// Constructor
BehaviourPlanner::BehaviourPlanner() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  // topics
  std::string odom_topic_;
  std::string path_topic_;
  std::string desired_steering_angle_topic_;
  std::string current_steering_angle_topic_;
  std::string obstacle_topic_;
  std::string obstacle2_topic_;
  std::string obstacle3_topic_;
  std::string collision_detector_topic_;
  std::string special_waypoint_topic_;
  std::string lane_info_topic_;

  std::string nav_cmd_topic_;
  std::string front_left_obs_topic_;
  std::string front_right_obs_topic_;
  std::string back_left_obs_topic_;
  std::string back_right_obs_topic_;
  std::string stopwall_marker_;

  double planning_frequency_;

  // Input Topics
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("path_msg_topic", path_topic_));
  ROS_ASSERT(private_nh.getParam("desired_steering_angle_topic", desired_steering_angle_topic_));
  ROS_ASSERT(private_nh.getParam("current_steering_angle_topic", current_steering_angle_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle2_topic", obstacle2_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle3_topic", obstacle3_topic_));
  ROS_ASSERT(private_nh.getParam("collision_detector", collision_detector_topic_));
  ROS_ASSERT(private_nh.getParam("special_waypoint", special_waypoint_topic_));
  ROS_ASSERT(private_nh.getParam("lane_info_topic", lane_info_topic_));

  // Output Topics
  ROS_ASSERT(private_nh.getParam("nav_cmd_topic", nav_cmd_topic_));
  ROS_ASSERT(private_nh.getParam("front_left_obs_topic", front_left_obs_topic_));
  ROS_ASSERT(private_nh.getParam("front_right_obs_topic", front_right_obs_topic_));
  ROS_ASSERT(private_nh.getParam("back_left_obs_topic", back_left_obs_topic_));
  ROS_ASSERT(private_nh.getParam("back_right_obs_topic", back_right_obs_topic_));
  ROS_ASSERT(private_nh.getParam("stopwall_marker", stopwall_marker_));

  // Parameters
  ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  odom_sub = nh.subscribe(odom_topic_, 1, &BehaviourPlanner::odomCallback, this);
  path_sub = nh.subscribe(path_topic_, 1, &BehaviourPlanner::pathCallback, this);
  desired_steering_angle_sub =
      nh.subscribe(desired_steering_angle_topic_, 1, &BehaviourPlanner::desiredSteeringAngleCallback, this);
  current_steering_angle_sub =
      nh.subscribe(current_steering_angle_topic_, 1, &BehaviourPlanner::currentSteeringAngleCallback, this);
  obstacle_sub = nh.subscribe(obstacle_topic_, 1, &BehaviourPlanner::obstacleCallback, this);
  obstacle2_sub = nh.subscribe(obstacle2_topic_, 1, &BehaviourPlanner::obstacle2Callback, this);
  obstacle3_sub = nh.subscribe(obstacle3_topic_, 1, &BehaviourPlanner::obstacle3Callback, this);
  collision_detector_sub =
      nh.subscribe(collision_detector_topic_, 1, &BehaviourPlanner::collisionDetectorCallback, this);
  special_waypoint_sub = nh.subscribe(special_waypoint_topic_, 1, &BehaviourPlanner::specialWaypointCallback, this);
  lane_info_sub = nh.subscribe(lane_info_topic_, 1, &BehaviourPlanner::laneInfoCallback, this);

  nav_cmd_pub = nh.advertise<geometry_msgs::Twist>(nav_cmd_topic_, 1);
  front_left_obs_pub = nh.advertise<obstacle_detector::Obstacles>(front_left_obs_topic_, 1);
  front_right_obs_pub = nh.advertise<obstacle_detector::Obstacles>(front_right_obs_topic_, 1);
  back_left_obs_pub = nh.advertise<obstacle_detector::Obstacles>(back_left_obs_topic_, 1);
  back_right_obs_pub = nh.advertise<obstacle_detector::Obstacles>(back_right_obs_topic_, 1);
  stopwall_pub = nh.advertise<visualization_msgs::Marker>(stopwall_marker_, 0);

  // timer
  timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &BehaviourPlanner::mainTimerCallback, this);

  initializeStartingVariables();

  ROS_INFO("Behaviour Planner: Ready.");
};

void BehaviourPlanner::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  processObstacles();
  publishObstacles();

  if (dynamic_obstacle_counter == 0)
  {
    finalModeToNavCmd();
  }
  else if (dynamic_obstacle_counter == 30)
  {
    dynamic_obstacle_counter = 0;
  }
  else
  {
    ROS_WARN("Stopping for dynamic obstacle!");
    desired_speed_from_final_mode = 0;
    publishNavCmd(DYNAMIC_OBSTACLE_BRAKE);
    dynamic_obstacle_counter++;
    ROS_INFO("counter: %d", dynamic_obstacle_counter);
  }
}

void BehaviourPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
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
}

// check if path exists
void BehaviourPlanner::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  path.clear();
  for (int i = 0; i < path_msg->poses.size(); i++)
  {
    path.x.push_back(path_msg->poses[i].pose.position.x);
  }
}

void BehaviourPlanner::desiredSteeringAngleCallback(const std_msgs::Float64::ConstPtr& desired_steering_angle_msg)
{
  desired_steering_angle = desired_steering_angle_msg->data;
}

void BehaviourPlanner::currentSteeringAngleCallback(const std_msgs::Float64::ConstPtr& current_steering_angle_msg)
{
  current_steering_angle = current_steering_angle_msg->data;
}

void BehaviourPlanner::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles.push_back(obstacle);
  }
}

void BehaviourPlanner::obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles2.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles2.push_back(obstacle);
  }
}

void BehaviourPlanner::obstacle3Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles3.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles3.push_back(obstacle);
  }
}

// get desired_mode from collision detector node
void BehaviourPlanner::collisionDetectorCallback(const std_msgs::Int16::ConstPtr& collision_msg)
{
  collision_mode = collision_msg->data;
  collision_cb_flag = true;
}

// get desired_mode from lane publisher node
void BehaviourPlanner::specialWaypointCallback(const std_msgs::Int16::ConstPtr& waypoint_msg)
{
  waypoint_mode = waypoint_msg->data;
  waypoint_cb_flag = true;
}

void BehaviourPlanner::laneInfoCallback(const agv::LaneInfo::ConstPtr& lane_info)
{
  map_ = agv::common::Map(lane_info);
}

void BehaviourPlanner::initializeStartingVariables()
{
  desired_speed_from_final_mode = NORMAL_CRUISE_SPEED;
  desired_speed_from_steering_angle = NORMAL_CRUISE_SPEED;
  desired_steering_angle = 0;
  current_state_.v = 0;
}

void BehaviourPlanner::publishNavCmd(const double brake_intensity)
{
  if (std::max(std::fabs(desired_steering_angle), std::fabs(current_steering_angle)) > LARGE_STEERING_ANGLE)
  {
    desired_speed_from_steering_angle = LARGE_STEERING_ANGLE_SLOWDOWN_SPEED;
  }
  else if (std::max(std::fabs(desired_steering_angle), std::fabs(current_steering_angle)) > MEDIUM_STEERING_ANGLE)
  {
    desired_speed_from_steering_angle = MEDIUM_STEERING_ANGLE_SLOWDOWN_SPEED;
  }
  else
  {
    desired_speed_from_steering_angle = NORMAL_CRUISE_SPEED;
  }

  nav_cmd_msg.linear.x = std::min(desired_speed_from_final_mode, desired_speed_from_steering_angle);
  nav_cmd_msg.linear.y = 0;  // TODO: can delete this?
  nav_cmd_msg.linear.z = brake_intensity;
  nav_cmd_msg.angular.z = (near_stop_flag) ? 0 : desired_steering_angle;

  nav_cmd_pub.publish(nav_cmd_msg);

  if (nav_cmd_msg.linear.x == 0)
  {
    stopwall_flag = true;
    stopwallVisualization();
    stopwall_flag = false;
    return;
  }
  else
  {
    stopwall_flag = false;
    stopwallVisualization();
  }
}

// execute the most conservative mode
void BehaviourPlanner::finalModeToNavCmd()
{
  if (!collision_cb_flag || !waypoint_cb_flag || path.x.size() == 0)  // check if we are getting the info that we need
  {
    ROS_WARN("Insufficient information! Stop!");
    desired_speed_from_final_mode = 0;
    publishNavCmd(0.8);
    return;
  }

  int final_mode = std::min(collision_mode, waypoint_mode);  // choose most conservative mode

  switch (final_mode)
  {
    case DYNAMIC_OBSTACLE_STOP:
      dynamic_obstacle_counter = 1;
      ROS_WARN("Dynamic obstacle detected!");
      desired_speed_from_final_mode = 0;
      publishNavCmd(DYNAMIC_OBSTACLE_BRAKE);
      break;

    case STATIC_OBSTACLE_STOP:
      ROS_WARN("Static obstacle detected!");
      desired_speed_from_final_mode = 0;
      publishNavCmd(STATIC_OBSTACLE_BRAKE);
      break;

    case ENDPOINT_STOP:
      ROS_WARN("Destination reached!");
      desired_speed_from_final_mode = 0;
      near_stop_flag = true;
      publishNavCmd(ENDPOINT_BRAKE);
      break;

    case SLOPE_SLOWDOWN:
      ROS_WARN("Slowing down at slope!");
      desired_speed_from_final_mode = SLOPE_SLOWDOWN_SPEED;
      publishNavCmd(SLOPE_BRAKE);
      break;

    case NORMAL_SLOWDOWN:
      ROS_WARN("Slowing down!");
      desired_speed_from_final_mode = NORMAL_SLOWDOWN_SPEED;
      publishNavCmd(0);
      break;

    case NORMAL_SPEED:
      ROS_INFO("It is safe!");
      desired_speed_from_final_mode = NORMAL_CRUISE_SPEED;
      publishNavCmd(0);
      break;
  }
}

bool BehaviourPlanner::feedWaypoints()
{
  if (map_.x.empty() || map_.y.empty())
  {
    ROS_WARN("Behaviour Planner: Waiting for Map XY ");
    return false;
  }
  else if (map_.dx.empty() || map_.dy.empty())
  {
    ROS_WARN("Behaviour Planner: Waiting for Map dx dy");
    return false;
  }

  int start_id = agv::common::lastWaypoint(current_state_, map_);

  // if reached the end of the lane, stop
  if (start_id >= map_.x.size() - 2)
  {
    return false;
  }
  
  if (start_id > map_.x.size() - 5)
  {
    start_id = map_.x.size() - 5;
  }

  if (start_id < 2)
  {
    start_id = 2;
  }

  local_map_.clear();

  for (int i = -2; i < 3; i++)
  {
    // feed the new waypoints
    local_map_.x.push_back(map_.x[start_id + i]);
    local_map_.y.push_back(map_.y[start_id + i]);
    local_map_.dx.push_back(map_.dx[start_id + i]);
    local_map_.dy.push_back(map_.dy[start_id + i]);
    local_map_.s.push_back(map_.s[start_id + i]);
    local_map_.left_widths.push_back(map_.left_widths[start_id + i]);
    local_map_.right_widths.push_back(map_.right_widths[start_id + i]);
    local_map_.far_right_widths.push_back(map_.far_right_widths[start_id + i]);

    // std::cout << "waypoint no:" << start_id + i << std::endl;
  }

  // Get the reference lane's centerline as a spline
  agv::local_planner::FrenetOptimalTrajectoryPlanner::ResultType result = frenet_planner_instance.generateReferenceCurve(local_map_);

  // Store reference spline
  ref_spline.x = result.rx;
  ref_spline.y = result.ry;
  ref_spline.yaw = result.ryaw;

  return true;
}

// get absolute frenet coordinates of object
agv::common::FrenetState BehaviourPlanner::getVehicleFrenet(const agv::common::VehicleState& state)
{
  if (!local_map_.x.empty())
  {
    auto frenet_coords = ref_spline.yaw.empty()? agv::common::getFrenet(state, local_map_) : agv::common::getFrenet(state, ref_spline);
    return frenet_coords;
  }
}

// to find out the frenet coordinates of an obstacle. The s longitudinal distance is wrt baselink, the d lateral
// distance is wrt to reference spline
agv::common::FrenetState BehaviourPlanner::getRelativeVehicleFrenet(const agv::common::CircleObstacle& obstacle, const agv::common::FrenetState& vehicle_frenet)
{
  if (!local_map_.x.empty())
  {
    auto obstacle_frenet = getVehicleFrenet(obstacle.state);
    obstacle_frenet.s = obstacle_frenet.s - vehicle_frenet.s;
    obstacle_frenet.s_d = obstacle_frenet.s_d - vehicle_frenet.s_d;

    return obstacle_frenet;
  }
}

// pre-process obstacles
void BehaviourPlanner::processObstacles()
{
  if (!feedWaypoints())
  {
    ROS_WARN("Waypoints are empty!");
    return;
  }
  else if (obstacles.empty() || obstacles2.empty() || obstacles3.empty())
  {
    ROS_WARN("Obstacles are empty!");
    return;
  }

  agv::common::FrenetState current_frenet = getVehicleFrenet(current_state_);

  for (int i = 0; i < obstacles.size(); i++)
  {
    obstacles.at(i).frenet_state = getRelativeVehicleFrenet(obstacles.at(i), current_frenet);
    // Filter obstacle based on its frenet coordinates
    const int closest_wp_id = agv::common::closestWaypoint(obstacles.at(i).state, map_);
    filterAndClassifyObstacle(obstacles.at(i), closest_wp_id);  
  }

  for (int i = 0; i < obstacles2.size(); i++)
  {
    obstacles2.at(i).frenet_state = getRelativeVehicleFrenet(obstacles2.at(i), current_frenet);
    // Filter obstacle based on its frenet coordinates
    const int closest_wp_id = agv::common::closestWaypoint(obstacles2.at(i).state, map_);
    filterAndClassifyObstacle(obstacles2.at(i), closest_wp_id);
  }

  for (int i = 0; i < obstacles3.size(); i++)
  {
    obstacles3.at(i).frenet_state = getRelativeVehicleFrenet(obstacles3.at(i), current_frenet);
    // Filter obstacle based on its frenet coordinates
    const int closest_wp_id = agv::common::closestWaypoint(obstacles3.at(i).state, map_);
    filterAndClassifyObstacle(obstacles3.at(i), closest_wp_id);
  }
}

// filter out obstacles that are not in lanes, and classify them into 4 different quadrants
void BehaviourPlanner::filterAndClassifyObstacle(agv::common::CircleObstacle& obstacle, const int closest_wp_id)
{
  const int lane = checkObstacleLane(obstacle, closest_wp_id);

  switch (lane)
  {
    case OWN_LANE:
      // Check if the obstacle is in front of us
      obstacle.quadrant = (obstacle.frenet_state.s >= 0)? 1 : 4;
      // Check if the obstacle is static
      obstacle.is_static = (obstacle.state.v < DYNAMIC_OBSTACLE_THRESH)? true : false;
      break;

    case NEXT_LANE:
      // Check if the obstacle is in front of us
      obstacle.quadrant = (obstacle.frenet_state.s >= 0)? 2 : 3;
      // Check if the obstacle is static
      obstacle.is_static = (obstacle.state.v < DYNAMIC_OBSTACLE_THRESH)? true : false;
      break;

    case BOTH_LANES: 
     // Check if the obstacle is in front of us
      obstacle.quadrant = (obstacle.frenet_state.s >= 0)? 1 : 4;
      // Check if the obstacle is static
      obstacle.is_static = (obstacle.state.v < DYNAMIC_OBSTACLE_THRESH)? true : false;
      break;

    case NOT_IN_LANES:
      break;
  }
}

// check which lane the obstacle is in
int BehaviourPlanner::checkObstacleLane(const agv::common::CircleObstacle& obstacle, const int closest_wp_id)
{
  // take the lane width to be the same as that at the closest waypoint
  const double left_width = map_.left_widths.at(closest_wp_id);
  const double right_width = map_.right_widths.at(closest_wp_id);
  const double far_right_width = map_.far_right_widths.at(closest_wp_id);

  int lane_id;

  if (obstacle.frenet_state.d > left_width)
  {
    if ((obstacle.frenet_state.d - obstacle.radius) <= left_width)
    {
      lane_id = OWN_LANE;
    }
  }
  else if (obstacle.frenet_state.d <= left_width && obstacle.frenet_state.d >= -right_width)
  {
    lane_id = ((obstacle.frenet_state.d - obstacle.radius) <= -right_width)? BOTH_LANES : OWN_LANE;
  }
  else if (obstacle.frenet_state.d < -right_width && obstacle.frenet_state.d >= -far_right_width)
  {
    lane_id = ((obstacle.frenet_state.d + obstacle.radius) >= -right_width)? BOTH_LANES : NEXT_LANE;
  }
  else if (obstacle.frenet_state.d < -far_right_width)
  {
    if ((obstacle.frenet_state.d + obstacle.radius) >= -far_right_width)
    {
      lane_id = NEXT_LANE;
    }
  }
  else
  {
    lane_id = NOT_IN_LANES;
  }

  return lane_id;
}

// visualize obstacles on rviz
void BehaviourPlanner::publishObstacles()
{
  obstacle_detector::Obstacles front_left_obs_msg = feedObstaclesMsg(obstacles, obstacles2, obstacles3, 1);
  obstacle_detector::Obstacles front_right_obs_msg = feedObstaclesMsg(obstacles, obstacles2, obstacles3, 2);
  obstacle_detector::Obstacles back_left_obs_msg = feedObstaclesMsg(obstacles, obstacles2, obstacles3, 4);
  obstacle_detector::Obstacles back_right_obs_msg = feedObstaclesMsg(obstacles, obstacles2, obstacles3, 3);

  front_left_obs_pub.publish(front_left_obs_msg);
  front_right_obs_pub.publish(front_right_obs_msg);
  back_left_obs_pub.publish(back_left_obs_msg);
  back_right_obs_pub.publish(back_right_obs_msg);

  return;
}

void BehaviourPlanner::stopwallVisualization()
{
  double marker_height = MARK_HEIGHT;

  visualization_msgs::Marker stopwall_marker;
  stopwall_marker.header.frame_id = "map";
  stopwall_marker.header.stamp = ros::Time::now();
  stopwall_marker.ns = "rviz_stopwall_visualization";
  stopwall_marker.id = 0;
  stopwall_marker.type = visualization_msgs::Marker::CUBE;
  stopwall_marker.action = visualization_msgs::Marker::ADD;

  double front_x = current_state_.x + cos(current_state_.yaw) * 2.7;
  double front_y = current_state_.y + sin(current_state_.yaw) * 2.7;

  stopwall_marker.pose.position.x = front_x;
  stopwall_marker.pose.position.y = front_y;
  stopwall_marker.pose.position.z = marker_height / 2;

  stopwall_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, current_state_.yaw);

  if (stopwall_flag)
  {
    stopwall_marker.scale.x = 0.1;
    stopwall_marker.scale.y = marker_height;
    stopwall_marker.scale.z = 3.0;
  }
  else
  {
    stopwall_marker.scale.x = 0;
    stopwall_marker.scale.y = 0;
    stopwall_marker.scale.z = 0;
  }

  stopwall_marker.color.r = 1.0f;
  stopwall_marker.color.g = 0.2f;
  stopwall_marker.color.b = 0.1f;
  stopwall_marker.color.a = 0.45;
  stopwall_marker.lifetime = ros::Duration();

  stopwall_pub.publish(stopwall_marker);
}

}  // end of namespace behaviour_planner
}  // end of namespace agv

int main(int argc, char** argv)
{
  ros::init(argc, argv, "behaviour_planner_node");
  agv::behaviour_planner::BehaviourPlanner behaviour_planner_obj;
  ros::spin();
  return 0;
}