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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <obstacle_detector/Obstacles.h>
#include <agv/Behaviour.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <agv/collision_detector_Config.h>

#include "collision_detector/collision_detector_helper.h"

namespace agv
{
namespace behaviour_planner
{
double COLLISION_TIME_THRESHOLD;
double LOOK_AHEAD_TIME;
double DELTA_T;

double CIRCLE_RADIUS;
double SAFETY_MARGIN;
double BACK_SAFETY_MARGIN;  // back lidar safety margin
double SPEED_THRESHOLD;     // min speed of buggy to be used for forward projection
double DYNAMIC_OBSTACLE_SPEED;

void dynamicParamCallback(agv::collision_detector_Config& config, uint32_t level)
{
  COLLISION_TIME_THRESHOLD = config.collision_time_threshold;
  LOOK_AHEAD_TIME = config.look_ahead_time;
  DELTA_T = config.delta_t;
  CIRCLE_RADIUS = config.circle_radius;
  SAFETY_MARGIN = config.safety_margin;
  BACK_SAFETY_MARGIN = config.back_safety_margin;
  SPEED_THRESHOLD = config.speed_threshold;
  DYNAMIC_OBSTACLE_SPEED = config.dynamic_obstacle_speed;
}
class CollisionDetector
{
public:
  // Constructor
  CollisionDetector();

  // Destructor
  virtual ~CollisionDetector(){};

private:
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber obstacle2_sub;
  ros::Subscriber obstacle3_sub;
  ros::Subscriber path_sub;
  ros::Subscriber steering_angle_sub;

  // Publishers
  ros::Publisher collision_detector_pub;
  ros::Publisher pred_traj_pub;
  ros::Publisher dynamic_obstacle_path_pub;
  ros::Publisher stopwall_pub;
  ros::Publisher collision_time_pub;

  ros::Timer timer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  double planning_frequency_;

  agv::common::VehicleState current_state_;
  agv::common::VehicleState frontaxle_state_;
  agv::common::VehicleState COG_state_;

  agv::common::Path predicted_path;
  agv::common::Path dynamic_obstacle_path;

  std::vector<agv::common::CircleObstacle> obstacles;   // 3D LiDAR data from /obstacles topic
  std::vector<agv::common::CircleObstacle> obstacles2;  // 2D front LiDAR data from /obstacles2 topic
  std::vector<agv::common::CircleObstacle> obstacles3;  // 2D left and right LiDAR data from /obstacles3 topic

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

  bool odom_flag = false;  // check odom

  // TODO: why need to initialise here?
  double current_steering_angle = 0;

  dynamic_reconfigure::Server<agv::collision_detector_Config> server;
  dynamic_reconfigure::Server<agv::collision_detector_Config>::CallbackType f;

  // Callbacks
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void obstacle3Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void steeringAngleCallback(const std_msgs::Float64::ConstPtr& steering_angle_msg);

  // Functions
  void mainTimerCallback(const ros::TimerEvent& timer_event);
  void detectObstaclesInPredictedPath();
  void projectDynamicObstaclePath(agv::common::CircleObstacle& dynamic_obstacle);
  void publishDynamicObstaclePath();

  agv::common::VehicleState getLinkPosition(const agv::common::VehicleState& state, double link_length);
  agv::common::VehicleState getFrontAxleFromBaselink(const agv::common::VehicleState& baselink_state);
  agv::common::VehicleState getCOGFromBaselink(const agv::common::VehicleState& baselink_state);
  void publishPredictedTrajectoryAsPolygon(const agv::common::Path& path, double vehicle_width, double SAFETY_MARGIN,
                                           double height);
  std::vector<geometry_msgs::Point32> getBothSidesPoints(double x, double y, double yaw, double width, double margin,
                                                         double height);
};

// Constructor
CollisionDetector::CollisionDetector() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  // topics
  std::string collision_detector_topic_;
  std::string odom_topic_;
  std::string obstacle_topic_;
  std::string obstacle2_topic_;
  std::string obstacle3_topic_;
  std::string steering_angle_topic;
  std::string pred_traj_topic_;
  std::string dynamic_obstacle_path_topic_;
  std::string stopwall_marker_;
  std::string time_to_collision_topic_;
  double planning_frequency_;

  // Input Topics
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle2_topic", obstacle2_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle3_topic", obstacle3_topic_));
  ROS_ASSERT(private_nh.getParam("steering_angle_topic", steering_angle_topic));

  // Output Topics
  ROS_ASSERT(private_nh.getParam("collision_detector", collision_detector_topic_));
  ROS_ASSERT(private_nh.getParam("predicted_trajectory_topic", pred_traj_topic_));
  ROS_ASSERT(private_nh.getParam("dynamic_obstacle_path_topic", dynamic_obstacle_path_topic_));
  ROS_ASSERT(private_nh.getParam("time_to_collision_topic", time_to_collision_topic_));

  // Parameters
  ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency_));

  obstacle_sub = nh.subscribe(obstacle_topic_, 1, &CollisionDetector::obstacleCallback, this);
  obstacle2_sub = nh.subscribe(obstacle2_topic_, 1, &CollisionDetector::obstacle2Callback, this);
  obstacle3_sub = nh.subscribe(obstacle3_topic_, 1, &CollisionDetector::obstacle3Callback, this);
  odom_sub = nh.subscribe(odom_topic_, 1, &CollisionDetector::odomCallback, this);
  steering_angle_sub = nh.subscribe(steering_angle_topic, 1, &CollisionDetector::steeringAngleCallback, this);

  collision_detector_pub = nh.advertise<std_msgs::Int16>(collision_detector_topic_, 1);
  pred_traj_pub = nh.advertise<geometry_msgs::PolygonStamped>(pred_traj_topic_, 1);
  dynamic_obstacle_path_pub = nh.advertise<nav_msgs::Path>(dynamic_obstacle_path_topic_, 1);
  stopwall_pub = nh.advertise<visualization_msgs::Marker>(stopwall_marker_, 0);
  collision_time_pub = nh.advertise<std_msgs::Float64>(time_to_collision_topic_, 1);

  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);
  timer = nh.createTimer(ros::Duration(1.0 / planning_frequency_), &CollisionDetector::mainTimerCallback, this);

  ROS_INFO("Collision Detector: Ready.");
};

void CollisionDetector::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  detectObstaclesInPredictedPath();
}

void CollisionDetector::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
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
  odom_flag = true;
}

void CollisionDetector::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles.push_back(obstacle);
  }
}

void CollisionDetector::obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles2.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles2.push_back(obstacle);
  }
}

void CollisionDetector::obstacle3Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
  obstacles3.clear();
  for (auto circle : obstacle_msg->circles)
  {
    const agv::common::CircleObstacle obstacle = agv::common::CircleObstacle(circle);
    obstacles3.push_back(obstacle);
  }
}

void CollisionDetector::steeringAngleCallback(const std_msgs::Float64::ConstPtr& steering_angle_msg)
{
  current_steering_angle = steering_angle_msg->data;
}

void CollisionDetector::detectObstaclesInPredictedPath()
{
  std_msgs::Int16 collision_detector_msg;
  std_msgs::Float64 collision_time_msg;

  collision_detector_msg.data = NORMAL_SPEED;

  if (!odom_flag)
  {
    collision_detector_msg.data = STATIC_OBSTACLE_STOP;
    ROS_ERROR("Collision Detector: Odom not updated!!!");
    ROS_ERROR(" ");
    collision_detector_pub.publish(collision_detector_msg);
    return;
  }
  else if (obstacles.size() <= 0 && obstacles2.size() <= 0)
  {
    collision_detector_msg.data = STATIC_OBSTACLE_STOP;
    ROS_ERROR("Collision Detector: 2D & 3D obstacles are empty!!!");
    ROS_ERROR(" ");
    collision_detector_pub.publish(collision_detector_msg);
    return;
  }

  frontaxle_state_ = getFrontAxleFromBaselink(current_state_);
  COG_state_ = getCOGFromBaselink(current_state_);

  predicted_path.clear();
  predicted_path =
      projectFuturePath(frontaxle_state_, current_steering_angle, SPEED_THRESHOLD, LOOK_AHEAD_TIME, DELTA_T);
  publishPredictedTrajectoryAsPolygon(predicted_path, CIRCLE_RADIUS, SAFETY_MARGIN, 0.0);

  bool collision_flag;

  for (int i = 0; i < obstacles3.size(); i++)
  {
    double total_dist_from_baselink =
        agv::common::distance(current_state_.x, current_state_.y, obstacles3.at(i).state.x, obstacles3.at(i).state.y);
    double net_dist_from_baselink = total_dist_from_baselink - (CIRCLE_RADIUS + obstacles3.at(i).radius);
    double obstacle_speed = obstacles3.at(i).state.v;
    collision_flag = checkForCollision(net_dist_from_baselink, BACK_SAFETY_MARGIN);

    if (collision_flag && (obstacle_speed > current_state_.v) && current_state_.v >= 0.1)
    {
      collision_detector_msg.data = DYNAMIC_OBSTACLE_STOP;
      ROS_INFO("Collision distance: %f", net_dist_from_baselink);
      ROS_INFO("Dynamic obstacle detected behind!");
      projectDynamicObstaclePath(obstacles3.at(i));
      collision_detector_pub.publish(collision_detector_msg);
      return;
    }

    double total_dist_from_COG =
        agv::common::distance(COG_state_.x, COG_state_.y, obstacles3.at(i).state.x, obstacles3.at(i).state.y);
    double net_dist_from_COG = total_dist_from_COG - (CIRCLE_RADIUS + obstacles3.at(i).radius);
    collision_flag = checkForCollision(net_dist_from_COG, BACK_SAFETY_MARGIN);
    if (collision_flag && (obstacle_speed > current_state_.v) && current_state_.v >= 0.1)
    {
      collision_detector_msg.data = DYNAMIC_OBSTACLE_STOP;
      ROS_INFO("Collision distance: %f", net_dist_from_COG);
      ROS_INFO("Dynamic obstacle detected beside!");
      projectDynamicObstaclePath(obstacles3.at(i));
      collision_detector_pub.publish(collision_detector_msg);
      return;
    }
  }

  double closest_obstacle_distance = 100;
  double time_to_collision;

  for (int t = 0; t <= int(LOOK_AHEAD_TIME / DELTA_T); t++)
  {
    for (int i = 0; i < std::max(obstacles.size(), obstacles2.size()); i++)
    {
      double obs_dist = 100;
      double obs2_dist = 100;
      double obs_speed = 0;
      double obs2_speed = 0;

      if (i < obstacles.size())
      {
        double total_dist = agv::common::distance(predicted_path.x.at(t), predicted_path.y.at(t),
                                                  obstacles.at(i).state.x, obstacles.at(i).state.y);
        obs_dist = total_dist - (CIRCLE_RADIUS + obstacles.at(i).radius);
        obs_speed = obstacles.at(i).state.v;
      }

      if (i < obstacles2.size())
      {
        double total_dist = agv::common::distance(predicted_path.x.at(t), predicted_path.y.at(t),
                                                  obstacles2.at(i).state.x, obstacles2.at(i).state.y);
        obs2_dist = total_dist - (CIRCLE_RADIUS + obstacles2.at(i).radius);
        obs2_speed = obstacles2.at(i).state.v;
      }

      collision_flag = checkForCollision(obs_dist, obs2_dist, SAFETY_MARGIN);

      if (t <= COLLISION_TIME_THRESHOLD && collision_flag)
      {
        if (std::max(obs_speed, obs2_speed) >= DYNAMIC_OBSTACLE_SPEED)
        {
          collision_detector_msg.data = DYNAMIC_OBSTACLE_STOP;
          closest_obstacle_distance = std::min(obs_dist, obs2_dist);
        }
        else
        {
          collision_detector_msg.data = STATIC_OBSTACLE_STOP;
          closest_obstacle_distance = std::min(obs_dist, obs2_dist);
        }
        ROS_WARN("Collision distance: %f", closest_obstacle_distance);
        ROS_WARN("Time to collision: %f", t * DELTA_T);
        collision_detector_pub.publish(collision_detector_msg);
        collision_time_msg.data = t * DELTA_T;
        collision_time_pub.publish(collision_time_msg);
        return;
      }
      else if (t > COLLISION_TIME_THRESHOLD && collision_flag)
      {
        collision_detector_msg.data = NORMAL_SLOWDOWN;
        if (std::min(obs_dist, obs2_dist) < closest_obstacle_distance)
        {
          closest_obstacle_distance = std::min(obs_dist, obs2_dist);
          time_to_collision = t * DELTA_T;
        }
      }
    }
  }

  if (collision_detector_msg.data != NORMAL_SPEED)
  {
    ROS_WARN("Collision distance: %f", closest_obstacle_distance);
    ROS_WARN("Time to collision: %f", time_to_collision);
    collision_time_msg.data = time_to_collision;
    collision_time_pub.publish(collision_time_msg);
  }

  collision_detector_pub.publish(collision_detector_msg);
}

void CollisionDetector::projectDynamicObstaclePath(agv::common::CircleObstacle& dynamic_obstacle)
{
  agv::common::VehicleState predicted_state = dynamic_obstacle.state;

  dynamic_obstacle_path.clear();

  for (double t = 0; t <= LOOK_AHEAD_TIME; t += DELTA_T)
  {
    agv::common::VehicleState next_state;

    next_state.x = predicted_state.x + DELTA_T * dynamic_obstacle.vx;
    next_state.y = predicted_state.y + DELTA_T * dynamic_obstacle.vy;
    next_state.yaw = 0;

    dynamic_obstacle_path.x.push_back(next_state.x);
    dynamic_obstacle_path.y.push_back(next_state.y);
    dynamic_obstacle_path.yaw.push_back(next_state.yaw);

    predicted_state = next_state;
  }

  publishDynamicObstaclePath();
}

void CollisionDetector::publishDynamicObstaclePath()
{
  nav_msgs::Path dynamic_obstacle_path_msg;

  dynamic_obstacle_path_msg.header.frame_id = "map";

  for (int i = 0; i < dynamic_obstacle_path.x.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = dynamic_obstacle_path.x.at(i);
    pose.pose.position.y = dynamic_obstacle_path.y.at(i);
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(dynamic_obstacle_path.yaw.at(i));
    pose.pose.orientation = pose_quat;

    dynamic_obstacle_path_msg.poses.emplace_back(pose);
  }

  dynamic_obstacle_path_pub.publish(dynamic_obstacle_path_msg);
}

agv::common::VehicleState CollisionDetector::getLinkPosition(const agv::common::VehicleState& state, double link_length)
{
  agv::common::VehicleState new_state;

  new_state.x = state.x + cos(state.yaw) * link_length;
  new_state.y = state.y + sin(state.yaw) * link_length;
  new_state.yaw = state.yaw;
  new_state.v = state.v;

  return new_state;
}

agv::common::VehicleState CollisionDetector::getFrontAxleFromBaselink(const agv::common::VehicleState& baselink_state)
{
  return getLinkPosition(baselink_state, agv::common::Vehicle::L());
}

agv::common::VehicleState CollisionDetector::getCOGFromBaselink(const agv::common::VehicleState& baselink_state)
{
  return getLinkPosition(baselink_state, agv::common::Vehicle::Lr());
}

void CollisionDetector::publishPredictedTrajectoryAsPolygon(const agv::common::Path& path, double vehicle_width,
                                                            double SAFETY_MARGIN, double height)
{
  geometry_msgs::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = "map";

  std::vector<geometry_msgs::Point32> left_bound;
  std::vector<geometry_msgs::Point32> right_bound;
  std::vector<geometry_msgs::Point32> left_right_points;

  // baselink position
  left_right_points =
      getBothSidesPoints(current_state_.x, current_state_.y, current_state_.yaw, vehicle_width, SAFETY_MARGIN, height);
  left_bound.push_back(left_right_points.at(0));
  right_bound.push_back(left_right_points.at(1));

  // Find the rest of the projected trajectory from front axle
  for (int i = 0; i < path.x.size(); i++)
  {
    left_right_points =
        getBothSidesPoints(path.x.at(i), path.y.at(i), path.yaw.at(i), vehicle_width, SAFETY_MARGIN, height);
    left_bound.push_back(left_right_points.at(0));
    right_bound.push_back(left_right_points.at(1));
  }

  for (int i = 0; i < left_bound.size(); i++)
  {
    polygon_msg.polygon.points.emplace_back(left_bound.at(i));
  }
  for (int i = right_bound.size(); i > 0; i--)
  {
    polygon_msg.polygon.points.emplace_back(right_bound.at(i - 1));
  }

  pred_traj_pub.publish(polygon_msg);
}

std::vector<geometry_msgs::Point32> CollisionDetector::getBothSidesPoints(double x, double y, double yaw, double width,
                                                                          double margin, double height)
{
  geometry_msgs::Point32 left_point, right_point;

  // left being positive
  double left_direction = yaw + M_PI / 2.0;

  left_point.x = x + cos(left_direction) * (width + margin);
  left_point.y = y + sin(left_direction) * (width + margin);
  left_point.z = height;

  right_point.x = x - cos(left_direction) * (width + margin);
  right_point.y = y - sin(left_direction) * (width + margin);
  right_point.z = height;

  return { left_point, right_point };
}

}  // namespace behaviour_planner
}  // namespace agv

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_detector");
  agv::behaviour_planner::CollisionDetector collision_detector_obj;

  ros::spin();
  return 0;
}
