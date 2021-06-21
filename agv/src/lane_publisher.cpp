/** This node:
 *  1) Reads the lane info from the file and publishes to lane_info_topic
 *  2) Reads the lane markings from the file and publishes the lane boundaries
 *  3) Detects special waypoints and publishes the desired mode to special_waypoint_topic
 */

#include <fstream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <obstacle_detector/Obstacles.h>
#include <agv/Waypoint.h>
#include <agv/LaneInfo.h>
#include <agv/Behaviour.h>

#include <dynamic_reconfigure/server.h>
#include <agv/lane_publisher_Config.h>

#include "common/vehicle_state.h"

namespace lane_publisher
{

  double NEAR_TURNING_POINT_RADIUS;
  double END_POINT_STOP_RADIUS;
  double END_POINT_SLOWDOWN_RADIUS;

void dynamicParamCallback(agv::lane_publisher_Config& config, uint32_t level)
{
  NEAR_TURNING_POINT_RADIUS = config.near_turning_point_radius;
  END_POINT_STOP_RADIUS = config.end_point_stop_radius;
  END_POINT_SLOWDOWN_RADIUS = config.end_point_slowdown_radius;
}

class LanePublisher
{
public:
  LanePublisher();
  virtual ~LanePublisher(){};

private:
  ros::NodeHandle nh;

  ros::Subscriber odom_sub;
  ros::Subscriber path_filename_sub;

  ros::Publisher lane_info_pub;
  ros::Publisher left_boundary_xa_pub;
  ros::Publisher left_boundary_ax_pub;
  ros::Publisher right_boundary_pub;
  ros::Publisher centerline_pub;
  ros::Publisher waypoints_info_pub;
  ros::Publisher marker_pub;
  ros::Publisher special_waypoint_pub;

  ros::Timer timer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  std::string lanes_directory;
  std::string current_direction;

  enum desired_mode
  {
    DYNAMIC_OBSTACLE_STOP,
    STATIC_OBSTACLE_STOP,
    ENDPOINT_STOP,
    SLOPE_SLOWDOWN,
    NORMAL_SLOWDOWN,
    NORMAL_SPEED
  };

  geometry_msgs::Point desired_endpoint;
  std::vector<geometry_msgs::Point> list_of_turning_points;
  std::vector<geometry_msgs::Point> list_of_slope_points;

  dynamic_reconfigure::Server<agv::lane_publisher_Config> server;
  dynamic_reconfigure::Server<agv::lane_publisher_Config>::CallbackType f;

  agv::common::VehicleState current_state_;

  // Callbacks
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void laneFilenameCallback(const std_msgs::String::ConstPtr &path_filename_msg);

  // Functions
  void mainTimerCallback(const ros::TimerEvent &timer_event);
  void readLaneInfoFromFile();
  void publishLaneBoundaries();
  void publishCenterline();
  void publishLaneMarking(ros::Publisher pub, std::string file_name);
  void sendDesiredStoppingPointVisualization();
  void checkForSpecialWaypoints();
};

LanePublisher::LanePublisher() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  std::string odom_topic;
  std::string path_filename_topic;
  std::string lane_info_topic;
  std::string left_boundary_xa_topic;
  std::string left_boundary_ax_topic;
  std::string right_boundary_topic;
  std::string centerline_topic;
  std::string waypoints_info_topic;
  std::string special_waypoint;

  double planning_frequency;

  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
  ROS_ASSERT(private_nh.getParam("path_filename_topic", path_filename_topic));

  ROS_ASSERT(private_nh.getParam("lane_info_topic", lane_info_topic));
  ROS_ASSERT(private_nh.getParam("left_boundary_xa_topic", left_boundary_xa_topic));
  ROS_ASSERT(private_nh.getParam("left_boundary_ax_topic", left_boundary_ax_topic));
  ROS_ASSERT(private_nh.getParam("right_boundary_topic", right_boundary_topic));
  ROS_ASSERT(private_nh.getParam("centerline_topic", centerline_topic));
  ROS_ASSERT(private_nh.getParam("waypoints_info_topic", waypoints_info_topic));
  ROS_ASSERT(private_nh.getParam("special_waypoint", special_waypoint));

  ROS_ASSERT(private_nh.getParam("planning_frequency", planning_frequency));
  ROS_ASSERT(private_nh.getParam("lanes_directory", lanes_directory));

  odom_sub = nh.subscribe(odom_topic, 1, &LanePublisher::odomCallback, this);
  path_filename_sub = nh.subscribe(path_filename_topic, 1, &LanePublisher::laneFilenameCallback, this);

  lane_info_pub = nh.advertise<agv::LaneInfo>(lane_info_topic, 1, true);
  left_boundary_xa_pub = nh.advertise<nav_msgs::Path>(left_boundary_xa_topic, 1, true);
  left_boundary_ax_pub = nh.advertise<nav_msgs::Path>(left_boundary_ax_topic, 1, true);
  right_boundary_pub = nh.advertise<nav_msgs::Path>(right_boundary_topic, 1, true);
  centerline_pub = nh.advertise<nav_msgs::Path>(centerline_topic, 1, true);
  waypoints_info_pub = nh.advertise<geometry_msgs::PoseArray>(waypoints_info_topic, 1, true);
  special_waypoint_pub = nh.advertise<std_msgs::Int16>(special_waypoint, 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  timer = nh.createTimer(ros::Duration(1.0 / planning_frequency), &LanePublisher::mainTimerCallback, this);

  ROS_INFO("Lane Publisher: Ready.");
}

void LanePublisher::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  current_state_.v = odom_msg->twist.twist.linear.x;

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
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

void LanePublisher::laneFilenameCallback(const std_msgs::String::ConstPtr &path_filename_msg)
{
  if (path_filename_msg->data == "vip3a.path")
  {
    current_direction = "_xa.lane";
  }
  else if (path_filename_msg->data == "vip3x.path")
  {
    current_direction = "_ax.lane";
  }

  readLaneInfoFromFile();
  publishLaneBoundaries();
  publishCenterline();
}

void LanePublisher::mainTimerCallback(const ros::TimerEvent &timer_event)
{
  checkForSpecialWaypoints();
}

void LanePublisher::readLaneInfoFromFile()
{
  std::ifstream in_file;
  std::string lane_filename = "lane_info"; // change filename
  std::string lane_info_path = lanes_directory + lane_filename + current_direction;

  in_file.open(lane_info_path);

  if (!in_file)
  {
    ROS_ERROR("Lane Publisher: Could not open %s", lane_filename.c_str());
    return;
  }

  ROS_INFO("Lane Publisher: Reading %s", lane_filename.c_str());

  agv::LaneInfo lane_info;
  geometry_msgs::PoseArray waypoints_info;
  waypoints_info.header.frame_id = "map";
  geometry_msgs::Point turning_point;
  geometry_msgs::Point slope_point;

  list_of_turning_points.clear();
  list_of_slope_points.clear();

  in_file >> lane_info.num_waypoints;
  for (int i = 0; i < lane_info.num_waypoints; i++)
  {
    agv::Waypoint waypoint_data;

    in_file >> waypoint_data.x >> waypoint_data.y >> waypoint_data.dx >> waypoint_data.dy >> waypoint_data.s >>
        waypoint_data.left_width >> waypoint_data.right_width >> waypoint_data.far_right_width >>
        waypoint_data.special_point;

    lane_info.waypoints.push_back(waypoint_data);

    switch (waypoint_data.special_point)
    {
    case 0: // not a special waypoint
      break;

    case 1: // endpoint
      desired_endpoint.x = waypoint_data.x;
      desired_endpoint.y = waypoint_data.y;
      break;

    case 2: // turning point
      turning_point.x = waypoint_data.x;
      turning_point.y = waypoint_data.y;
      list_of_turning_points.emplace_back(turning_point);
      break;

    case 3: // slope point
      slope_point.x = waypoint_data.x;
      slope_point.y = waypoint_data.y;
      list_of_slope_points.emplace_back(slope_point);
      break;
    }

    geometry_msgs::Pose pose;
    pose.position.x = waypoint_data.x;
    pose.position.y = waypoint_data.y;

    double theta = atan2(waypoint_data.dy, waypoint_data.dx) + M_PI / 2;
    if (theta > M_PI)
    {
      theta -= (2 * M_PI);
    }
    else if (theta < -M_PI)
    {
      theta += (2 * M_PI);
    }

    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(theta);
    pose.orientation = pose_quat;
    waypoints_info.poses.emplace_back(pose);
  }

  in_file.close();

  lane_info_pub.publish(lane_info);
  waypoints_info_pub.publish(waypoints_info);

  sendDesiredStoppingPointVisualization();

  ROS_INFO_STREAM("Lane Publisher: endpoint set at (" << desired_endpoint.x << ", " << desired_endpoint.y << ")");
  ROS_INFO_STREAM("Lane Publisher: list_of_turning_points initialized with " << list_of_turning_points.size()
                                                                             << " points");
  ROS_INFO("The points are: ");
  for (auto turn_point : list_of_turning_points)
  {
    ROS_INFO("(%f, %f)", turn_point.x, turn_point.y);
  }
  ROS_INFO_STREAM("Lane Publisher: list_of_slope_points initialized with " << list_of_slope_points.size()
                                                                           << " points");
  ROS_INFO("The points are: ");
  for (auto slope_point : list_of_slope_points)
  {
    ROS_INFO("(%f, %f)", slope_point.x, slope_point.y);
  }
}

void LanePublisher::publishLaneBoundaries()
{
  std::string left_boundary_xa_filename = "left_boundary_xa.lane";
  std::string left_boundary_ax_filename = "left_boundary_ax.lane";
  std::string right_boundary_filename = "right_boundary.lane";

  publishLaneMarking(left_boundary_xa_pub, left_boundary_xa_filename);
  publishLaneMarking(left_boundary_ax_pub, left_boundary_ax_filename);
  publishLaneMarking(right_boundary_pub, right_boundary_filename);
}

void LanePublisher::publishCenterline()
{
  std::string centerline_filename = "centerline" + current_direction;
  publishLaneMarking(centerline_pub, centerline_filename);
}

void LanePublisher::publishLaneMarking(ros::Publisher pub, std::string filename)
{
  std::ifstream in_file;
  std::string lane_path = lanes_directory + filename;

  in_file.open(lane_path);
  if (!in_file)
  {
    ROS_ERROR("Lane Publisher: Could not open %s", filename.c_str());
    return;
  }
  else
  {
    ROS_INFO("Lane Publisher: Reading %s", filename.c_str());
  }

  nav_msgs::Path lane_marking;
  lane_marking.header.frame_id = "map";

  double x, y;
  while ((in_file >> x) && (in_file >> y))
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(0);
    pose.pose.orientation = pose_quat;
    lane_marking.poses.emplace_back(pose);
  }

  pub.publish(lane_marking);

  in_file.close();
}

void LanePublisher::sendDesiredStoppingPointVisualization()
{
  visualization_msgs::Marker desired_endpoint_marker;
  desired_endpoint_marker.header.frame_id = "map";
  desired_endpoint_marker.header.stamp = ros::Time::now();
  desired_endpoint_marker.ns = "desired_endpoint";
  desired_endpoint_marker.id = 0;
  desired_endpoint_marker.action = visualization_msgs::Marker::ADD;
  desired_endpoint_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // desired_endpoint_marker.text = "END";
  desired_endpoint_marker.mesh_resource = "package://agv/meshes/endpoint.stl";

  desired_endpoint_marker.pose.position.x = desired_endpoint.x;
  desired_endpoint_marker.pose.position.y = desired_endpoint.y;
  desired_endpoint_marker.pose.position.z = 0;
  desired_endpoint_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, -2.7);

  desired_endpoint_marker.scale.x = 0.05;
  desired_endpoint_marker.scale.y = 0.05;
  desired_endpoint_marker.scale.z = 0.05;
  desired_endpoint_marker.color.r = 0.9f;
  desired_endpoint_marker.color.g = 0.0f;
  desired_endpoint_marker.color.b = 0.0f;
  desired_endpoint_marker.color.a = 1.0;
  desired_endpoint_marker.lifetime = ros::Duration();

  marker_pub.publish(desired_endpoint_marker);
}

void LanePublisher::checkForSpecialWaypoints()
{
  if (list_of_turning_points.size() == 0 && list_of_slope_points.size() == 0)
  {
    ROS_WARN("Lane Publisher: Turning and slope points are empty!");
    return;
  }

  std_msgs::Int16 special_waypoint_msg;
  special_waypoint_msg.data = NORMAL_SPEED;
  bool slope_flag = 0;

  for (geometry_msgs::Point slope_point : list_of_slope_points)
  {
    double slope_X = slope_point.x;
    double slope_Y = slope_point.y;

    // use the squared distance for comparision instead of hypot if need to improve performance
    if (std::hypot((current_state_.x - slope_X), (current_state_.y - slope_Y)) <= NEAR_TURNING_POINT_RADIUS)
    {
      special_waypoint_msg.data = SLOPE_SLOWDOWN;
      slope_flag = 1;
      special_waypoint_pub.publish(special_waypoint_msg);
      break;
    }
  }

  if (!slope_flag)
  {
    for (geometry_msgs::Point turning_point : list_of_turning_points)
    {
      double turning_X = turning_point.x;
      double turning_Y = turning_point.y;

      // use the squared distance for comparision instead of hypot if need to improve performance
      if (std::hypot((current_state_.x - turning_X), (current_state_.y - turning_Y)) <= NEAR_TURNING_POINT_RADIUS)
      {
        special_waypoint_msg.data = NORMAL_SLOWDOWN;
        special_waypoint_pub.publish(special_waypoint_msg);
        break;
      }
    }
  }

  double dist_to_stopping_point = std::hypot((current_state_.x - desired_endpoint.x), (current_state_.y - desired_endpoint.y));
  if (dist_to_stopping_point <= END_POINT_STOP_RADIUS)
  {
    special_waypoint_msg.data = STATIC_OBSTACLE_STOP;
  }
  else if (dist_to_stopping_point <= END_POINT_SLOWDOWN_RADIUS)
  {
    special_waypoint_msg.data = NORMAL_SLOWDOWN;
  }

  special_waypoint_pub.publish(special_waypoint_msg);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_publisher_node");
  lane_publisher::LanePublisher lane_publisher_obj;
  ros::spin();
  return 0;
}
