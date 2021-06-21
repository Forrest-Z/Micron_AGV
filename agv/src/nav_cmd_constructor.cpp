#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <fstream>

class NavCmdConstructor
{
public:
  NavCmdConstructor();

private:
  ros::NodeHandle nh;
  ros::Subscriber desired_steering_angle_sub;
  ros::Subscriber closest_obstacle_dist_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber path_filename_sub;
  ros::Publisher nav_cmd_pub;
  ros::Publisher marker_pub;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  geometry_msgs::Twist nav_cmd_msg;
  double last_velocity_published;

  double NORMAL_CRUISE_SPEED;
  double TURNING_POINT_SLOWDOWN_SPEED;
  double OBSTACLE_SLOWDOWN_SPEED;
  double MEDIUM_STEERING_ANGLE_SLOWDOWN_SPEED;
  double LARGE_STEERING_ANGLE_SLOWDOWN_SPEED;

  double OBSTACLE_STOP_RADIUS;
  double OBSTACLE_SLOWDOWN_RADIUS;
  double NEAR_TURNING_POINT_RADIUS;

  double END_POINT_STOP_RADIUS;
  double END_POINT_SLOWDOWN_RADIUS;

  std::string landmarks_directory;

  std::vector<geometry_msgs::Point> list_of_turning_points;
  double desired_speed_from_odom_pose;
  double current_speed;

  geometry_msgs::Point desired_end_point;

  double desired_steering_angle;
  double desired_speed_from_steering_angle;

  double desired_speed_from_obstacle_distance;
  bool soft_brake_flag;
  bool hard_brake_flag;
  bool near_stop = false;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void desiredSteeringAngleCallback(const std_msgs::Float64::ConstPtr& desired_steering_angle_msg);
  void nearestObstacleDistCallback(const std_msgs::Float64::ConstPtr& closest_obstacle_dist_msg);
  void pathFilenameCallback(const std_msgs::String::ConstPtr& path_filename_msg);
  void publishNavCmd();

  void updateLandmarks(std::string file_path);
  void sendDesiredStoppingPointVisualization();
  void initializeStartingVariables();
  bool checkIfSoftbrakeRequired();
};

NavCmdConstructor::NavCmdConstructor() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  std::string desired_steering_angle_topic;
  std::string closest_obstacle_distance_topic;
  std::string odom_topic;
  std::string nav_cmd_topic;
  std::string path_filename_topic;

  ROS_ASSERT(private_nh.getParam("desired_steering_angle_topic", desired_steering_angle_topic));
  ROS_ASSERT(private_nh.getParam("closest_obstacle_distance_topic", closest_obstacle_distance_topic));
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
  ROS_ASSERT(private_nh.getParam("nav_cmd_topic", nav_cmd_topic));
  ROS_ASSERT(private_nh.getParam("path_filename_topic", path_filename_topic));
  ROS_ASSERT(private_nh.getParam("normal_cruise_speed", NORMAL_CRUISE_SPEED));
  ROS_ASSERT(private_nh.getParam("turning_point_slowdown_speed", TURNING_POINT_SLOWDOWN_SPEED));
  ROS_ASSERT(private_nh.getParam("obstacle_slowdown_speed", OBSTACLE_SLOWDOWN_SPEED));
  ROS_ASSERT(private_nh.getParam("medium_steering_angle_slowdown_speed", MEDIUM_STEERING_ANGLE_SLOWDOWN_SPEED));
  ROS_ASSERT(private_nh.getParam("large_steering_angle_slowdown_speed", LARGE_STEERING_ANGLE_SLOWDOWN_SPEED));
  ROS_ASSERT(private_nh.getParam("obstacle_stop_radius", OBSTACLE_STOP_RADIUS));
  ROS_ASSERT(private_nh.getParam("obstacle_slowdown_radius", OBSTACLE_SLOWDOWN_RADIUS));
  ROS_ASSERT(private_nh.getParam("near_turning_point_radius", NEAR_TURNING_POINT_RADIUS));
  ROS_ASSERT(private_nh.getParam("end_point_stop_radius", END_POINT_STOP_RADIUS));
  ROS_ASSERT(private_nh.getParam("end_point_slowdown_radius", END_POINT_SLOWDOWN_RADIUS));

  odom_sub = nh.subscribe(odom_topic, 1, &NavCmdConstructor::odomCallback, this);
  desired_steering_angle_sub =
      nh.subscribe(desired_steering_angle_topic, 1, &NavCmdConstructor::desiredSteeringAngleCallback, this);
  closest_obstacle_dist_sub =
      nh.subscribe(closest_obstacle_distance_topic, 1, &NavCmdConstructor::nearestObstacleDistCallback, this);
  path_filename_sub = nh.subscribe(path_filename_topic, 1, &NavCmdConstructor::pathFilenameCallback, this);
  nav_cmd_pub = nh.advertise<geometry_msgs::Twist>(nav_cmd_topic, 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  ROS_ASSERT(private_nh.getParam("landmarks_directory", landmarks_directory));

  initializeStartingVariables();

  ROS_INFO("NavCmdConstructor: Ready.");
}

void NavCmdConstructor::initializeStartingVariables()
{
  last_velocity_published = NORMAL_CRUISE_SPEED;
  desired_speed_from_odom_pose = NORMAL_CRUISE_SPEED;
  desired_speed_from_steering_angle = NORMAL_CRUISE_SPEED;
  desired_speed_from_obstacle_distance = NORMAL_CRUISE_SPEED;
  desired_steering_angle = 0;
  current_speed = 0;
  soft_brake_flag = false;
  hard_brake_flag = false;
}

void NavCmdConstructor::sendDesiredStoppingPointVisualization()
{
  visualization_msgs::Marker desired_end_point_marker;
  desired_end_point_marker.header.frame_id = "map";
  desired_end_point_marker.header.stamp = ros::Time::now();
  desired_end_point_marker.ns = "desired_end_point";
  desired_end_point_marker.id = 0;
  desired_end_point_marker.action = visualization_msgs::Marker::ADD;
  desired_end_point_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // desired_end_point_marker.text = "END";
  desired_end_point_marker.mesh_resource = "package://agv/meshes/endpoint.stl";

  desired_end_point_marker.pose.position.x = desired_end_point.x;
  desired_end_point_marker.pose.position.y = desired_end_point.y;
  desired_end_point_marker.pose.position.z = 0;
  desired_end_point_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, -2.7);

  desired_end_point_marker.scale.x = 0.05;
  desired_end_point_marker.scale.y = 0.05;
  desired_end_point_marker.scale.z = 0.05;
  desired_end_point_marker.color.r = 0.9f;
  desired_end_point_marker.color.g = 0.0f;
  desired_end_point_marker.color.b = 0.0f;
  desired_end_point_marker.color.a = 1.0;
  desired_end_point_marker.lifetime = ros::Duration();

  marker_pub.publish(desired_end_point_marker);
}

void NavCmdConstructor::updateLandmarks(std::string file_path)
{
  std::ifstream in_file;
  in_file.open(file_path);
  if (!in_file)
  {
    ROS_ERROR("NavCmdConstructor: Could not open landmarks file!!!");
    return;
  }
  else
  {
    ROS_INFO("NavCmdConstructor: Reading a landmarks file.");
  }

  //Do not forget to clear the turning points.
  list_of_turning_points.clear();

  bool point_is_endpoint_flag = true;
  geometry_msgs::Point turning_point;
  double x, y;
  while ((in_file >> x) && (in_file >> y))
  {
    if (point_is_endpoint_flag == true)
    {
      desired_end_point.x = x;
      desired_end_point.y = y;
      point_is_endpoint_flag = false;
      ROS_DEBUG_STREAM(
        "NavCmdConstructor: end point set at (" 
        << desired_end_point.x << ", " << desired_end_point.y
        << ")");
    }
    else
    {
      turning_point.x = x;
      turning_point.y = y;
      list_of_turning_points.emplace_back(turning_point);
    }
   
  }
  in_file.close();

  ROS_DEBUG_STREAM("NavCmdConstructor: list_of_turning_points initialized with " << list_of_turning_points.size()
                                                                                 << " points");
  ROS_DEBUG("The points are :");
  for (auto turn_point : list_of_turning_points){
    ROS_DEBUG("(%f, %f)", turn_point.x, turn_point.y);
  }
}

void NavCmdConstructor::publishNavCmd()
{
  nav_cmd_msg.linear.x = std::min(desired_speed_from_odom_pose,
                                  std::min(desired_speed_from_steering_angle, desired_speed_from_obstacle_distance));

  soft_brake_flag = checkIfSoftbrakeRequired();

  // nav_cmd_msg.linear.y = (soft_brake_flag) ? 1.0 : 0;
  nav_cmd_msg.linear.y = 0;
  nav_cmd_msg.linear.z = (hard_brake_flag || near_stop) ? 1.0 : 0;

  nav_cmd_msg.angular.z = (near_stop) ? 0 : desired_steering_angle;

  nav_cmd_pub.publish(nav_cmd_msg);
  last_velocity_published = nav_cmd_msg.linear.x;
}

bool NavCmdConstructor::checkIfSoftbrakeRequired()
{
  if (soft_brake_flag == true)
    return true;
  if (hard_brake_flag == true)
    return false;
  if (nav_cmd_msg.linear.x > (current_speed + 1.0))
    return true;
  return false;
}

void NavCmdConstructor::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_speed = odom_msg->twist.twist.linear.x;

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

  // transform from 'pose' (odom frame) to 'pose_after_transform' (map frame)
  geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
  pose_before_transform.header.frame_id = odom_msg->header.frame_id;
  pose_before_transform.header.stamp = odom_msg->header.stamp;
  pose_before_transform.pose = odom_msg->pose.pose;
  tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

  double current_X = pose_after_transform.pose.position.x;
  double current_Y = pose_after_transform.pose.position.y;

  desired_speed_from_odom_pose = NORMAL_CRUISE_SPEED;
  for (geometry_msgs::Point turning_point : list_of_turning_points)
  {
    double turning_X = turning_point.x;
    double turning_Y = turning_point.y;

    // use the squared distance for comparision instead of hypot if need to improve performance
    if (std::hypot((current_X - turning_X), (current_Y - turning_Y)) <= NEAR_TURNING_POINT_RADIUS)
    {
      ROS_DEBUG("Nav Cmd Constructor: Turning point detected!");
      desired_speed_from_odom_pose = TURNING_POINT_SLOWDOWN_SPEED;
      if (desired_speed_from_odom_pose < last_velocity_published)
      {
        publishNavCmd();
      }
      break;
    }
  }

  double dist_to_stopping_point = std::hypot((current_X - desired_end_point.x), (current_Y - desired_end_point.y));
  if (dist_to_stopping_point <= END_POINT_STOP_RADIUS)
  {
    ROS_DEBUG("NavCmdConstructor: Destination reached!");
    desired_speed_from_odom_pose = 0;
    near_stop = true;
    publishNavCmd();
  }
  else if (dist_to_stopping_point <= END_POINT_SLOWDOWN_RADIUS)
  {
    ROS_DEBUG("NavCmdConstructor: Reaching destination!");
    desired_speed_from_odom_pose = std::min(desired_speed_from_odom_pose, OBSTACLE_SLOWDOWN_SPEED);  // stub
    near_stop = false;
    publishNavCmd();
  }
  else
  {
    near_stop = false;
  }
}

void NavCmdConstructor::desiredSteeringAngleCallback(const std_msgs::Float64::ConstPtr& desired_steering_angle_msg)
{
  desired_steering_angle = desired_steering_angle_msg->data;

  if (std::abs(desired_steering_angle) > 0.17)  // 10 deg
  {
    desired_speed_from_steering_angle = LARGE_STEERING_ANGLE_SLOWDOWN_SPEED;
  }
  else if (std::abs(desired_steering_angle) > 0.085)  // 5 deg
  {
    desired_speed_from_steering_angle = MEDIUM_STEERING_ANGLE_SLOWDOWN_SPEED;
  }
  else
  {
    desired_speed_from_steering_angle = NORMAL_CRUISE_SPEED;
  }

  publishNavCmd();
}

void NavCmdConstructor::nearestObstacleDistCallback(const std_msgs::Float64::ConstPtr& closest_obstacle_dist_msg)
{
  double closest_obstacle_distance = closest_obstacle_dist_msg->data;

  if (closest_obstacle_distance < OBSTACLE_STOP_RADIUS)
  {
    hard_brake_flag = true;
    soft_brake_flag = false;
    desired_speed_from_obstacle_distance = 0;
  }
  else if (closest_obstacle_distance < OBSTACLE_SLOWDOWN_RADIUS)
  {
    hard_brake_flag = false;
    soft_brake_flag = false;
    // TODO: determine the slowdown speed, note that this may run at up to 40hz
    // desired_speed_from_obstacle_distance = std::max((desired_speed_from_obstacle_distance - 0.1), 0);
    desired_speed_from_obstacle_distance = OBSTACLE_SLOWDOWN_SPEED;
  }
  else
  {
    hard_brake_flag = false;
    soft_brake_flag = false;
    desired_speed_from_obstacle_distance = NORMAL_CRUISE_SPEED;
  }

  if (desired_speed_from_obstacle_distance < last_velocity_published)
  {
    publishNavCmd();
  }
}

void NavCmdConstructor::pathFilenameCallback(const std_msgs::String::ConstPtr& path_filename_msg)
{
  std::string path_filename = path_filename_msg->data;
  if (path_filename.length() < 5)
  {
    ROS_ERROR("NavCmdConstructor: invalid path filename length!");
    return;
  }
  std::string path_name = path_filename.substr(0, path_filename.length() - 5);
  std::string path_filename_extension = path_filename.substr(path_filename.length() - 5, path_filename.length());
  if (path_filename_extension.compare(".path") != 0)
  {
    ROS_ERROR("NavCmdConstructor: invalid path extension!");
    return;
  }
  ROS_INFO_STREAM("NavCmdConstructor: reading " << path_name << ".landmarks");
  updateLandmarks(landmarks_directory + path_name + ".landmarks");
  sendDesiredStoppingPointVisualization();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_cmd_constructor_node");
  NavCmdConstructor nav_cmd_constructor_obj;
  ros::spin();
  return 0;
}