#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

enum class RvizBaselinkVisualizationColours
{
  MICRON_BLUE,
  RED,
  WHITE,
  ORANGE,
  YELLOW
};

class RvizBaselinkVisualization
{
public:
  RvizBaselinkVisualization();

private:
  ros::NodeHandle nh;

  ros::Subscriber odom_sub;
  // ros::Subscriber amcl_sub;
  ros::Subscriber nav_cmd_sub;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber joy_sub;
  ros::Publisher marker_pub;

  visualization_msgs::Marker amcl_marker;

  double TURNING_POINT_SLOWDOWN_SPEED;
  double OBSTACLE_SLOWDOWN_SPEED;

  bool autonomous_mode_activated = false;
  bool brake_flag_activated = false;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  // void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg);
  void navCmdVelCallback(const geometry_msgs::Twist::ConstPtr& nav_cmd_vel_msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void initializeAMCLMarker();
  void updateMarkerColour(RvizBaselinkVisualizationColours colour);
};

RvizBaselinkVisualization::RvizBaselinkVisualization() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  std::string odom_topic_ = "/odometry/filtered";
  // std::string amcl_topic = "/amcl_pose";
  std::string joy_topic = "/joy";
  
  std::string cmd_vel_topic = "/cmd_vel_out";
  std::string nav_cmd_topic = "/nav_cmd_vel";

  ROS_ASSERT(private_nh.getParam("turning_point_slowdown_speed", TURNING_POINT_SLOWDOWN_SPEED));
  ROS_ASSERT(private_nh.getParam("obstacle_slowdown_speed", OBSTACLE_SLOWDOWN_SPEED));

  odom_sub = nh.subscribe(odom_topic_, 1, &RvizBaselinkVisualization::odomCallback, this);
  // amcl_sub = nh.subscribe(amcl_topic, 1, &RvizBaselinkVisualization::amclCallback, this);
  nav_cmd_sub = nh.subscribe(nav_cmd_topic, 1, &RvizBaselinkVisualization::navCmdVelCallback, this);
  cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1, &RvizBaselinkVisualization::cmdVelCallback, this);
  joy_sub = nh.subscribe(joy_topic, 1, &RvizBaselinkVisualization::joyCallback, this);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  initializeAMCLMarker();
}

void RvizBaselinkVisualization::initializeAMCLMarker()
{
  amcl_marker.header.frame_id = "map";
  amcl_marker.header.stamp = ros::Time::now();
  amcl_marker.ns = "localization_marker";
  amcl_marker.id = 0;
  amcl_marker.action = visualization_msgs::Marker::ADD;
  amcl_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  amcl_marker.mesh_resource = "package://agv/meshes/buggy.stl"; //arrow

  amcl_marker.pose.position.z = 0;

  amcl_marker.scale.x = 0.01; //0.001;
  amcl_marker.scale.y = 0.01; //0.001;
  amcl_marker.scale.z = 0.01; //0.001;
  updateMarkerColour(RvizBaselinkVisualizationColours::WHITE);
  amcl_marker.color.a = 0.75;
  amcl_marker.lifetime = ros::Duration();
}

void RvizBaselinkVisualization::updateMarkerColour(RvizBaselinkVisualizationColours colour)
{
  switch (colour)
  {
    case RvizBaselinkVisualizationColours::MICRON_BLUE:
      amcl_marker.color.r = 0.0f;
      amcl_marker.color.g = 0.466f;
      amcl_marker.color.b = 0.784f;
      break;
    case RvizBaselinkVisualizationColours::RED:
      amcl_marker.color.r = 1.0f;
      amcl_marker.color.g = 0.0f;
      amcl_marker.color.b = 0.0f;
      break;
    case RvizBaselinkVisualizationColours::WHITE:
      amcl_marker.color.r = 1.0f;
      amcl_marker.color.g = 1.0f;
      amcl_marker.color.b = 1.0f;
      break;
    case RvizBaselinkVisualizationColours::ORANGE:
      amcl_marker.color.r = 1.0f;
      amcl_marker.color.g = 0.4f;
      amcl_marker.color.b = 0.0f;
      break;
    case RvizBaselinkVisualizationColours::YELLOW:
      amcl_marker.color.r = 1.0f;
      amcl_marker.color.g = 0.8f;
      amcl_marker.color.b = 0.0f;
      break;
    default:
      ROS_DEBUG("RvizBaselinkVisualization: Undefined Colour!");
  }
}

void RvizBaselinkVisualization::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons[1] == 1)
  {
    brake_flag_activated = true;
    autonomous_mode_activated = false;
  }
  else if (joy_msg->buttons[2] == 1 || joy_msg->buttons[5] == 1)
  {
    brake_flag_activated = false;
    autonomous_mode_activated = false;
  }
  else if (joy_msg->buttons[0] == 1 && brake_flag_activated)
  {
    brake_flag_activated = false;
    autonomous_mode_activated = true;
  }
}

void RvizBaselinkVisualization::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  // current_speed = odom_msg->twist.twist.linear.x;

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
  // tf::Matrix3x3 m(q);
  // double roll, pitch;
  // m.getRPY(roll, pitch, current_yaw);

  // Current XY of robot (map frame)
  amcl_marker.pose.position.x = pose_after_transform.pose.position.x;
  amcl_marker.pose.position.y = pose_after_transform.pose.position.y;
  amcl_marker.pose.orientation = pose_after_transform.pose.orientation;

  marker_pub.publish(amcl_marker);
}

// void RvizBaselinkVisualization::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg)
// {
//   amcl_marker.pose.position.x = amcl_msg->pose.pose.position.x;
//   amcl_marker.pose.position.y = amcl_msg->pose.pose.position.y;

//     // geometry_msgs::TransformStamped tranform_stamped;
//     // tranform_stamped.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
//     // geometry_msgs::Quaternion orientation_after_transform;
//     // geometry_msgs::Quaternion orientation_before_transform = amcl_msg->pose.pose.orientation;
//     // tf2::doTransform(orientation_before_transform, orientation_after_transform, tranform_stamped);

//   amcl_marker.pose.orientation = amcl_msg->pose.pose.orientation;

//   marker_pub.publish(amcl_marker);
// }

void RvizBaselinkVisualization::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
  if (autonomous_mode_activated)
  {
    return;
  }

  if (cmd_vel_msg->linear.z == 1.0)
  {
    updateMarkerColour(RvizBaselinkVisualizationColours::RED);
  }
  else if (cmd_vel_msg->linear.y == 1.0)
  {
    updateMarkerColour(RvizBaselinkVisualizationColours::ORANGE);
  }
  else
  {
    updateMarkerColour(RvizBaselinkVisualizationColours::WHITE);
  }

  marker_pub.publish(amcl_marker);
}

void RvizBaselinkVisualization::navCmdVelCallback(const geometry_msgs::Twist::ConstPtr& nav_cmd_vel_msg)
{
  if (!autonomous_mode_activated)
  {
    return;
  }

  if (nav_cmd_vel_msg->linear.x == TURNING_POINT_SLOWDOWN_SPEED || nav_cmd_vel_msg->linear.x == OBSTACLE_SLOWDOWN_SPEED)
  {
    updateMarkerColour(RvizBaselinkVisualizationColours::YELLOW);
  }
  else if (nav_cmd_vel_msg->linear.z == 1.0)
  {
    updateMarkerColour(RvizBaselinkVisualizationColours::RED);
  }
  else
  {
    updateMarkerColour(RvizBaselinkVisualizationColours::MICRON_BLUE);
  }

  marker_pub.publish(amcl_marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_baselink_visualization_node");
  RvizBaselinkVisualization rviz_baselink_visualization_obj;
  ros::spin();
  return 0;
}
