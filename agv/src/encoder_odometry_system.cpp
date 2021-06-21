/*
 * This program outputs the odometry information based on
 * rate of change of wheel encoders and dt
 *
 * Standard units of measurement are:
 * Distance => Metres
 * Time => Seconds
 * Angle => Radians
 *
 * Includes: IMU
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <agv/EncoderCount.h>
#include <cmath>

#define RADIAN_TO_DEG_CONST 57.295779513
#define DEG_TO_RADIAN_CONST 0.01745329251

class EncoderOdometrySystem
{
public:
  EncoderOdometrySystem();

private:
  ros::NodeHandle nh;

  ros::Subscriber encoder_count_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber yaw_manual_update_sub;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;

  nav_msgs::Odometry odom_msg;

  double ENCODER_TO_DIST = 0;
  double AXLE_TRACK = 0.95;  // measured from centre to centre of rear wheels. 1.13m if end to end

  double left_encoder_count = 0;
  double right_encoder_count = 0;
  double encoder_x_coordinate;
  double encoder_y_coordinate;
  double current_imu_yaw;
  double previous_imu_yaw;

  bool getEncoderToDistConst(double BUGGY_UNIT_NUMBER);
  void encoderCallback(const agv::EncoderCount::ConstPtr& encoder_msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void publish_to_odom_topic();
};

EncoderOdometrySystem::EncoderOdometrySystem()
{
  ros::NodeHandle private_nh("~");

  std::string encoder_count_topic;
  std::string imu_topic;
  std::string odom_from_encoder_topic;

  ROS_ASSERT(private_nh.getParam("encoder_count_topic", encoder_count_topic));
  ROS_ASSERT(private_nh.getParam("imu_topic", imu_topic));
  ROS_ASSERT(private_nh.getParam("odom_from_encoder_topic", odom_from_encoder_topic));
  ROS_ASSERT(private_nh.getParam("encoder_x_coordinate", encoder_x_coordinate));
  ROS_ASSERT(private_nh.getParam("encoder_y_coordinate", encoder_y_coordinate));

  double BUGGY_UNIT_NUMBER = 0;
  ROS_ASSERT(private_nh.getParam("buggy_unit_number", BUGGY_UNIT_NUMBER));
  ROS_ASSERT(getEncoderToDistConst(BUGGY_UNIT_NUMBER));
  ROS_ASSERT(ENCODER_TO_DIST != 0);

  encoder_count_sub =
      nh.subscribe<agv::EncoderCount>(encoder_count_topic, 1, &EncoderOdometrySystem::encoderCallback, this);
  imu_sub = nh.subscribe(imu_topic, 1, &EncoderOdometrySystem::imuCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_from_encoder_topic, 1);

  odom_msg.header.frame_id = "odom";
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.pose.pose.position.x = encoder_x_coordinate;
  odom_msg.pose.pose.position.y = encoder_y_coordinate;
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.angular.z = 0;
  publish_to_odom_topic();
}

bool EncoderOdometrySystem::getEncoderToDistConst(double BUGGY_UNIT_NUMBER)
{
  if (BUGGY_UNIT_NUMBER == 1.0)
  {
    /*
     * ENCODER_TO_DIST = gear ratio * PI * wheel diameter / encoder count per rev / encoder library quarter cycle /
     * bias = (12 / 74) * PI * 0.508 / 1000 / 4 / 1.04 = 0.00006231137
     */
    ENCODER_TO_DIST = 0.00006277870;
  }
  else if (BUGGY_UNIT_NUMBER == 2.0)
  {
    ENCODER_TO_DIST = 0.00064975359;
  }
  else if (BUGGY_UNIT_NUMBER == 3.0)
  {
    ENCODER_TO_DIST = 0.000063787957;
  }
  else
  {
    ROS_ERROR("Encoder Odometry System: Invalid buggy unit number.");
    return false;
  }
  return true;

}

void EncoderOdometrySystem::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  previous_imu_yaw = current_imu_yaw;
  tf::Quaternion q_imu(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
  tf::Matrix3x3 m_imu(q_imu);
  double roll, pitch;
  m_imu.getRPY(roll, pitch, current_imu_yaw);
}

void EncoderOdometrySystem::encoderCallback(const agv::EncoderCount::ConstPtr& msg)
{
  /*
   * Updates the difference in counts in dt time
   * Unit of counts is number of revolutions of encoders
   * Unit of dt is seconds, typ 0.1s
   */
  left_encoder_count = msg->left;
  right_encoder_count = msg->right;
  double dt = msg->dt;

  double dist_right = right_encoder_count * ENCODER_TO_DIST;  // = rev * dist/rev (unit in m)
  double dist_left = left_encoder_count * ENCODER_TO_DIST;
  double dist_average = (dist_left + dist_right) / 2.0;
  double d_theta = current_imu_yaw - previous_imu_yaw;
  double mid_theta = (current_imu_yaw + previous_imu_yaw) / 2.0;

  double velocity_instantaneous = dist_average / dt;
  double d_x = cos(d_theta) * dist_average;
  double d_y = sin(d_theta) * dist_average;
  encoder_x_coordinate += d_x * cos(previous_imu_yaw) - d_y * sin(previous_imu_yaw);
  encoder_y_coordinate += d_x * sin(previous_imu_yaw) + d_y * cos(previous_imu_yaw);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_imu_yaw);

  // publish the transform (if not using ekf's transform)
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = ros::Time::now();
  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";
  // odom_trans.transform.translation.x = encoder_x_coordinate;
  // odom_trans.transform.translation.y = encoder_y_coordinate;
  // odom_trans.transform.translation.z = 0.0;
  // odom_trans.transform.rotation = odom_quat;
  // odom_broadcaster.sendTransform(odom_trans);

  // publish the message
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.pose.pose.position.x = encoder_x_coordinate;
  odom_msg.pose.pose.position.y = encoder_y_coordinate;
  odom_msg.twist.twist.linear.x = velocity_instantaneous;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.angular.z = d_theta / dt;
  odom_msg.header.stamp = ros::Time::now();

  publish_to_odom_topic();
}

void EncoderOdometrySystem::publish_to_odom_topic()
{
  odom_pub.publish(odom_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "encoder_odometry_system_node");
  EncoderOdometrySystem encoder_odometry_system_obj;
  ros::spin();
  return 0;
}
