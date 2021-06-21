/*
 *
 * This node consolidates important data into 1 topic for easier debugging and monitoring
 *
 * Currently only publishes in:
 * 1) cmd_vel callback (either manual or autonomous cmd)
 * 2) IMU callback
 *
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <agv/DataReader.h>

#define RADIAN_TO_DEG_CONST 57.295779513

class DataReader
{
public:
  DataReader();

private:
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber odom_encoder_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber odom_filtered_sub;
  ros::Subscriber steering_angle_sub;
  ros::Subscriber nav_cmd_sub;
  ros::Publisher data_reader_pub;
  ros::Publisher current_speed_pub;

  agv::DataReader data_reader_msg;

  void cmdVelocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);
  void odomEncoderCallback(const nav_msgs::Odometry::ConstPtr& odom_encoder_msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void odomFilteredCallback(const nav_msgs::Odometry::ConstPtr& odom_filtered_msg);
  void steeringAnglecallback(const std_msgs::Float64::ConstPtr& stanley_steering_angle_msg);
  void navCmdVelCallback(const geometry_msgs::Twist::ConstPtr& nav_cmd_vel_msg);
};

DataReader::DataReader()
{
  ros::NodeHandle private_nh("~");

  std::string cmd_vel_topic;
  std::string odom_encoder_topic;
  std::string imu_topic;
  std::string odom_filtered_topic;
  std::string steering_angle_topic;
  std::string nav_cmd_topic;
  std::string data_reader_topic;

  ROS_ASSERT(private_nh.getParam("cmd_vel_topic", cmd_vel_topic));
  ROS_ASSERT(private_nh.getParam("odom_encoder_topic", odom_encoder_topic));
  ROS_ASSERT(private_nh.getParam("imu_topic", imu_topic));
  ROS_ASSERT(private_nh.getParam("odom_filtered_topic", odom_filtered_topic));
  ROS_ASSERT(private_nh.getParam("steering_angle_topic", steering_angle_topic));
  ROS_ASSERT(private_nh.getParam("nav_cmd_topic", nav_cmd_topic));
  ROS_ASSERT(private_nh.getParam("data_reader_topic", data_reader_topic));

  cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1, &DataReader::cmdVelocityCallback, this);
  odom_encoder_sub = nh.subscribe(odom_encoder_topic, 1, &DataReader::odomEncoderCallback, this);
  imu_sub = nh.subscribe(imu_topic, 1, &DataReader::imuCallback, this);
  odom_filtered_sub = nh.subscribe(odom_filtered_topic, 1, &DataReader::odomFilteredCallback, this);
  steering_angle_sub = nh.subscribe(steering_angle_topic, 1, &DataReader::steeringAnglecallback, this);
  nav_cmd_sub = nh.subscribe(nav_cmd_topic, 1, &DataReader::navCmdVelCallback, this);
  data_reader_pub = nh.advertise<agv::DataReader>(data_reader_topic, 1);
  current_speed_pub = nh.advertise<std_msgs::Float64>("current_speed_topic", 1);
}

void DataReader::cmdVelocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
  data_reader_msg.cmd_vel.angular = cmd_vel_msg->angular;
  data_reader_msg.cmd_vel.linear = cmd_vel_msg->linear;
  data_reader_pub.publish(data_reader_msg);
}

void DataReader::odomEncoderCallback(const nav_msgs::Odometry::ConstPtr& odom_encoder_msg)
{
  data_reader_msg.odom_encoder_speed = odom_encoder_msg->twist.twist.linear.x;
  data_reader_msg.odom_encoder_yaw_rate = odom_encoder_msg->twist.twist.angular.z * RADIAN_TO_DEG_CONST;

  tf::Quaternion q_encoder(odom_encoder_msg->pose.pose.orientation.x, odom_encoder_msg->pose.pose.orientation.y,
                           odom_encoder_msg->pose.pose.orientation.z, odom_encoder_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m_encoder(q_encoder);
  double roll_encoder, pitch_encoder;
  m_encoder.getRPY(roll_encoder, pitch_encoder, data_reader_msg.odom_encoder_yaw);
  data_reader_msg.odom_encoder_yaw *= RADIAN_TO_DEG_CONST;

  data_reader_msg.encoder_pose_x = odom_encoder_msg->pose.pose.position.x;
  data_reader_msg.encoder_pose_y = odom_encoder_msg->pose.pose.position.y;
}

void DataReader::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  tf::Quaternion q_imu(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
  tf::Matrix3x3 m_imu(q_imu);
  m_imu.getRPY(data_reader_msg.imu_roll, data_reader_msg.imu_pitch, data_reader_msg.imu_yaw);
  data_reader_msg.imu_roll *= RADIAN_TO_DEG_CONST;
  data_reader_msg.imu_pitch *= RADIAN_TO_DEG_CONST;
  data_reader_msg.imu_yaw *= RADIAN_TO_DEG_CONST;
  data_reader_msg.imu_yaw_rate = imu_msg->angular_velocity.z * RADIAN_TO_DEG_CONST;
  data_reader_pub.publish(data_reader_msg);
}

void DataReader::odomFilteredCallback(const nav_msgs::Odometry::ConstPtr& odom_filtered_msg)
{
  data_reader_msg.odom_filtered_speed = odom_filtered_msg->twist.twist.linear.x;
  data_reader_msg.odom_filtered_yaw_rate = odom_filtered_msg->twist.twist.angular.z * RADIAN_TO_DEG_CONST;

  tf::Quaternion q_odom_filtered(odom_filtered_msg->pose.pose.orientation.x, odom_filtered_msg->pose.pose.orientation.y,
                                 odom_filtered_msg->pose.pose.orientation.z,
                                 odom_filtered_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m_odom_filtered(q_odom_filtered);
  double roll_filtered, pitch_filtered;
  m_odom_filtered.getRPY(roll_filtered, pitch_filtered, data_reader_msg.odom_filtered_yaw);
  data_reader_msg.odom_filtered_yaw *= RADIAN_TO_DEG_CONST;

  data_reader_msg.filtered_pose_x = odom_filtered_msg->pose.pose.position.x;
  data_reader_msg.filtered_pose_y = odom_filtered_msg->pose.pose.position.y;

  std_msgs::Float64 speed_msg;
  speed_msg.data = odom_filtered_msg->twist.twist.linear.x;
  current_speed_pub.publish(speed_msg);
}

void DataReader::steeringAnglecallback(const std_msgs::Float64::ConstPtr& stanley_steering_angle_msg)
{
  data_reader_msg.desired_steering_angle = stanley_steering_angle_msg->data;
}

void DataReader::navCmdVelCallback(const geometry_msgs::Twist::ConstPtr& nav_cmd_vel_msg)
{
  data_reader_msg.nav_cmd_vel.linear = nav_cmd_vel_msg->linear;
  data_reader_msg.nav_cmd_vel.angular = nav_cmd_vel_msg->angular;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_reader_node");
  DataReader data_reader_obj;
  ros::spin();
  return 0;
}
