/*
 * This program receives the IMU data and publishes a transformed version of it
 * It is assumed that the input IMU data is in NWU frame
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

enum class TargetTransFormAxis
{
  ENU,
  NED
};

class ImuFrameTransformer
{
public:
  ImuFrameTransformer();

private:
  ros::NodeHandle nh;
  ros::Subscriber imu_sub;
  ros::Publisher imu_pub;

  TargetTransFormAxis target_transform_axis;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

  void doImuTransformToNED(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void doImuTransformToENU(const sensor_msgs::Imu::ConstPtr& imu_msg);
};

ImuFrameTransformer::ImuFrameTransformer()
{
  ros::NodeHandle private_nh("~");

  std::string target_transform_axis_str;
  std::string imu_data_in_topic;

  ROS_ASSERT(private_nh.getParam("target_transform_axis", target_transform_axis_str));
  ROS_ASSERT(private_nh.getParam("imu_data_in_topic", imu_data_in_topic));

  if (target_transform_axis_str.compare("ENU") == 0)
  {
    target_transform_axis = TargetTransFormAxis::ENU;
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/ENU", 1);
  }
  else if (target_transform_axis_str.compare("NED") == 0)
  {
    target_transform_axis = TargetTransFormAxis::NED;
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/NED", 1);
  }
  else
  {
    ROS_ASSERT_MSG(false, "Imu Frame Transformer: Unknown target axis");
  }

  imu_sub = nh.subscribe(imu_data_in_topic, 1, &ImuFrameTransformer::imuCallback, this);
}

void ImuFrameTransformer::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  switch (target_transform_axis)
  {
    case TargetTransFormAxis::ENU:
      doImuTransformToENU(imu_msg);
      break;
    case TargetTransFormAxis::NED:
      doImuTransformToNED(imu_msg);
      break;
    default:
      ROS_ERROR("Imu Frame Transformer: No such target transform axis!");
  }
}

void ImuFrameTransformer::doImuTransformToNED(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  geometry_msgs::Quaternion orientation_before_transform = imu_msg->orientation;
  geometry_msgs::Vector3 angular_velocity_before_transform = imu_msg->angular_velocity;
  geometry_msgs::Vector3 linear_acceleration_before_transform = imu_msg->linear_acceleration;

  geometry_msgs::TransformStamped NWU_to_NED_transform_stamped;
  NWU_to_NED_transform_stamped.header.stamp = imu_msg->header.stamp;
  NWU_to_NED_transform_stamped.header.frame_id = imu_msg->header.frame_id;
  NWU_to_NED_transform_stamped.child_frame_id = "NED_imu_link";
  NWU_to_NED_transform_stamped.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);

  geometry_msgs::Quaternion orientation_after_transform;
  tf2::doTransform(orientation_before_transform, orientation_after_transform, NWU_to_NED_transform_stamped);

  geometry_msgs::Vector3 angular_velocity_after_transform;
  tf2::doTransform(angular_velocity_before_transform, angular_velocity_after_transform, NWU_to_NED_transform_stamped);

  geometry_msgs::Vector3 linear_acceleration_after_transform;
  tf2::doTransform(linear_acceleration_before_transform, linear_acceleration_after_transform,
                   NWU_to_NED_transform_stamped);

  sensor_msgs::Imu transformed_imu_msg_NED;
  transformed_imu_msg_NED.header.stamp = NWU_to_NED_transform_stamped.header.stamp;
  transformed_imu_msg_NED.header.frame_id = "NED_imu_link";
  transformed_imu_msg_NED.orientation = orientation_after_transform;
  transformed_imu_msg_NED.angular_velocity = angular_velocity_after_transform;
  transformed_imu_msg_NED.linear_acceleration = linear_acceleration_after_transform;
  imu_pub.publish(transformed_imu_msg_NED);
}

void ImuFrameTransformer::doImuTransformToENU(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  geometry_msgs::Quaternion orientation_before_transform = imu_msg->orientation;
  geometry_msgs::Vector3 angular_velocity_before_transform = imu_msg->angular_velocity;
  geometry_msgs::Vector3 linear_acceleration_before_transform = imu_msg->linear_acceleration;

  geometry_msgs::TransformStamped NWU_to_ENU_transform_stamped;
  NWU_to_ENU_transform_stamped.header.stamp = imu_msg->header.stamp;
  NWU_to_ENU_transform_stamped.header.frame_id = imu_msg->header.frame_id;
  NWU_to_ENU_transform_stamped.child_frame_id = "ENU_imu_link";
  NWU_to_ENU_transform_stamped.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);

  geometry_msgs::Quaternion orientation_after_transform;
  tf2::doTransform(orientation_before_transform, orientation_after_transform, NWU_to_ENU_transform_stamped);

  geometry_msgs::Vector3 angular_velocity_after_transform;
  tf2::doTransform(angular_velocity_before_transform, angular_velocity_after_transform, NWU_to_ENU_transform_stamped);

  geometry_msgs::Vector3 linear_acceleration_after_transform;
  tf2::doTransform(linear_acceleration_before_transform, linear_acceleration_after_transform,
                   NWU_to_ENU_transform_stamped);

  sensor_msgs::Imu transformed_imu_msg_ENU;
  transformed_imu_msg_ENU.header.stamp = NWU_to_ENU_transform_stamped.header.stamp;
  transformed_imu_msg_ENU.header.frame_id = "ENU_imu_link";
  transformed_imu_msg_ENU.orientation = orientation_after_transform;
  transformed_imu_msg_ENU.angular_velocity = angular_velocity_after_transform;
  transformed_imu_msg_ENU.linear_acceleration = linear_acceleration_after_transform;
  imu_pub.publish(transformed_imu_msg_ENU);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "class_name_node");
  ImuFrameTransformer class_name_obj;
  ros::spin();
  return 0;
}
