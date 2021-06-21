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
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <stdlib.h>

#define RADIAN_TO_DEG_CONST 57.295779513
#define M_PI 3.14159265358979323846
#define FRONTLINK_TO_BASELINK 2.55

class StanleySteeringController
{
public:
  StanleySteeringController();

private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Publisher steering_angle_pub;
  ros::Timer stanley_steering_timer;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  std_msgs::Float64 steering_angle_msg;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_k_cloud;
  pcl::PointXYZ search_point;

  double stanley_max_steering_radius;
  double current_yaw;
  double current_speed;
  double nearest_X, nearest_Y, next_X, next_Y;
  double track_error_gain;
  int path_size;
  bool kdtree_ready = false;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
  void mainTimerCallback(const ros::TimerEvent& timer_event);

  double findClosestPathSegment();
  double calculateDistanceToPath();
  double calculateYawDelta(int& track_error_direction);
};

StanleySteeringController::StanleySteeringController() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  std::string odom_topic;
  std::string path_topic;
  std::string steering_angle_pub_topic;

  ROS_ASSERT(private_nh.getParam("track_error_gain", track_error_gain));
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
  ROS_ASSERT(private_nh.getParam("path_topic", path_topic));
  ROS_ASSERT(private_nh.getParam("steering_angle_pub_topic", steering_angle_pub_topic));
  ROS_ASSERT(private_nh.getParam("stanley_max_steering_radius", stanley_max_steering_radius));

  odom_sub = nh.subscribe(odom_topic, 1, &StanleySteeringController::odomCallback, this);
  path_sub = nh.subscribe(path_topic, 1, &StanleySteeringController::pathCallback, this);
  steering_angle_pub = nh.advertise<std_msgs::Float64>(steering_angle_pub_topic, 1);
  stanley_steering_timer = nh.createTimer(ros::Duration(0.1), &StanleySteeringController::mainTimerCallback, this);

  search_point.z = 0;
}

/*
 * Main calculation logic, the steps are:
 * 1. find the closest path segment (coordinates)
 * 2. calculate the track error
 * 3. calculate the track error direction
 * 4. calculate angle relative to vehicle's orientation, yaw delta is delta between actual and desired yaw
 * 5. arctan(track_error_gain * track_error / vehicle's velocity); (track_error_compensation, in radian)
 * 6. steering angle = yaw delta + track error compensation
 * 7. publish the desired steering angle
 */
void StanleySteeringController::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  if (!kdtree_ready)
    return;

  // step 1 & 2
  double track_error = findClosestPathSegment();

  // step 3 & 4
  int track_error_direction;
  double yaw_delta = calculateYawDelta(track_error_direction);
  if (yaw_delta > M_PI)
  {
    yaw_delta -= (2 * M_PI);
  }
  else if (yaw_delta < -M_PI)
  {
    yaw_delta += (2 * M_PI);
  }
  // step 5
  double u = std::max(1.0, current_speed);

  double track_error_compensation = track_error_direction * atan(track_error_gain * track_error / u);

  // step 6
  double desired_steering_angle = yaw_delta + track_error_compensation;

  ROS_WARN_STREAM_COND((std::abs(yaw_delta) > 1.309),
                       "Stanley Steering Controller: yaw_delta is rather big: " << yaw_delta);

  ROS_WARN_STREAM_COND(
      (std::abs(track_error_compensation) > 1.309),
      "Stanley Steering Controller: track_error_compensation is rather big: " << track_error_compensation);

  // For debugging
  // ROS_DEBUG_STREAM("Stanley Steering Controller: ");
  // ROS_DEBUG_STREAM("the u value is " << u);
  // ROS_DEBUG_STREAM("track_error: " << track_error);
  // ROS_DEBUG_STREAM("yaw_delta: " << (yaw_delta * RADIAN_TO_DEG_CONST));
  // ROS_DEBUG_STREAM("track_error_direction: " << track_error_direction);
  // ROS_DEBUG_STREAM("current speed: " << current_speed);
  // ROS_DEBUG_STREAM("track_error_compensation: " << (track_error_compensation * RADIAN_TO_DEG_CONST));
  // ROS_DEBUG_STREAM("desired_steering_angle: " << (desired_steering_angle * RADIAN_TO_DEG_CONST));

  // step 7
  if (desired_steering_angle > stanley_max_steering_radius)
  {
    desired_steering_angle = stanley_max_steering_radius;
  }
  else if (desired_steering_angle < -stanley_max_steering_radius)
  {
    desired_steering_angle = -stanley_max_steering_radius;
  }

  // ROS_DEBUG_STREAM("desired_steering_angle (FINAL): " << (desired_steering_angle * RADIAN_TO_DEG_CONST));

  steering_angle_msg.data = desired_steering_angle;
  steering_angle_pub.publish(steering_angle_msg);
}

/*
 * This function finds the closest path segment and updates nearest_X, nearest_Y, next_X and next_Y.
 * Note: if there is more than 1 closest point, nearestKSearch takes the smaller coordinates (based on observation, not
 * documentation).
 */
double StanleySteeringController::findClosestPathSegment()
{
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  double min_dist;

  if (kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    pointIdxNKNSearch[0] = std::min(path_size - 2, pointIdxNKNSearch[0]);
    nearest_X = nearest_k_cloud->points[pointIdxNKNSearch[0]].x;
    nearest_Y = nearest_k_cloud->points[pointIdxNKNSearch[0]].y;
    next_X = nearest_k_cloud->points[pointIdxNKNSearch[0] + 1].x;
    next_Y = nearest_k_cloud->points[pointIdxNKNSearch[0] + 1].y;
    // min_dist = sqrt(pointNKNSquaredDistance[0]);
    min_dist = calculateDistanceToPath();
    // ROS_DEBUG_STREAM("min dist in function " << min_dist);
  }
  else
  {
    // This should only happen if there is no path
    ROS_ERROR("Stanley Steering Controller: No nearest neighbour found!");
    return -1.0;
  }
  return min_dist;
}

// Do not use this function for now
double StanleySteeringController::calculateDistanceToPath()
{
  // Find min of perpendicular distance to the tangent of the path, refer to
  // https://www.quora.com/How-can-I-find-the-altitude-of-a-triangle-whose-3-vertices-points-are-given
  //   double triangle_area = abs(nearest_X * search_point.y + next_X * nearest_Y + search_point.x * next_Y -
  //                              (search_point.x * nearest_Y + nearest_X * next_Y + next_X * search_point.y)) /
  //                          2.0;
  double triangle_area = std::abs(nearest_X * (next_Y - search_point.y) + next_X * (search_point.y - nearest_Y) +
                                  search_point.x * (nearest_Y - next_Y)) /
                         2.0;
  // path length between nearest XY and next XY using Pythagoras Theorem
  double path_length = std::hypot((next_Y - nearest_Y), (next_X - nearest_X));
  // Area = 0.5 * b * h -> h = 2 * area / b
  double min_path = 2.0 * triangle_area / path_length;

  // For debugging
  // ROS_DEBUG_STREAM("current coordinates: ( " << search_point.x << ", " << search_point.y << ")");
  // ROS_DEBUG_STREAM("nearest path segment: (" << nearest_X << ", " << nearest_Y << ")");
  // ROS_DEBUG_STREAM("next path segment: (" << next_X << ", " << next_Y << ")");
  // ROS_DEBUG_STREAM("triangle area & path length: (" << triangle_area << ", " << path_length << ")");
  // ROS_DEBUG_STREAM("track_error: " << min_path);

  return min_path;
}

double StanleySteeringController::calculateYawDelta(int& track_error_direction)
{
  double del_Y = next_Y - nearest_Y;
  double del_X = next_X - nearest_X;
  double path_tangential_slope = del_Y / del_X;                                        // m = (y2-y1)/(x2-x1)
  double path_tangential_y_intercept = nearest_Y - path_tangential_slope * nearest_X;  // c = y - mx

  // Check if current robot pose is on the left or right side of path tangent line
  bool isLeft =
      ((search_point.x * path_tangential_slope + path_tangential_y_intercept) < search_point.y) ? true : false;

  // Current equation assumes robot's yaw and path tangent angle is within +- 45deg
  // TODO: handle cases when robot's current yaw is in opposite direction to path direction,
  //       currently the direction will be wrong
  double path_tangential_angle;
  if (del_X >= 0)  // in top quadrants
  {
    track_error_direction = isLeft ? -1 : 1;
    path_tangential_angle = atan(del_Y / del_X);
  }
  else  // in btm quadrants
  {
    track_error_direction = isLeft ? 1 : -1;
    if (del_Y >= 0)  // in btm left quadrant
    {
      // give 90 deg to 180 deg, since atan gives -90deg to 90deg only
      path_tangential_angle = M_PI + atan(del_Y / del_X);
    }
    else  // in btm right quadrant
    {
      // give -90 deg to -180 deg, since atan gives -90deg to 90deg only
      path_tangential_angle = -1 * M_PI + atan(del_Y / del_X);
    }
  }

  // ROS_DEBUG_STREAM("tangentslope:" << path_tangential_slope << "tangent y intercept: " << path_tangential_y_intercept
  //                                  << "isLeft: " << isLeft);

  return path_tangential_angle - current_yaw;  // swap because we invert the z axis direction, now z+ is up
}

void StanleySteeringController::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
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

  geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
  pose_before_transform.header.frame_id = odom_msg->header.frame_id;
  pose_before_transform.header.stamp = odom_msg->header.stamp;
  pose_before_transform.pose = odom_msg->pose.pose;
  tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

  tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                   pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_yaw);

  // Current XY of robot (map frame)
  search_point.x = pose_after_transform.pose.position.x + (FRONTLINK_TO_BASELINK * std::cos(current_yaw));
  search_point.y = pose_after_transform.pose.position.y + (FRONTLINK_TO_BASELINK * std::sin(current_yaw));
  // ROS_DEBUG("baselink (%f, %f)", pose_after_transform.pose.position.x , pose_after_transform.pose.position.y);
  // ROS_DEBUG("frontlink (%f, %f)", search_point.x , search_point.y);
}

void StanleySteeringController::pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ROS_INFO("Stanley Steering Controller: Received a new path.");

  pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  path_cloud->width = path_msg->poses.size();
  path_cloud->height = 1;
  path_cloud->points.resize(path_cloud->width * path_cloud->height);
  path_size = path_msg->poses.size();
  for (int i = 0; i < path_msg->poses.size(); i++)
  {
    geometry_msgs::Point point = path_msg->poses[i].pose.position;
    path_cloud->points[i].x = point.x;
    path_cloud->points[i].y = point.y;
    path_cloud->points[i].z = 0;
  }
  nearest_k_cloud = path_cloud;
  kdtree.setInputCloud(nearest_k_cloud);
  kdtree_ready = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stanley_steering_controller_node");
  StanleySteeringController stanley_steering_controller_obj;
  ros::spin();
  return 0;
}
