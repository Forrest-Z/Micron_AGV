/* mpc_controller.cpp

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

  Model Predictive Control ROS Node
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "common/polynomials.h"
#include "mpc/MPC.h"
// #include "mpc/helpers.h"

namespace agv
{
namespace mpc
{

class ModelPredictiveControl
{
public:
  // Constructor
  ModelPredictiveControl();
  // Destructor
  virtual ~ModelPredictiveControl(){};

private:
  // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Private Variables $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
  ros::NodeHandle nh;

  // MPC instance
  MPC mpc;

  // Hyperparameters
  double mpc_frequency_;

  double N_;
  double M_;
  double dt_;

  double max_path_size_;
  double min_path_size_;
  double order_of_poly_;

  double L_;
  double Lf_;

  double w_cte_;
  double w_epsi_;
  double w_ref_v_;

  double w_use_steer_;
  double w_use_accel_;

  double w_cont_steer_;
  double w_cont_accel_;

  double max_steering_angle_;
  double max_acceleration_;
  double max_deceleration_;

  double ref_v_ = 0.0;
  bool emergency_mode_ = false;

  // Vehicle's current state
  // Received from odom data
  double current_x = 0.0;
  double current_y = 0.0;
  double current_yaw = 0.0;
  double current_speed = 0.0;

  // Path waypoints
  std::vector<double> path_x;
  std::vector<double> path_y;

  // Output steering angle and accleration / target speed
  double steering_angle;
  double accleration;
  double target_speed;

  // subscriber and publishers
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Subscriber cmd_sub;

  ros::Publisher steering_angle_pub;
  ros::Publisher acceleration_pub;
  ros::Publisher control_output_pub;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // timer
  ros::Timer timer;

  // ###################################### Private Functions ######################################

  // Main Function in ROS running primary logics
  void mainTimerCallback(const ros::TimerEvent& timer_event);

  std::vector<double> getFrontAxlePose(double baselink_x, double baselink_y, double yaw,
                                       double baselink_to_front_axle_length);
  Eigen::VectorXd getPolynomialCoeffs(const std::vector<double>& car_pose, const std::vector<double>& path_x,
                                      const std::vector<double>& path_y, double order);
  double calcCrossTrackError(const Eigen::VectorXd& coeffs);
  double calcOrientationError(const Eigen::VectorXd& coeffs);
  void updateState(Eigen::VectorXd& state, double v, double cte, double epsi);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& path);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  void publishSteeringAngle(double angle);
  void publishControlOutputs(double angle, double accel);
};

ModelPredictiveControl::ModelPredictiveControl() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");

  // Topics
  std::string odom_topic_;
  std::string path_topic_;
  std::string cmd_topic_;
  std::string steering_angle_topic_;
  std::string control_output_topic_;

  // Topic Names
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("path_topic", path_topic_));
  ROS_ASSERT(private_nh.getParam("cmd_topic", cmd_topic_));
  ROS_ASSERT(private_nh.getParam("steering_angle_topic", steering_angle_topic_));
  ROS_ASSERT(private_nh.getParam("control_output_topic", control_output_topic_));

  // Hyperparameters
  ROS_ASSERT(private_nh.getParam("mpc_frequency", mpc_frequency_));
  ROS_ASSERT(private_nh.getParam("prediction_horizon", N_));
  ROS_ASSERT(private_nh.getParam("control_horizon", M_));
  ROS_ASSERT(private_nh.getParam("sample_time", dt_));

  // Path related params
  ROS_ASSERT(private_nh.getParam("max_path_size", max_path_size_));
  ROS_ASSERT(private_nh.getParam("min_path_size", min_path_size_));
  ROS_ASSERT(private_nh.getParam("order_of_poly", order_of_poly_));

  // Vehicle model params
  ROS_ASSERT(private_nh.getParam("baselink_to_front_axle_length", L_));
  ROS_ASSERT(private_nh.getParam("front_to_cog_length", Lf_));

  // Cost weights
  ROS_ASSERT(private_nh.getParam("cross_track_error_weight", w_cte_));
  ROS_ASSERT(private_nh.getParam("orientation_error_weight", w_epsi_));
  ROS_ASSERT(private_nh.getParam("speed_diff_error_weight", w_ref_v_));

  ROS_ASSERT(private_nh.getParam("steering_usage_weight", w_use_steer_));
  ROS_ASSERT(private_nh.getParam("acceleration_usage_weight", w_use_accel_));

  ROS_ASSERT(private_nh.getParam("steering_smoothness_weight", w_cont_steer_));
  ROS_ASSERT(private_nh.getParam("acceleration_smoothness_weight", w_cont_accel_));

  // Hard constraints
  ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle_));
  ROS_ASSERT(private_nh.getParam("max_acceleration", max_acceleration_));
  ROS_ASSERT(private_nh.getParam("max_deceleration", max_deceleration_));

  // Pass the params to the planner constructor
  std::vector<double> params = { N_, M_, dt_, Lf_ };

  std::vector<double> weights = { w_cte_, w_epsi_, w_ref_v_, w_use_steer_, w_use_accel_, w_cont_steer_, w_cont_accel_ };

  std::vector<double> constraints = { max_steering_angle_, max_acceleration_, max_deceleration_ };

  // MPC instance
  mpc = MPC(params, weights, constraints);

  // Subscribe & Advertise
  odom_sub = nh.subscribe(odom_topic_, 1, &ModelPredictiveControl::odomCallback, this);
  path_sub = nh.subscribe(path_topic_, 1, &ModelPredictiveControl::pathCallback, this);
  cmd_sub = nh.subscribe(cmd_topic_, 1, &ModelPredictiveControl::cmdCallback, this);
  steering_angle_pub = nh.advertise<std_msgs::Float64>(steering_angle_topic_, 1);
  control_output_pub = nh.advertise<geometry_msgs::Twist>(control_output_topic_, 1);

  // timer
  timer = nh.createTimer(ros::Duration(1.0 / mpc_frequency_), &ModelPredictiveControl::mainTimerCallback, this);
}

void ModelPredictiveControl::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  ROS_DEBUG("MPC: START!");

  // STEP 0: Safety check
  if (path_x.size() < min_path_size_)
  {
    ROS_ERROR("MPC: Path Size Is Less Than Required, QUIT");
    return;
  }

  // STEP 1: Update the coefficients
  std::vector<double> front_axle_pose = getFrontAxlePose(current_x, current_y, current_yaw, L_);
  Eigen::VectorXd coeffs = getPolynomialCoeffs(front_axle_pose, path_x, path_y, order_of_poly_);

  // STEP 2: Update the vehicle state
  double cte = calcCrossTrackError(coeffs);
  double epsi = calcOrientationError(coeffs);

  std::cout << "Speed: " << current_speed << " CTE: " << cte << " EPSI: " << epsi << std::endl;

  Eigen::VectorXd state(6);
  updateState(state, current_speed, cte, epsi);

  // STEP 3: Solve for the best control acutations
  std::vector<double> vars = mpc.Solve(state, coeffs, ref_v_);
  ROS_DEBUG("MPC: SOLVED!");

  // steering_angle = vars[6];
  accleration = vars[7];
  target_speed = vars[7] * dt_ + current_speed;

  // Debug
  std::cout << "Output States: " << std::endl;
  std::cout << "x = " << vars[0] << std::endl;
  std::cout << "y = " << vars[1] << std::endl;
  std::cout << "psi = " << vars[2] << std::endl;
  std::cout << "v = " << vars[3] << std::endl;
  std::cout << "cte = " << vars[4] << std::endl;
  std::cout << "epsi = " << vars[5] << std::endl;
  std::cout << "delta = " << vars[6] << std::endl;
  std::cout << "a = " << vars[7] << std::endl;
  std::cout << std::endl;

  publishSteeringAngle(steering_angle);
  publishControlOutputs(steering_angle, accleration);

  ROS_DEBUG("MPC: SUCCESS!");
}

std::vector<double> ModelPredictiveControl::getFrontAxlePose(double baselink_x, double baselink_y, double yaw,
                                                             double baselink_to_front_axle_length)
{
  double front_axle_x = baselink_x + cos(yaw) * baselink_to_front_axle_length;
  double front_axle_y = baselink_y + sin(yaw) * baselink_to_front_axle_length;

  return { front_axle_x, front_axle_y, yaw };
}

Eigen::VectorXd ModelPredictiveControl::getPolynomialCoeffs(const std::vector<double>& car_pose,
                                                            const std::vector<double>& path_x,
                                                            const std::vector<double>& path_y, double order)
{
  Eigen::VectorXd ptsx(path_x.size());
  Eigen::VectorXd ptsy(path_y.size());

  const double sinyaw = sin(car_pose.at(2));
  const double cosyaw = cos(car_pose.at(2));

  // transform path into car's local coordinate
  for (int i = 0; i < path_x.size(); i++)
  {
    const double dx = path_x.at(i) - car_pose.at(0);
    const double dy = path_y.at(i) - car_pose.at(1);
    ptsx(i) = dx * cosyaw + dy * sinyaw;
    ptsy(i) = -dx * sinyaw + dy * cosyaw;

    // std::cout << "x: " << ptsx(i) << " y: " << ptsy(i) << std::endl;
  }

  ROS_DEBUG("MPC: ptsx and ptsy created");

  auto coeffs = agv::common::polyfit(ptsx, ptsy, order);

  return coeffs;
}

double ModelPredictiveControl::calcCrossTrackError(const Eigen::VectorXd& coeffs)
{
  return -agv::common::polyeval(coeffs, 0);
}

double ModelPredictiveControl::calcOrientationError(const Eigen::VectorXd& coeffs)
{
  return -atan(coeffs[1]);
}

void ModelPredictiveControl::updateState(Eigen::VectorXd& state, double v, double cte, double epsi)
{
  state << 0, 0, 0, v, cte, epsi;
}

void ModelPredictiveControl::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
  path_x.clear();
  path_y.clear();

  if (path->poses.size() < 2)
  {
    ROS_ERROR("MPC: Path Size Is Less Than 2");
  }
  else
  {
    int size = max_path_size_ < path->poses.size() ? max_path_size_ : path->poses.size();
    Eigen::VectorXd ptsx(size);
    Eigen::VectorXd ptsy(size);
    for (int i = 0; i < size; i++)
    {
      path_x.push_back(path->poses[i].pose.position.x);
      path_y.push_back(path->poses[i].pose.position.y);
    }
  }

  ROS_DEBUG("MPC: Path Updated");
}

void ModelPredictiveControl::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
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

  tf::Quaternion quaternion(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                            pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
  tf::Matrix3x3 rpy_matrix(quaternion);
  double roll, pitch;
  rpy_matrix.getRPY(roll, pitch, current_yaw);

  // Current XY of robot (map frame)
  current_x = pose_after_transform.pose.position.x;
  current_y = pose_after_transform.pose.position.y;

  ROS_DEBUG("MPC: Odom Updated");
}

void ModelPredictiveControl::cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  ref_v_ = cmd_msg->linear.x;

  steering_angle = cmd_msg->angular.z;

  if (cmd_msg->linear.z == 1.0)
  {
    emergency_mode_ = true;
  }
  else
  {
    emergency_mode_ = false;
  }
}

void ModelPredictiveControl::publishSteeringAngle(double angle)
{
  std_msgs::Float64 steering_angle;
  steering_angle.data = angle;
  steering_angle_pub.publish(steering_angle);

  ROS_DEBUG("MPC: Steering Angle Published");
}

void ModelPredictiveControl::publishControlOutputs(double angle, double accel)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = accel * dt_ + current_speed; // desired speed
  if (emergency_mode_)
  {
    twist_msg.linear.z = 1;
  }
  else if (accel < -1.0)
  {
    twist_msg.linear.z = fabs(accel / max_deceleration_); // brake intensity
  }
  else
  {
    twist_msg.linear.z = 0;
  }
  twist_msg.angular.x = 1; // EM brake
  twist_msg.angular.z = angle; // steering angle

  
  control_output_pub.publish(twist_msg);

  ROS_DEBUG("MPC: Control Output Published");
}

} // namespace mpc
} // namespace agv

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_controller_node");
  // Construct a controller object
  agv::mpc::ModelPredictiveControl model_predictive_control;
  ros::spin();  // spin the ros node.
  return 0;
}