//This node subscribes to various topics and computes the health based on how many messages they send (compared to how many they *should* be sending). 
//Things that can be improved on:
//  1) determine the health of the algorithmns, such as the AMCL confidence in localisation, or path planner hz.
//  2) determine that Encoder and IMU agree with each other.
//  3) Make less spaghetti: I tried to make a class, with 1 message-type-agnostic subscriber in a class, using shapeshifter package. But it did not work well, because the topicCallback was not able to access variables that is owned by its own class!  
//      IE: HealthMonitor has a instance of xClass, where xClass is subscribed to IMU ; xClass has IMU topic callback xCallback(). If xCallback() try to access a variable that is owned by xClass, I keep getting unexpected value, such as a boolean that is always 1 when it is actually 0. This does not happen if the callback is a Timer-Callback (It only happen in topic-Callback)

#include <stdlib.h>

//topic data types 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
// #include <agv/EncoderCount.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <agv/HealthMonitor.h>
#include <agv/ActuatorHealth.h>
#include <std_msgs/Bool.h>

#define NO_ERROR "-"
#define MAX_TOPICS 7
#define LOW_MSG_FREQ "low message frequency"

class HealthMonitor
{
public:
  HealthMonitor();

private:
  //sensor parameters:
  int imu_msg_count = 0; //# of messages sent topic within the current frame. 
  int imu_expected_hz = 40;//Expected frequency at which each topic is supposed to publish at.
  float imu_health ;
  bool imu_is_initialised_flag = false;//This flag is false if we have not received even one message yet.
  //Rosparams: topic names.
  std::string imu_topic;
  //Subscriber nodes
  ros::Subscriber imu_sub;

  int velodyne_points_msg_count = 0; 
  int velodyne_points_expected_hz;
  float velodyne_points_health;
  bool velodyne_points_is_initialised_flag = false;
  std::string velodyne_points_topic;
  ros::Subscriber velodyne_points_sub;

  int front_scan_msg_count = 0; 
  int front_scan_expected_hz;
  float front_scan_health;
  bool front_scan_is_initialised_flag = false;
  std::string front_scan_topic;
  ros::Subscriber front_scan_sub;

  int left_scan_msg_count = 0;
  int left_scan_expected_hz;
  float left_scan_health;
  bool left_scan_is_initialised_flag = false;
  std::string left_scan_topic;
  ros::Subscriber left_scan_sub;

  int right_scan_msg_count = 0;
  int right_scan_expected_hz;
  float right_scan_health;
  bool right_scan_is_initialised_flag = false;
  std::string right_scan_topic;
  ros::Subscriber right_scan_sub;

  int brake_msg_count = 0;
  int brake_expected_hz;
  float brake_health;
  bool brake_is_initialised_flag = false;
  std::string brake_error = "";
  std::string brake_topic;
  ros::Subscriber brake_sub;

  int steering_msg_count = 0;
  int steering_expected_hz;
  float steering_health;
  bool steering_is_initialised_flag = false;
  std::string steering_error = "";
  std::string steering_topic;
  ros::Subscriber steering_sub;


  //The duration of each sample frame in seconds.
  double sample_period;
  
  ros::NodeHandle nh;

  //Publisher nodes
  ros::Publisher health_ok_pub;
  std::string health_monitor_topic;

  //timer
  ros::Timer timer;
  ros::Time previous_time;

  //intermediate values of computation: 
  double frame_time;

  float health_threshold;

  // Declare Subscriber callbacks
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void velodynePointsCallback(const sensor_msgs::PointCloud2::ConstPtr& velodyne_points_msg);
  void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& front_scan_msg);
  void leftScanCallback(const sensor_msgs::LaserScan::ConstPtr& left_scan_msg);
  void rightScanCallback(const sensor_msgs::LaserScan::ConstPtr& right_scan_msg);

  void brakeCallback(const agv::ActuatorHealth::ConstPtr& brake_health_msg);
  void steeringCallback(const agv::ActuatorHealth::ConstPtr& steering_health_msg);

  //void timerCallback(const ros::TimerEvent::ConstPtr& timer_event);
  void mainTimerCallback(const ros::TimerEvent& timer_event);

  // Declare other functions

  float computeSensorHealth(int actual_msg_count, int expected_hz, double frame_time);
  void createHealthMonitorMessage();
  // void killAndRestartNode(std::string node_name);
  void printStatus();

  std::string node_name ;
};

// constructor
HealthMonitor::HealthMonitor()
{
  ros::NodeHandle private_nh("~");

  //get key parameter: sampling period
  ROS_ASSERT(private_nh.getParam("sample_period", sample_period));

  //subscribe to the sensor topics
  ROS_ASSERT(private_nh.getParam("imu_topic", imu_topic));
  ROS_ASSERT(private_nh.getParam("imu_expected_hz", imu_expected_hz));
  ROS_ASSERT(private_nh.getParam("velodyne_points_topic", velodyne_points_topic));
  ROS_ASSERT(private_nh.getParam("velodyne_points_expected_hz", velodyne_points_expected_hz));
  ROS_ASSERT(private_nh.getParam("front_scan_topic", front_scan_topic));
  ROS_ASSERT(private_nh.getParam("front_scan_expected_hz", front_scan_expected_hz));
  ROS_ASSERT(private_nh.getParam("left_scan_topic", left_scan_topic));
  ROS_ASSERT(private_nh.getParam("left_scan_expected_hz", left_scan_expected_hz));
  ROS_ASSERT(private_nh.getParam("right_scan_topic", right_scan_topic));
  ROS_ASSERT(private_nh.getParam("right_scan_expected_hz", right_scan_expected_hz));

  ROS_ASSERT(private_nh.getParam("brake_topic", brake_topic));
  ROS_ASSERT(private_nh.getParam("brake_expected_hz", brake_expected_hz));
  ROS_ASSERT(private_nh.getParam("steering_topic", steering_topic));
  ROS_ASSERT(private_nh.getParam("steering_expected_hz", steering_expected_hz));

  ROS_ASSERT(private_nh.getParam("health_threshold", health_threshold));

  imu_sub = nh.subscribe(imu_topic, 1, &HealthMonitor::imuCallback, this);
  velodyne_points_sub = nh.subscribe(velodyne_points_topic, 1, &HealthMonitor::velodynePointsCallback, this);
  front_scan_sub = nh.subscribe(front_scan_topic, 1, &HealthMonitor::frontScanCallback, this);
  left_scan_sub = nh.subscribe(left_scan_topic, 1, &HealthMonitor::leftScanCallback, this);
  right_scan_sub = nh.subscribe(right_scan_topic, 1, &HealthMonitor::rightScanCallback, this);

  brake_sub = nh.subscribe(brake_topic, 1, &HealthMonitor::brakeCallback, this);
  steering_sub = nh.subscribe(steering_topic, 1, &HealthMonitor::steeringCallback, this);
 
  //advertise topic (make publisher)
  ROS_ASSERT(private_nh.getParam("health_monitor_topic", health_monitor_topic));
  health_ok_pub = nh.advertise<agv::HealthMonitor>(health_monitor_topic, 1);

  //start timer
  timer = nh.createTimer(ros::Duration(sample_period),&HealthMonitor::mainTimerCallback, this);

  //Others:
  
  //set time of the last frame
  previous_time = ros::Time::now();

  //get name of the node (for future printing)
  // node_name = ros::this_node::getName();
  // char * node_name_c_str = new char[node_name.length() + 1]; 
  // strcpy(node_name_c_str, node_name.c_str());
}

void HealthMonitor::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  imu_msg_count += 1;
  if (imu_is_initialised_flag == false) 
  {
    imu_is_initialised_flag = true;
  }
}

void HealthMonitor::velodynePointsCallback(const sensor_msgs::PointCloud2::ConstPtr& velodyne_points_msg)
{
  velodyne_points_msg_count += 1;
  if (velodyne_points_is_initialised_flag == false) 
  {
    velodyne_points_is_initialised_flag = true;
  }
}

void HealthMonitor::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& front_scan_msg)
{
  front_scan_msg_count += 1;
  if (front_scan_is_initialised_flag == false) 
  {
    front_scan_is_initialised_flag = true;
  }
}

void HealthMonitor::leftScanCallback(const sensor_msgs::LaserScan::ConstPtr& left_scan_msg)
{
  left_scan_msg_count += 1;
  if (left_scan_is_initialised_flag == false) 
  {
    left_scan_is_initialised_flag = true;
  }
}

void HealthMonitor::rightScanCallback(const sensor_msgs::LaserScan::ConstPtr& right_scan_msg)
{
  right_scan_msg_count += 1;
  if (right_scan_is_initialised_flag == false) 
  {
    right_scan_is_initialised_flag = true;
  }
}

void HealthMonitor::brakeCallback(const agv::ActuatorHealth::ConstPtr& brake_health_msg)
{
  if (brake_health_msg->health_ok == false)
  {
    brake_error = brake_health_msg->error;
  } 
  else 
  {
    brake_error = NO_ERROR;
  }
  brake_msg_count += 1;
  if (brake_is_initialised_flag  == false) 
  {
    brake_is_initialised_flag = true;
  }

}

void HealthMonitor::steeringCallback(const agv::ActuatorHealth::ConstPtr& steering_health_msg)
{
  if (steering_health_msg->health_ok == false)
  {
    steering_error = steering_health_msg->error;
  }
  else
  {
    steering_error = NO_ERROR;
  }

  steering_msg_count += 1;
  if (steering_is_initialised_flag == false) 
  {
    steering_is_initialised_flag = true;
  }    
}


/* This function calculates the health for the present sample frame, and publishes it in a health_monitor msg.  .
 * This function is called at the end of each sample frame.
 * 
 */
void HealthMonitor::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  //Calculate the time taken for the current frame.
  ros::Time current_time = ros::Time::now();
  ros::Duration deltaTime = current_time - previous_time;
  frame_time = double(deltaTime.nsec / 1000000000.0) + deltaTime.sec; //.sec gives you the time, rounded-down to seconds. But .nsec gives you nsecs (with the 'seconds' truncated away.)
  previous_time = current_time;

  //generate msg
  agv::HealthMonitor msg;
  msg.error_list.assign(MAX_TOPICS, NO_ERROR);

  //calculate the health of the topic.
  imu_health = computeSensorHealth(imu_msg_count , imu_expected_hz, frame_time);
  if (imu_health < health_threshold)
  {
    msg.index_fail.push_back(0);
    msg.error_string += imu_topic + " ";
  }
    
  velodyne_points_health = computeSensorHealth(velodyne_points_msg_count , velodyne_points_expected_hz, frame_time);
  if (velodyne_points_health < health_threshold)
  {
    msg.index_fail.push_back(1);
    msg.error_string += velodyne_points_topic + " ";
  }

  front_scan_health = computeSensorHealth(front_scan_msg_count , front_scan_expected_hz, frame_time);
  if (front_scan_health < health_threshold)
  {
    msg.index_fail.push_back(2);
    msg.error_string += front_scan_topic + " ";
  }

  left_scan_health = computeSensorHealth(left_scan_msg_count , left_scan_expected_hz, frame_time);
  if (left_scan_health < health_threshold)
  {
    msg.index_fail.push_back(3);
    msg.error_string += left_scan_topic + " ";
  }

  right_scan_health = computeSensorHealth(right_scan_msg_count , right_scan_expected_hz, frame_time);
  if (right_scan_health < health_threshold)
  {
    msg.index_fail.push_back(4);
    msg.error_string += right_scan_topic + " ";
  }


  brake_health = computeSensorHealth(brake_msg_count , brake_expected_hz, frame_time);
  if (brake_error != NO_ERROR)
  {
    msg.index_fail.push_back(5);
    msg.error_list.at(5) = brake_error;

    msg.error_string += brake_topic + " ";
  }
  else if (brake_health < health_threshold) 
  {
    msg.index_fail.push_back(5);
    msg.error_list.at(5) = LOW_MSG_FREQ;
  }

  steering_health = computeSensorHealth(steering_msg_count , steering_expected_hz, frame_time);
  if (steering_error != NO_ERROR)
  {
    msg.index_fail.push_back(6);
    msg.error_list.at(6) = steering_error;

    msg.error_string += steering_topic + " ";
  } 
  else if (steering_health < health_threshold)
  {
    msg.index_fail.push_back(6);
    msg.error_list.at(6) = LOW_MSG_FREQ;
  }

  msg.topic_list.push_back(imu_topic);
  msg.health_points_list.push_back(imu_health);
  msg.topic_list.push_back(velodyne_points_topic);
  msg.health_points_list.push_back(velodyne_points_health);
  msg.topic_list.push_back(front_scan_topic);
  msg.health_points_list.push_back(front_scan_health);
  msg.topic_list.push_back(left_scan_topic);
  msg.health_points_list.push_back(left_scan_health);
  msg.topic_list.push_back(right_scan_topic);
  msg.health_points_list.push_back(right_scan_health);

  msg.topic_list.push_back(brake_topic);
  msg.health_points_list.push_back(brake_health);
  msg.topic_list.push_back(steering_topic);
  msg.health_points_list.push_back(steering_health);

  if (msg.index_fail.empty() == false) //there is at least 1 error
  {
    //publish unhealthy message
    msg.health_ok = false;
    health_ok_pub.publish(msg);
  }
  else
  {
    msg.health_ok = true;
    health_ok_pub.publish(msg);
  }

  //printStatus();
  
  //reset
  imu_msg_count = 0;
  velodyne_points_msg_count = 0;
  front_scan_msg_count = 0;
  left_scan_msg_count = 0;
  right_scan_msg_count = 0;
  brake_msg_count = 0;
  steering_msg_count = 0;
}


// This function just handles the printing to terminal.
void HealthMonitor::printStatus()
{
  std::string debug_output = "";
  debug_output += "time taken for the previous sample frame:" + std::to_string(frame_time) + "\n";

  //imu
  if (! imu_is_initialised_flag)
  {
    debug_output += imu_topic + ": Not initialised yet. \n" ;
  } 
  else
  {
    debug_output += imu_topic + ": Health : " + std::to_string(imu_health) + "\n";
  }

  if (! velodyne_points_is_initialised_flag)
  {
    debug_output += velodyne_points_topic + ": Not initialised yet. \n" ;
  } 
  else
  {
    debug_output += velodyne_points_topic + ": Health : " + std::to_string(velodyne_points_health) + "\n";
  }
  

  if (! front_scan_is_initialised_flag)
  {
    debug_output += front_scan_topic + ": Not initialised yet. \n" ;
  } 
  else
  {
    debug_output += front_scan_topic + ": Health : " + std::to_string(front_scan_health) + "\n";
  }

  if (! left_scan_is_initialised_flag)
  {
    debug_output += left_scan_topic + ": Not initialised yet. \n" ;
  } 
  else
  {
    debug_output += left_scan_topic + ": Health : " + std::to_string(left_scan_health) + "\n";
  }

  if (! right_scan_is_initialised_flag)
  {
    debug_output += right_scan_topic + ": Not initialised yet. \n" ;
  } 
  else
  {
    debug_output += right_scan_topic + ": Health : " + std::to_string(right_scan_health) + "\n";
  }
  
  ROS_INFO_STREAM(debug_output);
}

// Get the health of the sensor based on sensor health formula. Higher health value means more healthy.
// frame_time is in seconds.
float HealthMonitor::computeSensorHealth(int actual_msg_count, int expected_hz, double frame_time)
{
  float sensor_health = (float) actual_msg_count / ( (float) expected_hz * frame_time);
  return(sensor_health);
}

// /*This function will kill and then restart a node.
//  * @param node_name string The name of the node to restart.
//  */
// void HealthMonitor::killAndRestartNode(std::string node_name)
// {
//   //char * node_name_c_str = new char[node_name.length() + 1]; 
//   //strcpy(node_name_c_str, node_name.c_str());

//   //char const* kill_c_str = strcat("rosnode kill ", node_name_c_str);
//   system("rosnode kill microstrain_3dm_gx5_25_node");      //kill
//   system("roslaunch agv imu.launch");  //start
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "health_monitor_node");
  HealthMonitor health_monitor_obj;
  ros::spin();
  return 0;
}