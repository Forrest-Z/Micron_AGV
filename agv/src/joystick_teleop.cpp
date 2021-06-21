/*
 * Joystick Teleop for mapping desired values
 * Output message is of type geometry_msgs::Twist
 * Mapping for output message:
 * linear.x => desired_velocity in m/s
 * linear.z => braking intensity, on a 0 - 1 range. 
 * angular.x =>  "em_brake_off"; EM brake switch closed/open (1/0)
 * angular.z => steering angle in radians
 * 
 * How health is handled by joystick teleop:
 * 1) We subscribe to /health_monitor topic.
 * 2) in the callback, we check message.health_ok. Then we know if our health is ok.
 * 3b) In case of good health:
 *    4) in joystickCallback: We take in user input from joystick and change the current_nav_mode accordingly. 
 *    5) in joystickCallback: Based on user input, we perform soft_brake, hard_brake, or Manual control, and send the signal to the motors. If user wants autonomous_mode, the output will be handled by  autonomousCmdVelCallback.
 *    6) in autonomousCmdVelCallback, if the user had pressed A for autonomous_mode, the autonomous command message will be passed directly into the output of joystick teleop, to the motors and actuators. 
 * 3a) In the case of bad health:
 *    4) in joystickCallback, we reject the user request if the user presses 'A' for autonomous mode. It goes to FailSafe instead.
 *    5) in autonomousCmdVelCallback, we realise we are bad health, and immediately send brake signal to motors. And we set current mode to soft_brake.
 */



#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <agv/HealthMonitor.h>

enum class NavMode
{
  HardBrake,
  FailSafe,
  Manual,
  Autonomous
};

class JoystickTeleop
{
public:
  JoystickTeleop();

private:
  ros::NodeHandle nh;

  ros::Subscriber joystick_sub;
  ros::Subscriber autonomous_cmd_sub;
  ros::Subscriber health_monitor_sub;
  ros::Publisher cmd_vel_pub;

  NavMode current_nav_mode;
  bool is_healthy_g = false; //'global' variable for if the vehicle is healthy
  float brake_threshold_float = 0.1;

  geometry_msgs::Twist cmd_vel_out;

  double MAX_ANGLE_STEERING = 0.698132;  // 40 deg in radians
  double MAX_SPEED = 3.9;                // m/s

  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void autonomousCmdVelCallback(const geometry_msgs::Twist::ConstPtr& autonomous_vel_msg);
  void healthMonitorCallback(const agv::HealthMonitor::ConstPtr& health_msg);
};

JoystickTeleop::JoystickTeleop()
{
  ros::NodeHandle private_nh("~");

  std::string joy_topic;
  std::string autonomous_cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string health_monitor_topic;

  ROS_ASSERT(private_nh.getParam("joy_topic", joy_topic));
  ROS_ASSERT(private_nh.getParam("autonomous_cmd_vel_in_topic", autonomous_cmd_vel_in_topic));
  ROS_ASSERT(private_nh.getParam("cmd_vel_out_topic", cmd_vel_out_topic));
  ROS_ASSERT(private_nh.getParam("health_monitor_topic", health_monitor_topic));

  joystick_sub = nh.subscribe(joy_topic, 1, &JoystickTeleop::joystickCallback, this);
  autonomous_cmd_sub = nh.subscribe(autonomous_cmd_vel_in_topic, 1, &JoystickTeleop::autonomousCmdVelCallback, this);
  health_monitor_sub = nh.subscribe(health_monitor_topic, 1, &JoystickTeleop::healthMonitorCallback, this);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_out_topic, 1);

  current_nav_mode = NavMode::HardBrake;
}

void JoystickTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // These controls are for the X-Box controller only
  int buttonA = joy_msg->buttons[0];
  int buttonB = joy_msg->buttons[1];
  int buttonX = joy_msg->buttons[2];
  // int buttonY = joy_msg->buttons[3];
  int buttonL1 = joy_msg->buttons[4];
  int buttonR1 = joy_msg->buttons[5];
  double forward_axes = joy_msg->axes[4];   // up & down on right-side axes
  double steering_axes = joy_msg->axes[0];  // left & right on left-side axes (xbox)
  // double steering_axes = joy_msg->axes[3];  // left & right on right-side axes (logitech) for testing
  // double triggerL = joy_msg->axes[2]; //left analog trigger
  // double triggerR = joy_msg->axes[5]; //right analog trigger

  if (buttonB == 1)
  {
    current_nav_mode = NavMode::HardBrake;
    ROS_INFO("Hard Brake Mode");
  }
  else if (buttonR1 == 1)
  {
    current_nav_mode = NavMode::FailSafe;
    ROS_INFO("Soft Brake Mode");
  }
  else if (buttonX == 1 || buttonL1 == 1)
  {
    current_nav_mode = NavMode::Manual;
    ROS_INFO("Manual Mode");
  }
  else if (buttonA == 1)
  {
    if (is_healthy_g == false)
    {
      current_nav_mode = NavMode::FailSafe;
      ROS_ERROR("Unhealthy vehicle! Check sensors! Going to Soft Brake Mode.");
    } 
    else if (current_nav_mode == NavMode::HardBrake)
    {
      current_nav_mode = NavMode::Autonomous;
      ROS_INFO("Autonomous Mode");
    }
    else if (current_nav_mode == NavMode::Autonomous)
    {
      ROS_INFO("Already in Autonomous Mode");
    }
    else
    {
      cmd_vel_out.linear.y = 1;
      ROS_INFO("Can only go to Autonomous Mode after Hard Brake!");
    }
  }
  else
  {
    // Empty Else
  }


  //set steering 
  cmd_vel_out.angular.z = steering_axes * MAX_ANGLE_STEERING;  

  if (current_nav_mode == NavMode::Manual)
  {
    //em-brake
    cmd_vel_out.angular.x = 1; //1 means do not engage the em-brake.
    cmd_vel_out.linear.y = 2;
    //set speed based on controls (joystick-upwards) , 
    //set brake based on controls (joystick-downward) 
    if (fabs(forward_axes) < 0.1) //dead zone.
    {
      cmd_vel_out.linear.x = 0; //set speed to 0
      cmd_vel_out.linear.z = 0; //set brake to 0
    }
    else
    {
      if (forward_axes > 0) //forward
      {
        cmd_vel_out.linear.x = (forward_axes - 0.1) * MAX_SPEED; //Speed: proportional.
        cmd_vel_out.linear.z = 0; //set brake to 0
      }
      else if (forward_axes < 0) //backward
      {
        cmd_vel_out.linear.z = (   (fabs(forward_axes) - 0.1) * (5.0/9.0)  )   +   0.5; //Var. brake: convert -0.1 -> -1 into 0.5 -> 1.0
        cmd_vel_out.linear.x = 0; //set speed to 0
      }
      
    }

    cmd_vel_pub.publish(cmd_vel_out);
  }
  else if (current_nav_mode == NavMode::HardBrake)
  {
    //set brake_intensity = 1
    cmd_vel_out.linear.y = 0;
    cmd_vel_out.linear.x = 0; //set speed to 0
    cmd_vel_out.angular.x = 0;
    cmd_vel_out.linear.z = 1; //braking intensity, 0-1
    cmd_vel_pub.publish(cmd_vel_out);
  }
  else if (current_nav_mode == NavMode::FailSafe)
  {
    //set brake_intensity = 0.7
    cmd_vel_out.linear.y = -1;
    cmd_vel_out.linear.x = 0; //set speed to 0
    cmd_vel_out.angular.x = 0;
    cmd_vel_out.linear.z = 0.7; //braking intensity, 0-1
    cmd_vel_pub.publish(cmd_vel_out);
  }
  else
  {
    // Empty Else
    // Let autonomousCmdVelCallback() function handle publishing autonomous mode messages
  }
}
  
void JoystickTeleop::autonomousCmdVelCallback(const geometry_msgs::Twist::ConstPtr& autonomous_vel_msg)
{
  if (current_nav_mode == NavMode::Autonomous)
  {
    //if we are unhealthy and in autonomous mode, go to soft brake mode.
    if (is_healthy_g == false)
    {
      current_nav_mode = NavMode::FailSafe;
      //set brake_intensity = 0.7
      cmd_vel_out.linear.x = 0; //set speed to 0
      cmd_vel_out.angular.x = 0;
      cmd_vel_out.linear.z = 0.7; //braking intensity, 0-1
      cmd_vel_pub.publish(cmd_vel_out);
      ROS_ERROR("Unhealthy vehicle! Check sensors! Going to Soft Brake Mode.");
    } 

    else //if we are healthy: autonomous variable brake.
    {
      if ((current_nav_mode == NavMode::Autonomous) and (is_healthy_g == true)) 
      {
        float brake_intensity = autonomous_vel_msg->linear.z;
        if (brake_intensity > brake_threshold_float)
        {
          //braking 
          ROS_INFO("Joystick Teleop: Autonomous Variable Brake");
          cmd_vel_out.linear.z = brake_intensity; 
          cmd_vel_out.linear.x = 0; //set velocity to 0        
        }
        else 
        {
          //not braking
          ROS_INFO("Joystick Teleop: Autonomous No Brake");
          cmd_vel_out.linear.z = 0.0; //no braking
          cmd_vel_out.linear.x = autonomous_vel_msg->linear.x; //set speed to desired speed
        }
        
        cmd_vel_out.angular.x = 1; //1 means do not engage the em-brake.
        cmd_vel_out.angular.z = autonomous_vel_msg->angular.z; //steering angle
        cmd_vel_pub.publish(cmd_vel_out);

        ROS_INFO("Joystick Teleop: Publishing Autonomous Commands");
      }
    }
  }

  else
  {
    //empty else
  }
}

//check the health of the system. determine if safe to engage to autonomous/ need to disengage autonomous.
void JoystickTeleop::healthMonitorCallback(const agv::HealthMonitor::ConstPtr& health_msg)
{
  //One mode of failure of the health monitor, is if the health monitor itself fails/ message from health monitor does not reach joystickteleop.

  bool health_ok = health_msg->health_ok;
  std::vector<std::string> topic_list = health_msg->topic_list;
  std::vector<float> health_points_list = health_msg->health_points_list;

  if (health_ok == false)
  {
    is_healthy_g = false;
  } 
  else if (health_ok == true)  
  {
    is_healthy_g = true;
  }

  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop_node");
  JoystickTeleop joystick_teleop_obj;
  ros::spin();
  return 0;
}