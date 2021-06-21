/*
 * For Dynamixel H54P-200-S500-R, Dynamixel Protocol 2.0
 * http://emanual.robotis.com/docs/en/dxl/pro_plus/h54p-200-s500-r/
 * Refer to documentation dynamixel_steering_controller.adoc for more details
 * How the restart works: 1) We continually detect error by running checkForPacketHandlerErrors(). if there is error, is_communication_error is set to TRUE.
 *                        2) In timercallback, we check is_communication_error and is_communication_error_prev, and if certain state is met, we attempt to re-setup the dynamixel.
 *                        3) We set is_communication_error_prev = is_communication_error and continue to the next loop.
 *                        4) The logic is not great and I'm pretty sure I'm missing out some edge cases.
 * 
 * Issues:  1) In testing, restarting the motor sometimes does not work. Also, the logic for restarting the motor is quite bad.
 *          2) There is alot of repetition between brake and steering motor nodes. Consider putting the repeated parts into a class/library/header file.
 *          3) If you rostopic echo /health_monitor and move the brake, you can see that the health_points fluctuates. I do not know why this happens, but my best guess is that the calls to dynamixelSDK might be blocking, thus delaying execution of the publisher.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdint>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <agv/ActuatorHealth.h>

#define PROTOCOL_VERSION 2.0

#define ADDR_PRO_PLUS_OPERATING_MODE 11
#define ADDR_PRO_PLUS_TORQUE_ENABLE 512
#define ADDR_PRO_PLUS_GOAL_POSITION 564
#define ADDR_PRO_PLUS_MOVING_SPEED 552
#define ADDR_PRO_PLUS_PRESENT_POSITION 580

#define EXTENDED_POSITION_CONTROL_MODE 4

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1
#define COMM_SUCCESS 0

#define STEERING_ANGLE_TO_GOAL_POSITION_CONST 3195340.53  // TODO
#define STEERING_ANGLE_LIMIT 0.610865

class DynamixelProPlusSteeringController
{
public:
  DynamixelProPlusSteeringController();

private:
  ros::NodeHandle nh;

  //subscriber, publisher, timer, related parameters
  ros::Subscriber steering_controller_msg_sub;
  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);
  ros::Publisher present_steering_angle_pub;
  ros::Publisher steering_health_pub;
  ros::Timer timer;
  void timerCallback(const ros::TimerEvent& timer_event);
  float publish_steering_period;

  //dynamixel stuff
  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  double WHEEL_CENTER_OFFSET = 0;

  int dxl_id;
  std::string device_port;
  int baud_rate;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  bool setupDyanmixelSteeringMotor();
  bool checkForPacketHandlerErrors();
  bool enableSteeringDynamixelTorque();
  bool setSteeringDynamixelSpeed();
  bool setSteeringMotorToStartPosition();
  bool setProPlusPID();
  bool setProPlusGoalCurrent();
  bool setProPlusGoalPosition(int ticks);
  int getPresentPosition();
  double limitSteeringAngle(double desired_steering_angle);
  int angleToTicks(double steering_angle);
  double ticksToAngle(int ticks);

  bool ping();

  //debug purpose
  bool debug_flag = 0; //1: skip the writing to dynamixel parts 
  long int frame_num = 0;
  std::string dxl_comm_error;
  void printStatus();
  int max_current_steering = -69;
  bool is_communication_error = false;
  bool is_communication_error_prev = false;
  bool is_initialized = false;

  //read values
  int present_position_value = 0 ;
  double present_steering_angle =0;

  //set values 
  double goal_steering_angle = 0;
  int goal_position_value = 0;
};

DynamixelProPlusSteeringController::DynamixelProPlusSteeringController()
{
  ros::NodeHandle private_nh("~");

  std::string controller_msg_topic;
  std::string present_steering_angle_topic;
  std::string steering_health_topic; 

  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("present_steering_angle_topic", present_steering_angle_topic));
  ROS_ASSERT(private_nh.getParam("publish_steering_period", publish_steering_period));
  ROS_ASSERT(private_nh.getParam("steering_health_topic", steering_health_topic));
  ROS_ASSERT(private_nh.getParam("dxl_id", dxl_id));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));
  ROS_ASSERT(private_nh.getParam("wheel_center_offset", WHEEL_CENTER_OFFSET));
  ROS_DEBUG("The wheel center offset is %f radians", WHEEL_CENTER_OFFSET);

  // Dynamixel needs to be ready before subscribing to other topics
  if (! debug_flag)
    ROS_ASSERT_MSG(setupDyanmixelSteeringMotor(), "Steering Dynamixel: Startup failure, please reset.");

  steering_controller_msg_sub =
      nh.subscribe(controller_msg_topic, 1, &DynamixelProPlusSteeringController::controllerMsgCallback, this);
  present_steering_angle_pub = nh.advertise<std_msgs::Float64> (present_steering_angle_topic,1);
  steering_health_pub = nh.advertise<agv::ActuatorHealth> (steering_health_topic,1);
  timer = nh.createTimer(ros::Duration(publish_steering_period), &DynamixelProPlusSteeringController::timerCallback, this);
}

bool DynamixelProPlusSteeringController::setupDyanmixelSteeringMotor()
{  
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  ROS_ASSERT(port_handler->openPort());
  ROS_ASSERT(port_handler->setBaudRate(baud_rate));
  ROS_ASSERT(enableSteeringDynamixelTorque());
  ROS_ASSERT(setSteeringDynamixelSpeed());
  ROS_ASSERT(setSteeringMotorToStartPosition());
  ROS_ASSERT(setProPlusPID());
  ROS_ASSERT(setProPlusGoalCurrent());

  ROS_DEBUG("Steering Dynamixel ready for operation.");
  is_initialized = true;

  return true;
}

bool DynamixelProPlusSteeringController::enableSteeringDynamixelTorque()
{
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

bool DynamixelProPlusSteeringController::setSteeringDynamixelSpeed()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_MOVING_SPEED, (uint32_t)2900, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

bool DynamixelProPlusSteeringController::setSteeringMotorToStartPosition()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t)0, &dxl_error);
  // dxl_comm_result = packet_handler->write4ByteTxRx(
  //     port_handler, dxl_id, ADDR_PRO_PLUS_GOAL_POSITION,
  //     (uint32_t)(WHEEL_CENTER_OFFSET * -STEERING_ANGLE_TO_GOAL_POSITION_CONST), &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

bool DynamixelProPlusSteeringController::setProPlusPID()
{
  bool error_flag_ = 0;
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 532, (uint16_t)32, &dxl_error);
  if (! checkForPacketHandlerErrors()) //if error occur:
  {
    error_flag_ = 1;
  }

  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 530, (uint16_t)0, &dxl_error);
  if (! checkForPacketHandlerErrors()) //if error occur:
  {
    error_flag_ = 1;
  }

  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 528, (uint16_t)32, &dxl_error);
  if (! checkForPacketHandlerErrors()) //if error occur:
  {
    error_flag_ = 1;
  }  
  
  if (error_flag_) 
  {
  }
  
  return true;
}

bool DynamixelProPlusSteeringController::setProPlusGoalCurrent()
{
  // Unit: mA
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 550, (uint16_t)12000, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

//set the motor position (ticks)
bool DynamixelProPlusSteeringController::setProPlusGoalPosition(int goal_position_value)
{
  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_GOAL_POSITION,
                                                   (uint32_t)goal_position_value, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

//get current position (ticks) of the motor 
int DynamixelProPlusSteeringController::getPresentPosition()
{
  int32_t data_; //funny thing is, the sdk writes the position value to an unsigned int? 
  dxl_comm_result = packet_handler->read4ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_PRESENT_POSITION, 
                                                  (uint32_t*)&data_, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return data_;
  }
  //return -1;
}

//Ping the dynamixel 
//Return true: dynamixel is working. false: dynamixel has error/ not responsive.
bool DynamixelProPlusSteeringController::ping()
{
  dxl_comm_result = packet_handler->ping(port_handler, dxl_id, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  else
  {
    return false;
  }  
}

/*
 * This function:
 * 1. checks for any errors from the Write operations to the Dynamixel Motor
 * 2. prints the errors via ROS_ERROR
 * 3a. returns false if errors are found OR 3b. returns true if no errors are found
 */
bool DynamixelProPlusSteeringController::checkForPacketHandlerErrors()
{
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::string error_msg_;
    error_msg_.append(packet_handler->getTxRxResult(dxl_comm_result));
    dxl_comm_error += error_msg_;
    
    is_communication_error = true;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::string error_msg_;
    error_msg_.append(packet_handler->getRxPacketError(dxl_error));
    dxl_comm_error += error_msg_;
  
    is_communication_error = true;
    return false;
  }
  else 
  {
    is_communication_error = false;
    dxl_comm_error = "no error";
  }

  return true;
}

//returns steering angle that does not exceed mechanical limits 
double DynamixelProPlusSteeringController::limitSteeringAngle(double desired_steering_angle)
{
  if (desired_steering_angle > STEERING_ANGLE_LIMIT)
  {
    return STEERING_ANGLE_LIMIT;
  }
  else if (desired_steering_angle < -STEERING_ANGLE_LIMIT)
  {
    return -STEERING_ANGLE_LIMIT;
  }
  else
  {
    return desired_steering_angle;
  }
}

//converts steering angle (rad) into dynamixel motor goal_position (ticks)
int DynamixelProPlusSteeringController::angleToTicks(double steering_angle)
{
  return (steering_angle + WHEEL_CENTER_OFFSET) * -STEERING_ANGLE_TO_GOAL_POSITION_CONST;
}

//converts dynamixel motor goal_position (ticks) into steering angle (rad)
double DynamixelProPlusSteeringController::ticksToAngle(int ticks)
{
  return (ticks / -STEERING_ANGLE_TO_GOAL_POSITION_CONST) - WHEEL_CENTER_OFFSET;
}

void DynamixelProPlusSteeringController::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  //printStatus();
 
  //send new position to dynamixel
  goal_steering_angle = limitSteeringAngle(controller_msg->angular.z);
  goal_position_value = angleToTicks(goal_steering_angle);
  // ROS_DEBUG("Steering Dynamixel: Position value: %f", goal_position_value);

  //ROS_DEBUG("Steering Dynamixel: Debug: goal_steering_angle: %f, goal_position_value: %d", goal_steering_angle,goal_position_value);

  if (! debug_flag)
  {
    setProPlusGoalPosition(goal_position_value);
  }
  return;
}

void DynamixelProPlusSteeringController::timerCallback(const ros::TimerEvent& timer_event)
{
  //initial setup - if you do not set it up the porthandler and packethandler you may get error -11
  if (is_initialized == false) 
  {
    setupDyanmixelSteeringMotor();
    ROS_INFO("DynamixelProPlusSteeringController: initial setup done.");
  }
  else if (is_initialized == true)
  {
    //if we are continually in a error state, keep pinging the motor.
    if (is_communication_error == true and is_communication_error_prev == true) 
    {
      //ROS_INFO("DynamixelProPlusSteeringController: Lost connection to dynamixel: pinging...");
      ping();
    }

    //set up the motor again if we are back to health.
    if (is_communication_error == false and is_communication_error_prev == true) 
    {
      setupDyanmixelSteeringMotor();
      ROS_ERROR("DynamixelProPlusSteeringController: Recovery from failure attempted!");
    }

    //read the motor status during operation.
    if (is_communication_error == false)
    {      
      //read present position from dynamixel 
      present_position_value = getPresentPosition();
      present_steering_angle = ticksToAngle(present_position_value);
      
      //publish the present steering angle
      std_msgs::Float64 steering_angle_msg;
      steering_angle_msg.data = present_steering_angle;
      present_steering_angle_pub.publish(steering_angle_msg);
    }
  }
  
  //publish if we are healthy (no dynamixel transmission error.)
  agv::ActuatorHealth health_msg;
  if ((is_communication_error == true) or (is_initialized == false)) 
  { 
    health_msg.health_ok = false;
  }
  else 
  {
    health_msg.health_ok = true;
  }
  health_msg.error = dxl_comm_error;
  steering_health_pub.publish(health_msg);
  dxl_comm_error = "";
  
  //reset health 
  is_communication_error_prev = is_communication_error;


}


void DynamixelProPlusSteeringController::printStatus() 
{
  //print the errors, then print general status
  std::string output_error_;
  output_error_.append("\n");
  output_error_.append("========DynamixelProPlusSteeringController, Frame " + std::to_string(frame_num) + " ====");
  output_error_.append("\n");
  output_error_.append("========Errors: ======");
  output_error_.append("\n");


  std::string output_info_;

  output_info_.append("========Info: ======");
  output_error_.append("\n");
  
  //print out the values we set
  output_info_.append("Set steering angle: ");
  output_info_.append(std::to_string(goal_steering_angle));
  output_info_.append("\n");

  //print out the values we read
  output_info_.append("Present position: ");
  output_info_.append(std::to_string(present_position_value));
  output_info_.append("\n");

  output_info_.append("Present steering angle: ");
  output_info_.append(std::to_string(present_steering_angle));
  output_info_.append("\n");

  output_info_.append("Max current: TODO");
  //output_info_.append(std::to_string(max_current_brake));
  output_info_.append("\n");

  ROS_INFO_STREAM(output_error_ + output_info_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_proplus_steering_controller_node");
  DynamixelProPlusSteeringController dynamixel_proplus_steering_controller_obj;
  ros::spin();
  return 0;
}
