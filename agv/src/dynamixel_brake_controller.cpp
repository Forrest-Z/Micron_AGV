/*
 * This node runs the brake dynamixel, using command message from joystick teleop node.
 * 
 * For Dynamixel H54P-200-S500-R, Dynamixel Protocol 2.0
 * http://emanual.robotis.com/docs/en/dxl/pro_plus/h54p-200-s500-r/
 * 
 * How the restart works: 1) We continually detect error by running checkForPacketHandlerErrors(). if there is error, is_communication_error is set to TRUE.
 *                        2) In timercallback, we check is_communication_error and is_communication_error_prev, and if certain state is met, we attempt to re-setup the dynamixel.
 *                        3) We set is_communication_error_prev = is_communication_error and continue to the next loop.
 *                        4) The logic is not great and I'm pretty sure I'm missing out some edge cases.
 * 
 * Issues:  1) In testing, restarting the motor sometimes does not work. Also, the logic for restarting the motor is quite bad.
 *          2) There is alot of repetition between brake and steering motor nodes. Consider putting the repeated parts into a class/library/header file.
 *          3) If you rostopic echo /health_monitor and move the brake, you can see that the health_points fluctuates from 0.8 to 1.2. I do not know why this happens, but my best guess is that the calls to dynamixelSDK might be blocking, thus delaying execution of the publisher. 
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdint>

#include <std_msgs/Bool.h>
#include <agv/ActuatorHealth.h>

#define PROTOCOL_VERSION 2.0

#define ADDR_PRO_PLUS_TORQUE_ENABLE 512
#define ADDR_PRO_PLUS_GOAL_POSITION 564
#define LEN_PRO_PLUS_GOAL_POSITION 4
#define ADDR_PRO_PLUS_PRESENT_CURRENT 574
#define ADDR_PRO_PLUS_PRESENT_POSITION 580

#define ADDR_PRO_PLUS_GOAL_VELOCITY 552
#define ADDR_PRO_PLUS_PRESENT_POSITION 580
#define ADDR_PRO_PLUS_GOAL_CURRENT 550

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1
#define COMM_SUCCESS 0

//Controls the brakes.
class DynamixelBrakeController
{
public:
  DynamixelBrakeController();
  
  //Other functions
  bool checkForPacketHandlerErrors();

  //This function calculates the new values to be written to the dynamixel.
  void updateWrittenVariables();

  //set dynamixel single parameter
  bool setGoalPosition(int goal_position);
  bool setGoalCurrent(int goal_current);  
  bool setGoalSpeed(int goal_speed);
  bool enableTorque();
  bool setPID(int p_, int i_, int d_); 

  //set dynamixel multiple variables
  bool setupDyanmixel();
  void setDynamixels();
  
  //Read dynamixel 
  int getPresentCurrent();
  int getPresentPosition();
  
  //Read dynamixel multiple variables
  void readDynamixels();

  bool ping();

private:
  ros::NodeHandle nh;

  //Subscriber & callback
  ros::Subscriber controller_msg_sub;
  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);
  std::string controller_msg_topic;
  ros::Timer timer;
  void timerCallback(const ros::TimerEvent& timer_event);
  ros::Publisher brake_health_pub;
  std::string brake_health_topic; 
  
  float brake_health_period;

  //From command message.
  double braking_intensity = 1;

  //parameters from launch file
  int goal_current_limit;
  int goal_position_current_braking;
  int goal_position_position_braking;
  int default_position;
  int goal_speed;
  int p_;
  int i_;
  int d_;
  int is_current_control;
  int max_position;

  float brake_threshold = 0.1; //must be 0.1-1.0 to engage the brake.
  bool brake_flag; //are we braking? floats are unreliable.

  //Debug: 
  int max_current_brake = -69;
  //read values from DXL
  int present_current_brake = -69;
  int present_position_brake = -69;

  //tracking health.
  bool is_communication_error = false;
  bool is_communication_error_prev = true;
  bool is_initialized = false;

  //Intermediate values that eventually get written to the DXL
  int goal_position_ = 0;
  int goal_current_ = 0; 

  //Debug prints
  void printStatus(); //prints dxl_comm_error_list.
  std::string dxl_comm_error;
  bool debug_flag = 0; //1: debug without connecting an actual motor.

  //dynamixel sdk stuff
  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;
  int dxl_id;
  std::string device_port;
  int baud_rate;
  //for errors and result reporting
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
};


DynamixelBrakeController::DynamixelBrakeController()
{
  ros::NodeHandle private_nh("~");

  //read rosparam from launch file
  ROS_ASSERT(private_nh.getParam("dxl_id", dxl_id));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));

  ROS_ASSERT(private_nh.getParam("goal_current_limit", goal_current_limit));
  ROS_ASSERT(private_nh.getParam("goal_position_current_braking", goal_position_current_braking));
  ROS_ASSERT(private_nh.getParam("goal_position_position_braking", goal_position_position_braking));
  ROS_ASSERT(private_nh.getParam("default_position", default_position));
  ROS_ASSERT(private_nh.getParam("goal_speed", goal_speed));
  ROS_ASSERT(private_nh.getParam("p", p_));
  ROS_ASSERT(private_nh.getParam("i", i_));
  ROS_ASSERT(private_nh.getParam("d", d_));

  ROS_ASSERT(private_nh.getParam("is_current_control", is_current_control));

  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("brake_health_topic", brake_health_topic));
  ROS_ASSERT(private_nh.getParam("brake_health_period", brake_health_period));

  
  
  //subscribe to the controller topic
  controller_msg_sub =
    nh.subscribe(controller_msg_topic, 1, &DynamixelBrakeController::controllerMsgCallback, this);
  timer = nh.createTimer(ros::Duration(brake_health_period), &DynamixelBrakeController::timerCallback, this);
  brake_health_pub = nh.advertise<agv::ActuatorHealth> (brake_health_topic,1);
  
  
  ROS_DEBUG("DynamixelBrakeController: DynamixelBrakeController initialized.");
}


bool DynamixelBrakeController::setupDyanmixel()
{
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  ROS_ASSERT(port_handler->openPort()); 
  ROS_ASSERT(port_handler->setBaudRate(baud_rate));

  //set variables in dynamixel
  ROS_ASSERT(enableTorque()); 
  ROS_ASSERT(setGoalSpeed(goal_speed)); 
  ROS_ASSERT(setPID(p_, i_, d_));

  ROS_DEBUG("DynamixelBrakeController: ready for operation.");
  is_initialized = true;

  return true;
}

bool DynamixelBrakeController::enableTorque()
{
  //enable brake torque
  dxl_comm_result = 
    packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

bool DynamixelBrakeController::setGoalSpeed(int goal_speed)
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_GOAL_VELOCITY, (uint32_t)goal_speed, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

bool DynamixelBrakeController::setPID(int p_, int i_, int d_)
{
  bool error_flag_ = 0;
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 532, (uint16_t)p_, &dxl_error);
  if (checkForPacketHandlerErrors() == 0) //if there was error, set the error flag.
    error_flag_ = 1;

  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 530, (uint16_t)i_, &dxl_error);
  if (checkForPacketHandlerErrors() == 0) //if there was error, set the error flag.
    error_flag_ = 1;

  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 528, (uint16_t)d_, &dxl_error);
  if (checkForPacketHandlerErrors() == 0) //if there was error, set the error flag.
    error_flag_ = 1;
  
  if (error_flag_ )
  {
    // addError("set PID");
  } 
  return true;
}

bool DynamixelBrakeController::setGoalCurrent(int goal_current)
{
  // Unit: mA
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, 550, (uint16_t)goal_current, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}

bool DynamixelBrakeController::setGoalPosition(int goal_position)
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t) goal_position, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    return true;
  }
  return false;
}


int DynamixelBrakeController::getPresentCurrent()
{
  // Unit: mA
  int16_t dxl_present_current;
  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return dxl_present_current;
  }
}

int DynamixelBrakeController::getPresentPosition()
{
  int16_t dxl_present_position;
  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler, dxl_id, ADDR_PRO_PLUS_PRESENT_CURRENT, (uint16_t*)&dxl_present_position, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    return dxl_present_position;
  }
}

//Ping the dynamixel 
//Return true: dynamixel is working. false: dynamixel has error/ not responsive.
bool DynamixelBrakeController::ping()
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
bool DynamixelBrakeController::checkForPacketHandlerErrors()
{
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::string error_msg_;
    //error_msg_.append("DynamixelBrakeController: ");
    error_msg_.append(packet_handler->getTxRxResult(dxl_comm_result));
    // dxl_comm_error_list.push_back(error_msg_);
    dxl_comm_error += error_msg_;

    is_communication_error = true;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::string error_msg_;
    //error_msg_.append("DynamixelBrakeController: ");
    error_msg_.append(packet_handler->getRxPacketError(dxl_error));
    // dxl_comm_error_list.push_back(error_msg_);
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

void DynamixelBrakeController::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  if (is_initialized == false)
  {
    return;
  } 
  
  braking_intensity = controller_msg->linear.z;
  //Braking_intensity must exceed 0.1 to be considered to be braking.
  brake_flag = (braking_intensity > brake_threshold);

  updateWrittenVariables(); //get new values
  setDynamixels(); //set dynamixel with the new values.
}

//Obtain the current and position to write to the brake motor, based on:
// 1) if we are braking
// 2) our braking mode (current vs position)
void DynamixelBrakeController::updateWrittenVariables()
{
  if (brake_flag) //braking.
  {
    if (is_current_control) //current control cannot release the brake properly after it is pushed down.
    {
      goal_position_ = goal_position_current_braking;
    
      //convert the brake_value (0-1) into braking dynamixel current 'ticks' (brake_goal_current value). Only accept values above the threshold.
      goal_current_ = braking_intensity * goal_current_limit;

    }
    else //position control.
    {
      goal_current_ = goal_current_limit;
      goal_position_ = default_position + (goal_position_position_braking - default_position)* braking_intensity;
    }
  }
  else //not braking 
  {
    if (is_current_control)
    {
      goal_position_ = default_position;
      goal_current_ = goal_current_limit*0.5; //to counteract (possible) PID I issue within the dyanmixel cpu
    }
    else
    {
      goal_current_ = goal_current_limit;
      goal_position_ = default_position;
    }
  }
}

void DynamixelBrakeController::timerCallback(const ros::TimerEvent& timer_event)
{
  //initial setup - if you do not set it up the porthandler and packethandler you may get error -11
  if (is_initialized == false) 
  {
    setupDyanmixel();
    ROS_INFO("DynamixelBrakeController: initial setup done.");
  }
  else if (is_initialized == true)
  {
    //if we are continually in a error state, keep pinging the motor.
    if (is_communication_error == true and is_communication_error_prev == true) 
    {
      //ROS_INFO("DynamixelBrakeController: Lost connection to dynamixel: pinging...");
      ping();
    }

    //set up the motor again if we are back to health.
    if (is_communication_error == false and is_communication_error_prev == true) 
    {
      setupDyanmixel();
      ROS_ERROR("DynamixelBrakeController: Recovery from failure attempted!");
    }

    //read the motor status during operation.
    if (is_communication_error == false)
    {
      readDynamixels(); 
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
  brake_health_pub.publish(health_msg);
  dxl_comm_error = "";
    
  //reset health 
  is_communication_error_prev = is_communication_error;
} 

//Updates the dynamixel's position + current once.
void DynamixelBrakeController::setDynamixels()
{
  //debug: skip the dyanmixel parts
  if (!debug_flag)
  {
    //write to motor
    setGoalCurrent(goal_current_); 
    setGoalPosition(goal_position_);
  }
}


//Updates the dynamixel's position + current once.
void DynamixelBrakeController::readDynamixels()
{
  //read from motor
  present_current_brake = getPresentCurrent();
  present_position_brake = getPresentPosition();
  //for debug
  max_current_brake = std::max(abs(max_current_brake), abs(present_current_brake));
}


void DynamixelBrakeController::printStatus() 
{
  //print the errors, then print general status
  std::string output_error_;
  output_error_.append("\n");
  output_error_.append("========DynamixelBrakeController  ====");
  output_error_.append("\n");
  output_error_.append("========Errors: ======");
  output_error_.append("\n");

  output_error_.append(dxl_comm_error);

  std::string output_info_;

  output_info_.append("========Info: ======");
  output_error_.append("\n");
  
  //R we braking?
  if (brake_flag)
    output_info_.append("State: We are braking \n");
  else
    output_info_.append("State: We are not braking \n");

  //print out the values we set
  output_info_.append("Goal current limit: ");
  output_info_.append(std::to_string(goal_current_limit));
  output_info_.append("\n");

  output_info_.append("Written current: ");
  output_info_.append(std::to_string(goal_current_));
  output_info_.append("\n");

  //print out the values we read
  output_info_.append("Present position: ");
  output_info_.append(std::to_string(present_position_brake));
  output_info_.append("\n");

  output_info_.append("Present current: ");
  output_info_.append(std::to_string(present_current_brake));
  output_info_.append("\n");

  output_info_.append("Max current: ");
  output_info_.append(std::to_string(max_current_brake));
  output_info_.append("\n");

  ROS_INFO_STREAM(output_error_ + output_info_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_brake_node");
  DynamixelBrakeController dynamixel_brake_controller;
  ros::spin();
  return 0;
}