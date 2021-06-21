/*
 * This node runs the steering and brake dynamixels, using command message from joystick teleop node.
 * 
 * For 2x Dynamixel H54P-200-S500-R, Dynamixel Protocol 2.0
 * http://emanual.robotis.com/docs/en/dxl/pro_plus/h54p-200-s500-r/
 * TODO: Refer to documentation dynamixel_steering_controller.adoc for more details
 * 
 * How the code works? Note that I did not use static classes so need to instantiate each one, even if it is just a utility class.
 * DynamixelMultiportControllerBrakeSteering: a wrapper class that instantiate and contains the following classes:
 * |--SubscriberClass: subscribes to the controller topic /cmd_vel_out
 * |--SteeringLogic: contains function to convert steering radians to steering 'ticks' (that is sent to dynamixel)
 * |--DynamixelDriver: contains function to convert steering radians to steering 'ticks' (that is sent to dynamixel)
 * 
 */

//#include "../include/agv/dynamixel_daisychain_brake_steering.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdint>

#define PROTOCOL_VERSION 2.0

#define ADDR_PRO_PLUS_TORQUE_ENABLE 512
#define ADDR_PRO_PLUS_GOAL_POSITION 564
#define LEN_PRO_PLUS_GOAL_POSITION 4
#define ADDR_PRO_PLUS_PRESENT_CURRENT 574

//for steering
#define ADDR_PRO_PLUS_MOVING_SPEED 552
//for brake
#define ADDR_PRO_PLUS_PRESENT_POSITION 580
#define ADDR_PRO_PLUS_GOAL_CURRENT 550

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1
#define COMM_SUCCESS 0

#define STEERING_ANGLE_TO_GOAL_POSITION_CONST 3195340.53  // TODO
#define STEERING_ANGLE_LIMIT 0.610865

#define DEBUG_WITHOUT_HARDWARE 0   //1 to remove dynamixel functions for debug
#define DEBUG 0

// Contains function that convert steering radians to dynamixel steering 'tick' values.
class SteeringLogic
{
public:
  SteeringLogic();
  int getSteeringPositionFromAngle(double desired_steering_angle);
private:
  double WHEEL_CENTER_OFFSET = 0;
};

SteeringLogic::SteeringLogic()
{
  ros::NodeHandle private_nh("~");

  ROS_ASSERT(private_nh.getParam("wheel_center_offset", WHEEL_CENTER_OFFSET));
  ROS_DEBUG("The wheel center offset is %f radians", WHEEL_CENTER_OFFSET);
 }

// Converts steering angle (radians) into a goal_position value for the steering dynamixel (int)
// @param desired_steering_angle: steering angle, in Radians
// @return The value to write into the dynamixel as goal_position.
int SteeringLogic::getSteeringPositionFromAngle(double desired_steering_angle)
{
  //limit steering angle (physical limit)
  if (desired_steering_angle > STEERING_ANGLE_LIMIT)
  {
    desired_steering_angle = STEERING_ANGLE_LIMIT;
  }
  else if (desired_steering_angle < -STEERING_ANGLE_LIMIT)
  {
    desired_steering_angle = -STEERING_ANGLE_LIMIT;
  }
  else
  {
    //nothing changes
  }

  double goal_position_value = (desired_steering_angle + WHEEL_CENTER_OFFSET) * -STEERING_ANGLE_TO_GOAL_POSITION_CONST;
  return (int) goal_position_value;
}




//Class for communication with dynamixel. It sends the packets to multiport dynamixels, using the dynamixel sdk.
class DynamixelDriver
{
public:
  DynamixelDriver();

  //functions for other class to drive the dynamixels
  bool setSteeringGoalPosition(int goal_position);
  bool setBrakeGoalPosition(int goal_position);
  bool setBrakeGoalCurrent(int goal_current);

  //sync write fcunctions
  bool setSyncWriteStorage(int steering_goal , bool brake_engage_flag);
  bool syncWriteToDynamixels();
  
  //read functions
  int getBrakePresentCurrent();
  int getSteeringPresentCurrent();

private:
  dynamixel::PortHandler* port_handler_brake;
  dynamixel::PortHandler* port_handler_steering;
  dynamixel::PacketHandler* packet_handler;

  // dynamixel::GroupSyncWrite group_sync_write = 
  //   dynamixel::GroupSyncWrite(port_handler_brake, packet_handler, ADDR_PRO_PLUS_GOAL_POSITION, LEN_PRO_PLUS_GOAL_POSITION); ; 

  int dxl_id_steering;
  int dxl_id_brake;
  std::string device_port_brake;
  std::string device_port_steering;
  int baud_rate;


  //for errors and result reporting
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  //for sync write/read.
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result
  uint8_t param_goal_position_brake[4];
  uint8_t param_goal_position_steering[4];

  bool setupDyanmixels();
  bool checkForPacketHandlerErrors();
  //steering
  bool enableSteeringTorque();
  bool setSteeringSpeed();
  bool setSteeringToStartPosition();
  bool setSteeringPID();
  bool setSteeringGoalCurrent();
  bool setSteeringMotorToGoalPosition();
  //brake
  bool enableBrakeTorque();
  
  bool setBrakePID(); 
};

DynamixelDriver::DynamixelDriver()
{
  ros::NodeHandle private_nh("~");

  //get rosparameters
  ROS_ASSERT(private_nh.getParam("dxl_id_steering", dxl_id_steering));
  ROS_ASSERT(private_nh.getParam("dxl_id_brake", dxl_id_brake));
  ROS_ASSERT(private_nh.getParam("device_port_brake", device_port_brake));
  ROS_ASSERT(private_nh.getParam("device_port_steering", device_port_steering));
  
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));
  // ROS_ASSERT(private_nh.getParam("default_position_brake", default_position_brake));
  
  if (!DEBUG_WITHOUT_HARDWARE)
  {
    // Dynamixel needs to be ready before subscribing to other topics
    ROS_ASSERT_MSG(setupDyanmixels(), "DynamixelDriver: Startup failure, please reset.");
  }
};

bool DynamixelDriver::setupDyanmixels()
{
  port_handler_brake = dynamixel::PortHandler::getPortHandler(device_port_brake.c_str());
  port_handler_steering = dynamixel::PortHandler::getPortHandler(device_port_steering.c_str());
  
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  //group_sync_write = dynamixel::GroupSyncWrite::GroupSyncWrite(port_handler_brake, packet_handler, ADDR_PRO_PLUS_GOAL_POSITION, LEN_PRO_PLUS_GOAL_POSITION);
  // group_sync_write = dynamixel::GroupSyncWrite(port_handler_brake, packet_handler, ADDR_PRO_PLUS_GOAL_POSITION, LEN_PRO_PLUS_GOAL_POSITION);

  ROS_ASSERT(port_handler_brake->openPort()); 
  ROS_ASSERT(port_handler_brake->setBaudRate(baud_rate));
  ROS_ASSERT(port_handler_steering->openPort()); 
  ROS_ASSERT(port_handler_steering->setBaudRate(baud_rate));

  //Initial setup uses normal (non-sync) write functions.
  //setup steering
  ROS_ASSERT(enableSteeringTorque()); 
  ROS_ASSERT(setSteeringSpeed()); 
  ROS_ASSERT(setSteeringToStartPosition());
  ROS_ASSERT(setSteeringPID());
  ROS_ASSERT(setSteeringGoalCurrent());
  //setup brake
  ROS_ASSERT(enableBrakeTorque()); 
  // ROS_ASSERT(setBrakeGoalCurrent( 0 ));
  ROS_ASSERT(setBrakePID());

  ROS_DEBUG("DynamixelDriver ready for operation.");

  return true;
}

bool DynamixelDriver::enableSteeringTorque()
{
  //enable steering torque
  dxl_comm_result = 
    packet_handler->write1ByteTxRx(port_handler_steering, dxl_id_steering, ADDR_PRO_PLUS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Steering torque enabled.");
    return true;
  }
  return false;
}

bool DynamixelDriver::enableBrakeTorque()
{
  //enable brake torque
  dxl_comm_result = 
    packet_handler->write1ByteTxRx(port_handler_brake, dxl_id_brake, ADDR_PRO_PLUS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Brake torque enabled.");
    return true;
  }
  return false;
}

bool DynamixelDriver::setSteeringSpeed()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler_steering, dxl_id_steering, ADDR_PRO_PLUS_MOVING_SPEED, (uint32_t)2900, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Steering speed set.");
    return true;
  }
  return false;
}
 
bool DynamixelDriver::setSteeringToStartPosition()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler_steering, dxl_id_steering, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t)0, &dxl_error);
  // dxl_comm_result = packet_handler->write4ByteTxRx(
  //     port_handler_brake, dxl_id, ADDR_PRO_PLUS_GOAL_POSITION,
  //     (uint32_t)(WHEEL_CENTER_OFFSET * -STEERING_ANGLE_TO_GOAL_POSITION_CONST), &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Steering initial position set.");
    return true;
  }
  return false;
}

bool DynamixelDriver::setSteeringPID()
{
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_steering, dxl_id_steering, 532, (uint16_t)32, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_steering, dxl_id_steering, 530, (uint16_t)0, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_steering, dxl_id_steering, 528, (uint16_t)32, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  ROS_DEBUG("Steering Dynamixel: Steering PID set.");
  return true;
}


bool DynamixelDriver::setBrakePID()
{
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_brake, dxl_id_brake, 532, (uint16_t)48, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_brake, dxl_id_brake, 530, (uint16_t)0, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_brake, dxl_id_brake, 528, (uint16_t)32, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  ROS_DEBUG("Steering Dynamixel: Brake PID set.");
  return true;
}

bool DynamixelDriver::setSteeringGoalCurrent()
{
  // Unit: mA
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_steering, dxl_id_steering, 550, (uint16_t)12000, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    // ROS_DEBUG("DynamixelDriver: Steering goal current set.");
    return true;
  }
  return false;
}


bool DynamixelDriver::setBrakeGoalCurrent(int goal_current)
{
  // Unit: mA
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler_brake, dxl_id_brake, 550, (uint16_t)goal_current, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    // ROS_DEBUG("DynamixelDriver: Brake goal current set.");
    return true;
  }
  return false;
}

int DynamixelDriver::getBrakePresentCurrent()
{
  // Unit: mA
  int16_t dxl_present_current;
  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler_brake, dxl_id_brake, ADDR_PRO_PLUS_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    // ROS_DEBUG("DynamixelDriver: Brake current read.");
    return dxl_present_current;
  }
  return -1;
}

int DynamixelDriver::getSteeringPresentCurrent()
{
  // Unit: mA
  int16_t dxl_present_current;
  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler_steering, dxl_id_steering, ADDR_PRO_PLUS_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    // ROS_DEBUG("DynamixelDriver: Steering current read.");
    return dxl_present_current;
  }
  return -1;
}
// Pushes the values to be written to each motor into the `Syncwrite storage`
// Required to sync the daisy chained motors.
// Run this to push the goal positions into a packet, then run `syncWriteToDynamixels()` to send the packet. 
// bool DynamixelDriver::setSyncWriteStorage(int steering_goal , bool brake_engage_flag)
// {
//   // Allocate goal position value into byte array
//   param_goal_position_steering[0] = DXL_LOBYTE(DXL_LOWORD(steering_goal));
//   param_goal_position_steering[1] = DXL_HIBYTE(DXL_LOWORD(steering_goal));
//   param_goal_position_steering[2] = DXL_LOBYTE(DXL_HIWORD(steering_goal));
//   param_goal_position_steering[3] = DXL_HIBYTE(DXL_HIWORD(steering_goal));

//   if (brake_engage_flag)
//   {
//     param_goal_position_brake[0] = DXL_LOBYTE(DXL_LOWORD(goal_position_brake));
//     param_goal_position_brake[1] = DXL_HIBYTE(DXL_LOWORD(goal_position_brake));
//     param_goal_position_brake[2] = DXL_LOBYTE(DXL_HIWORD(goal_position_brake));
//     param_goal_position_brake[3] = DXL_HIBYTE(DXL_HIWORD(goal_position_brake));
//     ROS_INFO("SyncWrite Storage: brake engaged: %d", goal_position_brake); 
//   }
//   else
//   {
//     param_goal_position_brake[0] = DXL_LOBYTE(DXL_LOWORD(default_position_brake));
//     param_goal_position_brake[1] = DXL_HIBYTE(DXL_LOWORD(default_position_brake));
//     param_goal_position_brake[2] = DXL_LOBYTE(DXL_HIWORD(default_position_brake));
//     param_goal_position_brake[3] = DXL_HIBYTE(DXL_HIWORD(default_position_brake));
//     ROS_INFO("SyncWrite Storage: brake NOT engaged");
//   }
  

//   // Add Steering dynamixel goal position value to the Syncwrite storage
//   dxl_addparam_result = group_sync_write.addParam(dxl_id_steering,  param_goal_position_steering);
//   if (dxl_addparam_result != true)
//   {
//     ROS_INFO("[ID:%03d] groupSyncWrite addparam failed", dxl_id_steering);
//     return 0;
//   }
//   ROS_INFO("SyncWrite Storage: Steering goal %d ", steering_goal); 
//   ROS_INFO("SyncWrite Storage: Steering to Write %d %d %d %d ", 
//     param_goal_position_steering[0],
//     param_goal_position_steering[1],
//     param_goal_position_steering[2],
//     param_goal_position_steering[3]
//     );

//   // Add Brake dynamixel goal position value to the Syncwrite storage
//   dxl_addparam_result = group_sync_write.addParam(dxl_id_brake, param_goal_position_brake);
//   if (dxl_addparam_result != true)
//   {
//     ROS_INFO("[ID:%03d] groupSyncWrite addparam failed", dxl_id_brake);
//     return 0;
//   }
//   ROS_INFO("SyncWrite Storage: Brake goal: %d ",  0); 
//   ROS_INFO("SyncWrite Storage: Brake to write: %d %d %d %d ", 
//     param_goal_position_brake[0],
//     param_goal_position_brake[1],
//     param_goal_position_brake[2],
//     param_goal_position_brake[3]
//     ); 
// }

//write the SyncWrite storage into the dynamixels (ie move the dynamixels)
// bool DynamixelDriver::syncWriteToDynamixels()
// {
//   // Syncwrite the goal position
//   dxl_comm_result = group_sync_write.txPacket();

//   // Clear syncwrite parameter storage
//   group_sync_write.clearParam();

//   if (checkForPacketHandlerErrors())
//   {
//     ROS_DEBUG("Dynamixel Driving: Both goal position set.");
//     return true;
//   }
//   return false;
// }

bool DynamixelDriver::setSteeringGoalPosition(int goal_position)
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler_steering, dxl_id_steering, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t) goal_position, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    // ROS_DEBUG("DynamixelDriver: Steering position set to %d.", goal_position);
    return true;
  }
  return false;
}

bool DynamixelDriver::setBrakeGoalPosition(int goal_position)
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler_brake, dxl_id_brake, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t) goal_position, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    // ROS_DEBUG("DynamixelDriver: Brake position set to %d.", goal_position);
    return true;
  }
  return false;
}

/*
 * This function:
 * 1. checks for any errors from the Write operations to the Dynamixel Motor
 * 2. prints the errors via ROS_ERROR
 * 3a. returns false if errors are found OR 3b. returns true if no errors are found
 */
bool DynamixelDriver::checkForPacketHandlerErrors()
{
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("DynamixelDriver: %s", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("DynamixelDriver: %s", packet_handler->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}



//This wrapper class is the main class, it drives the other classes of the node.
class DynamixelMultiportControllerBrakeSteering
{
public:
  DynamixelMultiportControllerBrakeSteering();
  void updateDynamixels();
private:
  ros::NodeHandle nh;

  //Subscriber
  ros::Subscriber steering_controller_msg_sub;
  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);

  //instantiate the logic nodes. to translate raw message data to data to write to dynamixel registers.
  SteeringLogic steering_logic;
  //instantiate the dynamixel-communication node
  DynamixelDriver dynamixel_driver;

  //helper function
  void printStatus();

  //The information that we read from in the command message.
  double steering_angle = 0;
  double brake_value = 1;

  //double hard_brake = 1; //ensure that we are braking at start.
  //double soft_brake = 1;
  double speed = -1; //TODO: not implemented. yet.


  //parameters to read from launch file
  int goal_current_soft_brake;
  int goal_current_hard_brake;
  int goal_position_brake;
  int default_position_brake;

  //params for debug print. To know what is the max reported current draw.
  int max_current_steering;
  int max_current_brake;
};


DynamixelMultiportControllerBrakeSteering::DynamixelMultiportControllerBrakeSteering()
{
  ros::NodeHandle private_nh("~");

  //read rosparameter from launch file
  ROS_ASSERT(private_nh.getParam("goal_current_soft_brake", goal_current_soft_brake));
  ROS_ASSERT(private_nh.getParam("goal_current_hard_brake", goal_current_hard_brake));
  ROS_ASSERT(private_nh.getParam("goal_position_brake", goal_position_brake));
  ROS_ASSERT(private_nh.getParam("default_position_brake", default_position_brake));

  std::string controller_msg_topic;
  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  
  //subscribe to the controller topic
  steering_controller_msg_sub =
    nh.subscribe(controller_msg_topic, 1, &DynamixelMultiportControllerBrakeSteering::controllerMsgCallback, this);
    ROS_DEBUG("SubscriberClass: Subscriber initialized.");

  ROS_DEBUG("DynamixelMultiportControllerBrakeSteering: DynamixelMultiportControllerBrakeSteering initialized.");
}

void DynamixelMultiportControllerBrakeSteering::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  //update the our variables with the latest controller message
  steering_angle = controller_msg->angular.z;
  brake_value = controller_msg->linear.z;

  updateDynamixels(); //write the new position/current to dynamixels.
}

//Updates the dynamixel's position once, based on the latest data to write to dynamixel.
void DynamixelMultiportControllerBrakeSteering::updateDynamixels()
{
  // convert from steering radians to dynamixel steering ticks
  int steering_value = (int)steering_logic.getSteeringPositionFromAngle(steering_angle);

  //pseudocode 
  /*
  if (pressing)
  {
    position is -inf
    goal current is prop to depression

  }
  // else if (pressing less in the joystick) 
  // {
  //   position is default??????/
  //   problem: do not know what position to write.

  // }
  else if (not press joystick) 
  {
    position is default
    goal current is max
  }
  */
 
  //figure out if we are braking or not
  float brake_threshold = 0.1; //must be 0.1-1.0 to engage the brake.
  bool brake_flag = (brake_value > brake_threshold); //brake flag because comparison of floats can be iffy
  ROS_INFO_STREAM("brake_flag : " + std::to_string(brake_flag));

  //prepare brake position and current
  int brake_position_value =0;
  int brake_goal_current; 
  if (brake_flag) //braking.
  {
    brake_position_value = goal_position_brake;
    
    //convert the brake_value (0-1) into braking dynamixel current 'ticks' (brake_goal_current value). Only accept values that are 0.1- 1.0.
    brake_goal_current = brake_value * goal_current_hard_brake;
    ROS_INFO_STREAM("Braking true");
  }
  else //not braking 
  {
    brake_position_value = default_position_brake;
    brake_goal_current = goal_current_hard_brake;
    ROS_INFO_STREAM("Braking false");
  }

  
  ROS_DEBUG("Command message Inputs: | angle: %f | brake_value: %f ", steering_angle, brake_value);

  ROS_INFO("Written position: | steering: %d | brake: %d", steering_value, brake_position_value);
  ROS_INFO("Written current: | brake: %d", brake_goal_current);
  

  //debug: skip the dyanmixel parts
  if (DEBUG_WITHOUT_HARDWARE)
  {
    return; 
  }

  //write to motors
  dynamixel_driver.setBrakeGoalCurrent(brake_goal_current); 
  dynamixel_driver.setBrakeGoalPosition(brake_position_value);
  dynamixel_driver.setSteeringGoalPosition(steering_value);

  //read from motors
  int present_current_brake = dynamixel_driver.getBrakePresentCurrent();
  int present_current_steering = dynamixel_driver.getSteeringPresentCurrent();
  
  //debug prints
  max_current_brake = std::max(abs(max_current_brake), abs(present_current_brake));
  max_current_steering = std::max(abs(max_current_steering), abs(present_current_steering));
  
  ROS_INFO("Present current: | steering: %d | brake: %d", present_current_steering,present_current_brake);
  ROS_INFO("Max current: | steering: %d | brake: %d", max_current_steering, max_current_brake);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_daisychain_brake_steering_node");
  DynamixelMultiportControllerBrakeSteering dynamixel_daisy_chain_controller;
  ros::spin();
  return 0;
}