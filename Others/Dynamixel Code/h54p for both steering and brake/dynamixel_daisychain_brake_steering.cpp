/*
 * This node runs the steering and brake dynamixels, using command message from joystick teleop node.
 * 
 * For 2x Dynamixel H54P-200-S500-R, Dynamixel Protocol 2.0
 * http://emanual.robotis.com/docs/en/dxl/pro_plus/h54p-200-s500-r/
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

#define DEBUG_WITHOUT_HARDWARE 0 //1 to remove dynamixel functions for debug
#define DEBUG 0

#define DAISY_CHAIN_TRUE 0 //set to 1: the motors are connected in daisy chain 0: if the motors are connected to separate U2D2

//This class, on instantiation, subscribes to the controller message from joystickteleop node
//Use the getter methods to get the present value of steering angle and to know if need to brake
class SubscriberClass
{
public:
  SubscriberClass();

  double getSteeringAngle() {
    return steering_angle;
  }
  double getHardbrake() {
    return hard_brake;
  }
  double getSoftbrake() {
    return soft_brake;
  }
  double getSpeed() {
    return speed;
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber steering_controller_msg_sub;

  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);

  double steering_angle = 0;
  double hard_brake = 1; //braking at initialisation
  double soft_brake = 1;
  double speed = -1; //TODO: to implment, if necessary.
};

SubscriberClass::SubscriberClass()
{
  ros::NodeHandle private_nh("~");

  std::string controller_msg_topic;

  //read rosparameter from launch file
  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  
  //subscribe to the controller topic
  steering_controller_msg_sub =
    nh.subscribe(controller_msg_topic, 1, &SubscriberClass::controllerMsgCallback, this);
    ROS_DEBUG("SubscriberClass: Subscriber initialized.");
}

//Callback function that writes the data in the message to the private class variables
void SubscriberClass::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
    ROS_INFO("callback triggered.");
  //steering
  steering_angle = controller_msg->angular.z;

  //brake
  if (controller_msg->linear.z == 1.0) //hard brake desired
  {
    hard_brake = true;
  } 
  else
  {
    hard_brake = false;
  }
  
  if (controller_msg->linear.y == 1.0)  // soft brake desired
  {
    soft_brake = true;
  }
  else
  {
    soft_brake = false;
  }
}

// Calculates steering.
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
  ROS_DEBUG("Dynamixel Node: Calculated goal position, Steering: %f", goal_position_value);
  return (int) goal_position_value;
}




//This class sends the packets to daisy chained dynamixels, using the dynamixel sdk.
class DynamixelDriver
{
public:
  DynamixelDriver();

  //functions for other class to drive the dynamixels
  bool setSteeringGoalPosition(int goal_position);
  bool setBrakeGoalPosition(int goal_position);

  //sync write fcunctions
  bool setSyncWriteStorage(int steering_goal , bool brake_engage_flag);
  bool syncWriteToDynamixels();
  
  //read functions
  int getBrakePresentCurrent();

private:
  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  dynamixel::GroupSyncWrite group_sync_write = 
    dynamixel::GroupSyncWrite(port_handler, packet_handler, ADDR_PRO_PLUS_GOAL_POSITION, LEN_PRO_PLUS_GOAL_POSITION); ; 

  int dxl_id_steering;
  int dxl_id_brake;
  std::string device_port;
  int baud_rate;

  //launch file Parameters
  int goal_current_brake;
  int goal_position_brake;
  int default_position_brake;

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
  bool setBrakeGoalCurrent();

};

DynamixelDriver::DynamixelDriver()
{
  ros::NodeHandle private_nh("~");

  //get rosparameters
  ROS_ASSERT(private_nh.getParam("dxl_id_steering", dxl_id_steering));
  ROS_ASSERT(private_nh.getParam("dxl_id_brake", dxl_id_brake));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));
  ROS_ASSERT(private_nh.getParam("goal_current_brake", goal_current_brake));
  ROS_ASSERT(private_nh.getParam("goal_position_brake", goal_position_brake));
  ROS_ASSERT(private_nh.getParam("default_position_brake", default_position_brake));
  
  if (!DEBUG_WITHOUT_HARDWARE)
  {
    // Dynamixel needs to be ready before subscribing to other topics
    ROS_ASSERT_MSG(setupDyanmixels(), "DynamixelDriver: Startup failure, please reset.");
  }
};

bool DynamixelDriver::setupDyanmixels()
{
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  //group_sync_write = dynamixel::GroupSyncWrite::GroupSyncWrite(port_handler, packet_handler, ADDR_PRO_PLUS_GOAL_POSITION, LEN_PRO_PLUS_GOAL_POSITION);
  group_sync_write = dynamixel::GroupSyncWrite(port_handler, packet_handler, ADDR_PRO_PLUS_GOAL_POSITION, LEN_PRO_PLUS_GOAL_POSITION);

  //TODO: read position

  //Assert because both dynamixels require these lines to work. (the node will just keep restarting and attempting to open port)
  ROS_ASSERT(port_handler->openPort()); 
  ROS_ASSERT(port_handler->setBaudRate(baud_rate));

  //Initial setup uses normal (non-sync) write functions.
  //setup steering
  ROS_ASSERT(enableSteeringTorque()); 
  ROS_ASSERT(setSteeringSpeed()); 
  ROS_ASSERT(setSteeringToStartPosition());
  ROS_ASSERT(setSteeringPID());
  ROS_ASSERT(setSteeringGoalCurrent());
  //setup brake
  ROS_ASSERT(enableBrakeTorque()); 
  ROS_ASSERT(setBrakeGoalCurrent());

  ROS_DEBUG("DynamixelDriver ready for operation.");

  return true;
}

bool DynamixelDriver::enableSteeringTorque()
{
  //enable steering torque
  dxl_comm_result = 
    packet_handler->write1ByteTxRx(port_handler, dxl_id_steering, ADDR_PRO_PLUS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

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
    packet_handler->write1ByteTxRx(port_handler, dxl_id_brake, ADDR_PRO_PLUS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

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
      packet_handler->write4ByteTxRx(port_handler, dxl_id_steering, ADDR_PRO_PLUS_MOVING_SPEED, (uint32_t)2900, &dxl_error);
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
      packet_handler->write4ByteTxRx(port_handler, dxl_id_steering, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t)0, &dxl_error);
  // dxl_comm_result = packet_handler->write4ByteTxRx(
  //     port_handler, dxl_id, ADDR_PRO_PLUS_GOAL_POSITION,
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
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id_steering, 532, (uint16_t)32, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id_steering, 530, (uint16_t)0, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id_steering, 528, (uint16_t)32, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
  ROS_DEBUG("Steering Dynamixel: Steering PID set.");
  return true;
}

bool DynamixelDriver::setSteeringGoalCurrent()
{
  // Unit: mA
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id_steering, 550, (uint16_t)12000, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Steering goal current set.");
    return true;
  }
  return false;
}


bool DynamixelDriver::setBrakeGoalCurrent()
{
  // Unit: mA
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id_brake, 550, (uint16_t)goal_current_brake, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Brake goal current set.");
    return true;
  }
  return false;
}

int DynamixelDriver::getBrakePresentCurrent()
{
  // Unit: mA
  int16_t dxl_present_current;
  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler, dxl_id_brake, ADDR_PRO_PLUS_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Brake current read.");
    return dxl_present_current;
  }
  return -1;
}

// Pushes the values to be written to each motor into the `Syncwrite storage`
// Required to sync the daisy chained motors.
// Run this to push the goal positions into a packet, then run `syncWriteToDynamixels()` to send the packet. 
bool DynamixelDriver::setSyncWriteStorage(int steering_goal , bool brake_engage_flag)
{
  // Allocate goal position value into byte array
  param_goal_position_steering[0] = DXL_LOBYTE(DXL_LOWORD(steering_goal));
  param_goal_position_steering[1] = DXL_HIBYTE(DXL_LOWORD(steering_goal));
  param_goal_position_steering[2] = DXL_LOBYTE(DXL_HIWORD(steering_goal));
  param_goal_position_steering[3] = DXL_HIBYTE(DXL_HIWORD(steering_goal));

  if (brake_engage_flag)
  {
    param_goal_position_brake[0] = DXL_LOBYTE(DXL_LOWORD(goal_position_brake));
    param_goal_position_brake[1] = DXL_HIBYTE(DXL_LOWORD(goal_position_brake));
    param_goal_position_brake[2] = DXL_LOBYTE(DXL_HIWORD(goal_position_brake));
    param_goal_position_brake[3] = DXL_HIBYTE(DXL_HIWORD(goal_position_brake));
    ROS_INFO("SyncWrite Storage: brake engaged: %d", goal_position_brake); 
  }
  else
  {
    param_goal_position_brake[0] = DXL_LOBYTE(DXL_LOWORD(default_position_brake));
    param_goal_position_brake[1] = DXL_HIBYTE(DXL_LOWORD(default_position_brake));
    param_goal_position_brake[2] = DXL_LOBYTE(DXL_HIWORD(default_position_brake));
    param_goal_position_brake[3] = DXL_HIBYTE(DXL_HIWORD(default_position_brake));
    ROS_INFO("SyncWrite Storage: brake NOT engaged");
  }
  

  // Add Steering dynamixel goal position value to the Syncwrite storage
  dxl_addparam_result = group_sync_write.addParam(dxl_id_steering,  param_goal_position_steering);
  if (dxl_addparam_result != true)
  {
    ROS_INFO("[ID:%03d] groupSyncWrite addparam failed", dxl_id_steering);
    return 0;
  }
  ROS_INFO("SyncWrite Storage: Steering goal %d ", steering_goal); 
  ROS_INFO("SyncWrite Storage: Steering to Write %d %d %d %d ", 
    param_goal_position_steering[0],
    param_goal_position_steering[1],
    param_goal_position_steering[2],
    param_goal_position_steering[3]
    );

  // Add Brake dynamixel goal position value to the Syncwrite storage
  dxl_addparam_result = group_sync_write.addParam(dxl_id_brake, param_goal_position_brake);
  if (dxl_addparam_result != true)
  {
    ROS_INFO("[ID:%03d] groupSyncWrite addparam failed", dxl_id_brake);
    return 0;
  }
  ROS_INFO("SyncWrite Storage: Brake goal: %d ",  0); 
  ROS_INFO("SyncWrite Storage: Brake to write: %d %d %d %d ", 
    param_goal_position_brake[0],
    param_goal_position_brake[1],
    param_goal_position_brake[2],
    param_goal_position_brake[3]
    ); 
}

//write the SyncWrite storage into the dynamixels (ie move the dynamixels)
bool DynamixelDriver::syncWriteToDynamixels()
{
  // Syncwrite the goal position
  dxl_comm_result = group_sync_write.txPacket();

  // Clear syncwrite parameter storage
  group_sync_write.clearParam();

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Dynamixel Driving: Both goal position set.");
    return true;
  }
  return false;
}

bool DynamixelDriver::setSteeringGoalPosition(int goal_position)
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id_steering, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t) goal_position, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Steering position set to %d.", goal_position);
    return true;
  }
  return false;
}

bool DynamixelDriver::setBrakeGoalPosition(int goal_position)
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id_brake, ADDR_PRO_PLUS_GOAL_POSITION, (uint32_t) goal_position, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("DynamixelDriver: Brake position set to %d.", goal_position);
    return true;
  }
  return false;
}

//This wrapper class is the main class, it drives the other classes of the node.
class DynamixelDaisyChainController
{
public:
  DynamixelDaisyChainController();
  void update();
private:
  ros::NodeHandle nh;
  
  //instantiate subscriber class. To listen for command messages.
  SubscriberClass subscriber_class;

  //instantiate the logic nodes. to translate raw message data to data to write to dynamixel registers.
  SteeringLogic steering_logic;

  //instantiate the dynamixel-communication node
  DynamixelDriver dynamixel_driver;

  //timer 
  void timerCallback(const ros::TimerEvent&);
  ros::Timer timer;
  //raw values that we obtain from the command message.
  double steering_angle;
  double hard_brake; 
  double soft_brake;
  double speed;

  //parameters to read from launch file
  int goal_current_brake;
  int goal_position_brake;
  int default_position_brake;
};


DynamixelDaisyChainController::DynamixelDaisyChainController()
{
  ros::NodeHandle private_nh("~");

  ROS_ASSERT(private_nh.getParam("goal_current_brake", goal_current_brake));
  ROS_ASSERT(private_nh.getParam("goal_position_brake", goal_position_brake));
  ROS_ASSERT(private_nh.getParam("default_position_brake", default_position_brake));
  
  std::string controller_msg_topic;

  //instnatiate the logic and dynamixel-communication clases (moved to the class definition)
  
  timer = nh.createTimer(ros::Duration(0.1), &DynamixelDaisyChainController::timerCallback, this);
  ROS_DEBUG("DynamixelDaisyChainController: DynamixelDaisyChainController initialized.");
  
}

void DynamixelDaisyChainController::timerCallback(const ros::TimerEvent&)
{
  update();
}

//This function will update the dynamixel's position once, based on the latest requested position.
void DynamixelDaisyChainController::update()
{
  //get data from subscriber class
  steering_angle = subscriber_class.getSteeringAngle();
  hard_brake = subscriber_class.getHardbrake();
  soft_brake = subscriber_class.getSoftbrake();
  speed = subscriber_class.getSpeed();

  ROS_DEBUG("data obtained: | angle: %f | hard_brake: %f | soft_brake: %f ", steering_angle, hard_brake, soft_brake);

  // get the value to be sent to the motors.
  // steering value is int value to write in register. 
  // steering_angle is in radians
  int steering_value = (int)steering_logic.getSteeringPositionFromAngle(steering_angle);
  int brake_value =0;
  bool brake_flag = 1;
  if (hard_brake || soft_brake) 
  {
    brake_flag = 1;
  }
  else 
  {
    brake_flag =0;
  }

  ROS_INFO("Write: | steering: %d | brake: %d", steering_value, brake_flag);
  
  //debug: skip the dyanamixel parts
  if (DEBUG_WITHOUT_HARDWARE)
  {
    return; 
  }

  if (DAISY_CHAIN_TRUE)
  {
    //set sync write storage (load the values to be written to the dynamixels-but not write them.)
    dynamixel_driver.setSyncWriteStorage(steering_value,brake_flag);

    //move the dynamixels
    dynamixel_driver.syncWriteToDynamixels();
    ROS_INFO("Wrote: | steering: %d | brake engaged?: %d", steering_value, brake_flag);
  
    // ROS_INFO("Present value: | brake current: %d", brake_present_current);
  } 
  else  
  {
    //write
    dynamixel_driver.setSteeringGoalPosition(steering_value);
    if (brake_flag)
    { 
      dynamixel_driver.setBrakeGoalPosition(goal_position_brake);
    }
    else
    {
      dynamixel_driver.setBrakeGoalPosition(default_position_brake);
    }
    //read current
    dynamixel_driver.getBrakePresentCurrent();
  }
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_daisychain_brake_steering_node");
  DynamixelDaisyChainController dynamixel_daisy_chain_controller;
  ros::spin();
  return 0;
}