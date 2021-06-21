/*
 * For Dynamixel MX-106, Dynamixel Protocol 2.0
 * http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106(2.0).htm
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdint>

#define PROTOCOL_VERSION 2.0

#define ADDR_MX_TORQUE_ENABLE 64
#define ADDR_MX_MOVING_SPEED 112
#define ADDR_MX_GOAL_POSITION 116

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1
#define COMM_SUCCESS 0

#define HARD_BRAKE_GOAL_POSITION 2550
#define SOFT_BRAKE_GOAL_POSITION 2200
#define DEFAULT_BRAKE_GOAL_POSITION 1024

enum class BrakeState
{
  HARDBRAKE,
  SOFTBRAKE,
  NOBRAKE
};

class DynamixelBrakeController
{
public:
  DynamixelBrakeController();

private:
  ros::NodeHandle nh;
  ros::Subscriber brake_controller_msg_sub;
  ros::Timer brake_motor_timer;

  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  int dxl_id;
  std::string device_port;
  int baud_rate;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // variables for brake timeout
  BrakeState brake_state = BrakeState::SOFTBRAKE;
  bool hard_brake_timeout = false;
  int hard_brake_count = 0;

  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);
  void brakeTimerCallback(const ros::TimerEvent& timer_event);

  bool setupDynamixelBrakeMotor();
  bool checkForPacketHandlerErrors();
  bool enableBrakeDynamixelTorque();
  bool setBrakeDynamixelSpeed();
  bool setBrakeMotorToStartPosition();
};

DynamixelBrakeController::DynamixelBrakeController()
{
  ros::NodeHandle private_nh("~");

  std::string controller_msg_topic;

  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("dxl_id", dxl_id));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));

  // Dynamixel needs to be ready before subscribing to other topics
  ROS_ASSERT_MSG(setupDynamixelBrakeMotor(), "Brake Dynamixel: Startup failure, please reset.");

  brake_controller_msg_sub =
      nh.subscribe(controller_msg_topic, 1, &DynamixelBrakeController::controllerMsgCallback, this);
  brake_motor_timer = nh.createTimer(ros::Duration(2.0), &DynamixelBrakeController::brakeTimerCallback, this);
}

void DynamixelBrakeController::brakeTimerCallback(const ros::TimerEvent& timer_event)
{
  if (hard_brake_timeout == true)
  {
    hard_brake_timeout = false;
    ROS_DEBUG("Brake Dynamixel: Hard Brake Timeout ended.");
  }

  if (brake_state == BrakeState::HARDBRAKE)
  {
    hard_brake_count++;
    if (hard_brake_count >= 2)
    {
      ROS_DEBUG("Brake Dynamixel: Hard Brake for too long! Going to Softbrake!");
      dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION,
                                                       (uint32_t)SOFT_BRAKE_GOAL_POSITION, &dxl_error);
      brake_state = BrakeState::SOFTBRAKE;
      hard_brake_timeout = true;
    }
  }
  else
  {
    hard_brake_count = 0;
  }
}

bool DynamixelBrakeController::setupDynamixelBrakeMotor()
{
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  ROS_ASSERT(port_handler->openPort());
  ROS_ASSERT(port_handler->setBaudRate(baud_rate));
  ROS_ASSERT(enableBrakeDynamixelTorque());
  ROS_ASSERT(setBrakeDynamixelSpeed());
  ROS_ASSERT(setBrakeMotorToStartPosition());

  ROS_DEBUG("Brake Dynamixel: Ready for operation.");
  return true;
}

bool DynamixelBrakeController::enableBrakeDynamixelTorque()
{
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Brake Dynamixel: Brake torque enabled.");
    return true;
  }

  return false;
}

bool DynamixelBrakeController::setBrakeDynamixelSpeed()
{
  // Sets the speed of the motor, resolution of roughly 0.002rpm
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_MOVING_SPEED, (uint32_t)768, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Brake Dynamixel: Brake speed set.");
    return true;
  }

  return false;
}

bool DynamixelBrakeController::setBrakeMotorToStartPosition()
{
  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION,
                                                   (uint32_t)SOFT_BRAKE_GOAL_POSITION, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Brake Dynamixel: Brakes set to initial position.");
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
bool DynamixelBrakeController::checkForPacketHandlerErrors()
{
  uint16_t error_code = 0;
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Brake Controller dxl_comm_result: %s", packet_handler->getTxRxResult(dxl_comm_result));
    packet_handler->read2ByteTxRx(port_handler, dxl_id, 63, &error_code, &dxl_error);
    ROS_WARN("%d", error_code);
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("Brake Controller dxl_error: %s", packet_handler->getRxPacketError(dxl_error));
    packet_handler->read2ByteTxRx(port_handler, dxl_id, 63, &error_code, &dxl_error);
    ROS_WARN("%d", error_code);
    return false;
  }

  return true;
}

void DynamixelBrakeController::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  int brake_goal_position;

  if (controller_msg->linear.z == 1.0)  // hard brake desired
  {
    if (hard_brake_timeout)
    {
      brake_goal_position = SOFT_BRAKE_GOAL_POSITION;
      brake_state = BrakeState::SOFTBRAKE;
    }
    else
    {
      brake_goal_position = HARD_BRAKE_GOAL_POSITION;
      brake_state = BrakeState::HARDBRAKE;
    }
  }
  else if (controller_msg->linear.y == 1.0)  // soft brake desired
  {
    brake_goal_position = SOFT_BRAKE_GOAL_POSITION;
    brake_state = BrakeState::SOFTBRAKE;
  }
  else
  {
    brake_goal_position = DEFAULT_BRAKE_GOAL_POSITION;
    brake_state = BrakeState::NOBRAKE;
  }

  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION,
                                                   (uint32_t)brake_goal_position, &dxl_error);
  checkForPacketHandlerErrors();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_brake_controller_node");
  DynamixelBrakeController dynamixel_brake_controller_obj;
  ros::spin();
  return 0;
}
