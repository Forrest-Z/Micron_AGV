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

class DynamixelBrakeController
{
public:
  DynamixelBrakeController();

private:
  ros::NodeHandle nh;
  ros::Subscriber brake_controller_msg_sub;

  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  int dxl_id;
  std::string device_port;
  int baud_rate;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  // TODO: change this BRAKE_ANGLE_LIMIT to the appropriate value
  double BRAKE_ANGLE_LIMIT = 1.5707963268;  // 90 deg in radians

  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);

  bool setupDynamixelBrakeMotor();
  bool checkForPacketHandlerErrors();
  bool enableBrakeDynamixelTorque();
  bool setBrakeDynamixelSpeed();
  bool setBrakeMotorToStartPosition();
};

DynamixelBrakeController::DynamixelBrakeController()
{
  ros::NodeHandle private_nh("~");
  ros::Duration(3).sleep();

  std::string controller_msg_topic;

  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("dxl_id", dxl_id));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));

  // Dynamixel needs to be ready before subscribing to other topics
  ROS_ASSERT_MSG(setupDynamixelBrakeMotor(), "Brake Dynamixel: Startup failure, please reset.");

  brake_controller_msg_sub =
      nh.subscribe(controller_msg_topic, 1, &DynamixelBrakeController::controllerMsgCallback, this);
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

  ROS_INFO("Brake Dynamixel: Ready for operation.");
  return true;
}

bool DynamixelBrakeController::enableBrakeDynamixelTorque()
{
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Brake Dynamixel: Brake torque enabled.");
    return true;
  }

  return false;
}

bool DynamixelBrakeController::setBrakeDynamixelSpeed()
{
  // TODO: Set the speed, resolution 0.002rpm
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_MOVING_SPEED, (uint32_t)768, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Brake Dynamixel: Brake speed set.");
    return true;
  }

  return false;
}

bool DynamixelBrakeController::setBrakeMotorToStartPosition()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION, (uint32_t)2200, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Brake Dynamixel: Brakes set to initial position.");
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
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s", packet_handler->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}



void DynamixelBrakeController::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  int brake_goal_position;

  if (controller_msg->linear.z == 1.0)
  {
    // 90 deg
    brake_goal_position = 2200;
  }
  else if (controller_msg->linear.y == 1.0)
  {
    // 45 deg
    brake_goal_position = 1800;
  }
  else
  {
    brake_goal_position = 1024;
  }

  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION,
                                                   (uint32_t)brake_goal_position, &dxl_error);
  // checkForPacketHandlerErrors();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_brake_controller_node");
  DynamixelBrakeController dynamixel_brake_controller_obj;
  ros::spin();
  return 0;
}
