/*
 * Untested!!!!!
 * For MX106R and H54 (non-pro)
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

#define DXL_ID_STEERING 1
#define DXL_ID_BRAKE 2

#define ADDR_PRO_OPERATING_MODE 11
#define ADDR_PRO_TORQUE_ENABLE 562
#define ADDR_PRO_GOAL_POSITION 596
#define ADDR_PRO_MOVING_SPEED 600

#define EXTENDED_POSITION_CONTROL_MODE 4

#define STEERING_ANGLE_TO_GOAL_POSITION_CONST 1597670.53
#define WHEEL_CENTER_OFFSET 0.12476638
#define STEERING_ANGLE_LIMIT 0.698132

class DynamixelDaisyChain
{
public:
  DynamixelDaisyChain();

private:
  ros::NodeHandle nh;
  ros::Subscriber dynamixel_controller_msg_sub;

  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  std::string device_port;
  int baud_rate;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  bool setupDynamixelBrakeMotor();
  bool enableBrakeDynamixelTorque();
  bool setBrakeDynamixelSpeed();
  bool setBrakeMotorToStartPosition();

  bool setupDynamixelSteeringMotor();
  bool enableSteeringDynamixelTorque();
  bool setSteeringDynamixelSpeed();
  bool setSteeringMotorToStartPosition();

  bool setupPortAndPacketHandlers();
  bool checkForPacketHandlerErrors();

  double calculateSteeringAngle(double desired_steering_angle);
  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);
};

DynamixelDaisyChain::DynamixelDaisyChain()
{
  ros::NodeHandle private_nh("~");

  std::string controller_msg_topic;

  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));

  // Dynamixel needs to be ready before subscribing to other topics
  ROS_ASSERT_MSG(setupPortAndPacketHandlers(), "Port/Packet handler setup error");
  ROS_ASSERT_MSG(setupDynamixelBrakeMotor(), "Brake Dynamixel: Startup failure, please reset.");
  ROS_ASSERT_MSG(setupDynamixelSteeringMotor(), "Steering Dynamixel: Startup failure, please reset.");

  dynamixel_controller_msg_sub =
      nh.subscribe(controller_msg_topic, 1, &DynamixelDaisyChain::controllerMsgCallback, this);
}

bool DynamixelDaisyChain::setupPortAndPacketHandlers()
{
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  ROS_ASSERT(port_handler->openPort());
  ROS_ASSERT(port_handler->setBaudRate(baud_rate));
  return true;
}

bool DynamixelDaisyChain::setupDynamixelBrakeMotor()
{
  ROS_ASSERT(enableBrakeDynamixelTorque());
  ROS_ASSERT(setBrakeDynamixelSpeed());
  ROS_ASSERT(setBrakeMotorToStartPosition());

  ROS_INFO("Brake Dynamixel: Ready for operation.");
  return true;
}

bool DynamixelDaisyChain::enableBrakeDynamixelTorque()
{
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, DXL_ID_BRAKE, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Brake Dynamixel: Brake torque enabled.");
    return true;
  }

  return false;
}

bool DynamixelDaisyChain::setBrakeDynamixelSpeed()
{
  // TODO: Set the speed, resolution 0.002rpm
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, DXL_ID_BRAKE, ADDR_MX_MOVING_SPEED, (uint32_t)768, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Brake Dynamixel: Brake speed set.");
    return true;
  }

  return false;
}

bool DynamixelDaisyChain::setBrakeMotorToStartPosition()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, DXL_ID_BRAKE, ADDR_MX_GOAL_POSITION, (uint32_t)2200, &dxl_error);

  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Brake Dynamixel: Brakes set to initial position.");
    return true;
  }

  return false;
}

bool DynamixelDaisyChain::setupDynamixelSteeringMotor()
{
  ROS_ASSERT(enableSteeringDynamixelTorque());
  ROS_ASSERT(setSteeringDynamixelSpeed());
  ROS_ASSERT(setSteeringMotorToStartPosition());

  ROS_INFO("Steering Dynamixel ready for operation.");
  return true;
}

bool DynamixelDaisyChain::enableSteeringDynamixelTorque()
{
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, DXL_ID_STEERING, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Steering Dynamixel: Steering torque enabled.");
    return true;
  }
  return false;
}

bool DynamixelDaisyChain::setSteeringDynamixelSpeed()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, DXL_ID_STEERING, ADDR_PRO_MOVING_SPEED, (uint32_t)17000, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Steering Dynamixel: Steering speed set.");
    return true;
  }
  return false;
}

bool DynamixelDaisyChain::setSteeringMotorToStartPosition()
{
  dxl_comm_result = packet_handler->write4ByteTxRx(
      port_handler, DXL_ID_STEERING, ADDR_PRO_GOAL_POSITION,
      (uint32_t)(WHEEL_CENTER_OFFSET * -STEERING_ANGLE_TO_GOAL_POSITION_CONST), &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_INFO("Steering Dynamixel: Steering set to initial position.");
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
bool DynamixelDaisyChain::checkForPacketHandlerErrors()
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

double DynamixelDaisyChain::calculateSteeringAngle(double desired_steering_angle)
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

void DynamixelDaisyChain::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  int brake_goal_position;

  if (controller_msg->linear.z == 1.0)
  {
    // Hard brake
    brake_goal_position = 2200;
  }
  else if (controller_msg->linear.y == 1.0)
  {
    // Soft brake
    brake_goal_position = 1800;
  }
  else
  {
    // No brake
    brake_goal_position = 1024;
  }

  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, DXL_ID_BRAKE, ADDR_MX_GOAL_POSITION,
                                                   (uint32_t)brake_goal_position, &dxl_error);
  checkForPacketHandlerErrors();
  ROS_INFO("Daisy Chain: Sent Brake Command");

  double steering_angle = calculateSteeringAngle(controller_msg->angular.z);
  double goal_position_value = (steering_angle + WHEEL_CENTER_OFFSET) * -STEERING_ANGLE_TO_GOAL_POSITION_CONST;
  // ROS_INFO("Steering Dynamixel: Position value: %f", goal_position_value);

  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, DXL_ID_STEERING, ADDR_PRO_GOAL_POSITION,
                                                   (uint32_t)goal_position_value, &dxl_error);
  checkForPacketHandlerErrors();
  ROS_INFO("Daisy Chain: Sent Steering Command");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_daisy_chain_node");
  DynamixelDaisyChain dynamixel_daisy_chain_obj;
  ros::spin();
  return 0;
}
