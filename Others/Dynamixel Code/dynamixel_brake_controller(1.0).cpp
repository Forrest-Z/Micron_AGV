/*
 * For Dynamixel MX-106, Dynamixel Protocol 1.0
 * http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106.htm
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdint>

#define PROTOCOL_VERSION 1.0
#define ADDR_MX_TORQUE_ENABLE 24  // 1 Byte  RW
#define ADDR_MX_GOAL_POSITION 30  // 2 Bytes RW
#define ADDR_MX_MOVING_SPEED 32   // 2 Bytes RW
#define TORQUE_ENABLE 1
#define BRAKE_LIMIT 4095
#define COMM_SUCCESS 0

class DynamixelBrakeController
{
public:
  DynamixelBrakeController();

private:
  ros::NodeHandle nh;
  ros::Subscriber controller_msg_sub;

  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  int dxl_id;
  std::string device_port;
  int baud_rate;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  uint8_t dxl_error = 0;               // Dynamixel error

  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);
  bool setupDynamixelBrakeMotor();
};

DynamixelBrakeController::DynamixelBrakeController()
{
  ros::NodeHandle private_nh("~");

  // Create the string object
  std::string controller_msg_topic;

  // Get the param from launch file
  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("dxl_id", dxl_id));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));

  // Dynamixel needs to be ready before subscribing to other topics
  ROS_ASSERT_MSG(setupDynamixelBrakeMotor(), "Brake Dynamixel: Startup failure, please reset.\n");

  // Subscribe or Advertise
  controller_msg_sub = nh.subscribe(controller_msg_topic, 1, &DynamixelBrakeController::controllerMsgCallback, this);
}

bool DynamixelBrakeController::setupDynamixelBrakeMotor()
{
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (port_handler->openPort())
  {
    ROS_INFO("Brake Dynamixel: Succeeded in opening port.\n");
  }
  else
  {
    ROS_ERROR("Brake Dynamixel: Failed in opening port!!!\n");
    return false;
  }

  // Set baud rate
  if (port_handler->setBaudRate(baud_rate))
  {
    ROS_INFO("Brake Dynamixel: Succeeded in configuring baudrate.\n");
  }
  else
  {
    ROS_ERROR("Brake Dynamixel: Failed in configuring baudrate!!!\n");
    return false;
  }

  // Enable Dynamixel Torque
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packet_handler->getRxPacketError(dxl_error));
    return false;
  }
  else
  {
    ROS_INFO("Brake Dynamixel: Brake torque enabled. \n");
  }

  // Set Dynamixel Speed
  // TODO: Set the RPM (0 to 1023), in steps of 0.114rpm
  dxl_comm_result =
      packet_handler->write2ByteTxRx(port_handler, dxl_id, ADDR_MX_MOVING_SPEED, (uint16_t)200, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packet_handler->getRxPacketError(dxl_error));
    return false;
  }
  else
  {
    ROS_INFO("Brake Dynamixel: Brake speed set. \n");
  }

  // Set Motor to Start Position
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION,
                                                   (uint16_t)(BRAKE_LIMIT / 2), &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packet_handler->getRxPacketError(dxl_error));
    return false;
  }
  else
  {
    ROS_INFO("Brake Dynamixel: Brakes set to initial position. \n");
  }

  // For testing, read the model number
  uint16_t model_number = 0;

  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler, dxl_id, 0, &model_number, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packet_handler->getRxPacketError(dxl_error));
    return false;
  }
  else
  {
    ROS_INFO("Brake Dynamixel: The model number is %d. \n", model_number);
  }

  return true;
}

// Brake is 0 to 4095 in steps of 0.088 degrees
void DynamixelBrakeController::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  // double brake_value = controller_msg->linear.z;
  double brake_value = controller_msg->linear.x / 2.0;  // for testing
  dxl_comm_result =
      packet_handler->write2ByteTxRx(port_handler, dxl_id, ADDR_MX_GOAL_POSITION, (uint16_t)brake_value, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s\n", packet_handler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packet_handler->getRxPacketError(dxl_error));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_brake_controller_node");
  DynamixelBrakeController dynamixel_brake_controller_obj;
  ros::spin();
  return 0;
}
