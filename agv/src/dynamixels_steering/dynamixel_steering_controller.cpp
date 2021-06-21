/*
 * For Dynamixel H54-200-S500-R, Dynamixel Protocol 2.0
 * http://emanual.robotis.com/docs/en/dxl/pro/h54-200-s500-r/
 * Refer to documentation dynamixel_steering_controller.adoc for more details
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <cstdint>

#define PROTOCOL_VERSION 2.0

#define ADDR_PRO_OPERATING_MODE 11
#define ADDR_PRO_TORQUE_ENABLE 562
#define ADDR_PRO_GOAL_POSITION 596
#define ADDR_PRO_MOVING_SPEED 600

#define EXTENDED_POSITION_CONTROL_MODE 4

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1
#define COMM_SUCCESS 0

#define STEERING_ANGLE_TO_GOAL_POSITION_CONST 1597670.53
#define STEERING_ANGLE_LIMIT 0.698132

class DynamixelSteeringController
{
public:
  DynamixelSteeringController();

private:
  ros::NodeHandle nh;
  ros::Subscriber steering_controller_msg_sub;

  dynamixel::PortHandler* port_handler;
  dynamixel::PacketHandler* packet_handler;

  double WHEEL_CENTER_OFFSET = 0;

  int dxl_id;
  std::string device_port;
  int baud_rate;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  void controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg);

  bool setupDyanmixelSteeringMotor();
  bool checkForPacketHandlerErrors();
  bool enableSteeringDynamixelTorque();
  bool setSteeringDynamixelSpeed();
  bool setSteeringMotorToStartPosition();
  double calculateSteeringAngle(double desired_steering_angle);
};

DynamixelSteeringController::DynamixelSteeringController()
{
  ros::NodeHandle private_nh("~");

  std::string controller_msg_topic;

  ROS_ASSERT(private_nh.getParam("controller_msg_topic", controller_msg_topic));
  ROS_ASSERT(private_nh.getParam("dxl_id", dxl_id));
  ROS_ASSERT(private_nh.getParam("device_port", device_port));
  ROS_ASSERT(private_nh.getParam("baud_rate", baud_rate));
  ROS_ASSERT(private_nh.getParam("wheel_center_offset", WHEEL_CENTER_OFFSET));
  ROS_DEBUG("The wheel center offset is %f radians", WHEEL_CENTER_OFFSET);

  // Dynamixel needs to be ready before subscribing to other topics
  ROS_ASSERT_MSG(setupDyanmixelSteeringMotor(), "Steering Dynamixel: Startup failure, please reset.");

  steering_controller_msg_sub =
      nh.subscribe(controller_msg_topic, 1, &DynamixelSteeringController::controllerMsgCallback, this);
}

bool DynamixelSteeringController::setupDyanmixelSteeringMotor()
{
  port_handler = dynamixel::PortHandler::getPortHandler(device_port.c_str());
  packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  ROS_ASSERT(port_handler->openPort());
  ROS_ASSERT(port_handler->setBaudRate(baud_rate));
  ROS_ASSERT(enableSteeringDynamixelTorque());
  ROS_ASSERT(setSteeringDynamixelSpeed());
  ROS_ASSERT(setSteeringMotorToStartPosition());

  ROS_DEBUG("Steering Dynamixel ready for operation.");
  return true;
}

bool DynamixelSteeringController::enableSteeringDynamixelTorque()
{
  dxl_comm_result =
      packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Steering Dynamixel: Steering torque enabled.");
    return true;
  }
  return false;
}

bool DynamixelSteeringController::setSteeringDynamixelSpeed()
{
  dxl_comm_result =
      packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_MOVING_SPEED, (uint32_t)15000, &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Steering Dynamixel: Steering speed set.");
    return true;
  }
  return false;
}

bool DynamixelSteeringController::setSteeringMotorToStartPosition()
{
  dxl_comm_result = packet_handler->write4ByteTxRx(
      port_handler, dxl_id, ADDR_PRO_GOAL_POSITION,
      (uint32_t)(WHEEL_CENTER_OFFSET * -STEERING_ANGLE_TO_GOAL_POSITION_CONST), &dxl_error);
  if (checkForPacketHandlerErrors())
  {
    ROS_DEBUG("Steering Dynamixel: Steering set to initial position.");
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
bool DynamixelSteeringController::checkForPacketHandlerErrors()
{
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Steering Dynamixel: %s", packet_handler->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("Steering Dynamixel: %s", packet_handler->getRxPacketError(dxl_error));
    return false;
  }

  return true;
}

double DynamixelSteeringController::calculateSteeringAngle(double desired_steering_angle)
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

void DynamixelSteeringController::controllerMsgCallback(const geometry_msgs::Twist::ConstPtr& controller_msg)
{
  double steering_angle = calculateSteeringAngle(controller_msg->angular.z);
  double goal_position_value = (steering_angle + WHEEL_CENTER_OFFSET) * -STEERING_ANGLE_TO_GOAL_POSITION_CONST;
  // ROS_DEBUG("Steering Dynamixel: Position value: %f", goal_position_value);

  dxl_comm_result = packet_handler->write4ByteTxRx(port_handler, dxl_id, ADDR_PRO_GOAL_POSITION,
                                                   (uint32_t)goal_position_value, &dxl_error);
  ROS_ASSERT(checkForPacketHandlerErrors());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_steering_controller_node");
  DynamixelSteeringController dynamixel_steering_controller_obj;
  ros::spin();
  return 0;
}
