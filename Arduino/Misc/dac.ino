/*
 * Arduino Mega code for DAC
 * Date: 12 June 2019
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define acceleratorEnPin 8  // to accelerator transistor

ros::NodeHandle nh;

Adafruit_MCP4725 dac;

long currentTime = 0;
long timeForTimeOut = 0;

long pidCurrentTime, pidPreviousTime;
double pidElapsedTime;
double error = 0, lastError = 0, cumError = 0, rateError = 0;

double pidSetpoint, pidOutput, pidInput;
double Kp = 0.5;
double Ki = 0.001;
double Kd = 0;

geometry_msgs::Twist dac_msg;
ros::Publisher dac_info_pub("dac_info", &dac_msg);

/*
 * Message format:
 * linear.x => desired velocity in m/s
 * linear.y => 1.0 if soft brake is desired, 0 otherwise
 * linear.z => 1.0 if hard brake is desired, 0 otherwise
 * angular.x => 1.0 if switch closed is desired (in manual and autonomous mode), 0 otherwise
 * angular.y => (UNUSED, always 0)
 * angular.z => desired steering angle in radians
 */
void messageCb(const geometry_msgs::Twist& msg)
{
  pidSetpoint = msg.linear.x;

  if (msg.linear.z == 1)
  {
    // Hard brake logic
    dac.setVoltage(0, false);
  }
  else
  {
    // No brake logic
    timeForTimeOut = millis() + 1000;
    double linearX = pidOutput;
    double voltageX = ((0.252 * linearX) + 2.0);
    if (linearX <= 0.1)
    {
      voltageX = 0;
    }
    int analogX = (int)(voltageX / 4.5 * 4095.0);
    if (analogX > 4095)
    {
      analogX = 4095;
    }
    // to debug
    dac_msg.angular.x = linearX;
    dac_msg.angular.y = voltageX;
    dac_msg.angular.z = analogX;

    dac.setVoltage(analogX, false);
  }

  if (msg.angular.x == 0)
  {
    digitalWrite(acceleratorEnPin, HIGH);
  }
  else
  {
    digitalWrite(acceleratorEnPin, LOW);
  }
}

void speedCb(const std_msgs::Float64& speed_msg)
{
  // Speed of the vehicle is the pidInput
  pidInput = abs(speed_msg.data);
  pidOutput = computePID(pidInput);

  // to debug
  dac_msg.linear.x = pidInput;
  dac_msg.linear.y = pidOutput;
  dac_msg.linear.z = pidSetpoint;

  dac_info_pub.publish(&dac_msg);
}

double computePID(double input)
{
  pidCurrentTime = millis();
  pidElapsedTime = pidCurrentTime - pidPreviousTime;

  error = pidSetpoint - input;
  cumError += error * pidElapsedTime - 10;
  if (cumError > 4000)
  {
    cumError = 4000;
  }
  if (cumError < 0)
  {
    cumError = 0;
  }
  rateError = (error - lastError) / pidElapsedTime;
  double out = (Kp * error) + (Ki * cumError) + (Kd * rateError);

  lastError = error;
  pidPreviousTime = pidCurrentTime;

  if (out > 3.5)
  {
    out = 3.5;
  }
  if (out < 0)
  {
    out = 0;
  }

  return out;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_out", &messageCb);
ros::Subscriber<std_msgs::Float64> speed_sub("current_speed_topic", &speedCb);

void setup()
{
  // For Accelerator
  dac.begin(0x60);
  pinMode(acceleratorEnPin, OUTPUT);

  // For ROS
  nh.initNode();
  nh.advertise(dac_info_pub);
  nh.subscribe(sub);
  nh.subscribe(speed_sub);

  pidSetpoint = 0;
  pidPreviousTime = millis();
}

void loop()
{
  nh.spinOnce();
  delay(1);

  // To timeout for joystick commands, does not work if vehicle is autonomous,
  // as vehicle will be publishing the callbacks at a certain rate
  currentTime = millis();
  if (currentTime >= timeForTimeOut)
  {
    dac.setVoltage(0, false);
  }
}
