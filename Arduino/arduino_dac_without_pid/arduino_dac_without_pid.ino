

/*
   Alexis removed the PID. Directly send the desired speed from ROS.
   Date: 12 Nov 2019
*/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#define ledPIN 6
#define acceleratorEnPin 4  // to accelerator transistor

Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, ledPIN, NEO_GRB + NEO_KHZ800);
ros::NodeHandle nh;

Adafruit_MCP4725 dac;

long currentTime = 0;
long timeForTimeOut = 0;
double brake_mode = 0;
//double turning_data = 0;
double velocity = 0;
int shift_right = 0;
int shift_left = 0;
double desired_speed = 0;
//double linearX;
//double voltageX;
//double analogX;

//long pidCurrentTime, pidPreviousTime;
//double pidElapsedTime;
//double error = 0, lastError = 0, cumError = 0, rateError = 0;

//double pidSetpoint = 0, pidOutput = 0, pidInput = 0;
//double Kp = 0.125;
//double Ki = 0.001;
//double Kd = 0;

geometry_msgs::Twist dac_msg;
ros::Publisher dac_info_pub("dac_info", &dac_msg);

void color_left_Wipe(uint32_t c) {
  for (int i = 7; i < 10; i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(1);
  }
}

void color_right_Wipe(uint32_t c) {
  for (int i = 2; i >= 0; i = i - 1) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(0);

  }
}

void color_brake_Wipe(uint32_t c) {
  for (int i = 3; i < 7; i = i + 1) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(1);
  }
}
void colorWipe(uint32_t c) {
  for (int i = 0; i < 10; i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(1);
  }
}



/*
   Input Message format:
   linear.x => desired velocity in m/s
   linear.y => 1.0 if soft brake is desired, 0 otherwise
   linear.z => 1.0 if hard brake is desired, 0 otherwise
   angular.x => 1.0 if switch closed is desired (in manual and autonomous mode), 0 otherwise
   angular.y => (UNUSED, always 0)
   angular.z => desired steering angle in radians
*/


/*
   Debug Message format:
   linear.x => unused
   linear.y => unused
   linear.z => desired steering angle in radians
   angular.x => desired speed (input) (-1 means we are braking.)
   angular.y => voltage corresponding to said desired speed;
   angular.z => digital signal to DAC based on said voltage
*/

void messageCb(const geometry_msgs::Twist& msg)
{
  /*
    if (msg.linear.z == 1)
    {
    //hard brake logic
    dac.setVoltage(0, false);
    pidSetpoint = 0;
    cumError = 0; //reset I cumulative error when hard braking, to prevent sudden jump when disengaging from hard brake
    pidOutput = computePID(pidInput);
    }
    else
    {
    pidSetpoint = msg.linear.x;
    timeForTimeOut = millis() + 1000;
    pidOutput = computePID(pidInput);
    // double linearX = pidOutput;
    double linearX = (msg.linear.x * 0.85) + pidOutput;
    double voltageX = ((0.252 * linearX) + 2.0);
    if (linearX <= 0.1)
    {
      voltageX = 0;
    }
    int analogX = (int)(voltageX / 4.85 * 4095.0);
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
  */
  timeForTimeOut = millis() + 1000;


  //if braking, do not throttle
  if (msg.linear.z != 0)
  {
    dac.setVoltage(0, false);

    dac_msg.angular.x = -1;
    dac_msg.angular.y = 0;
    dac_msg.angular.z = 0;
  }
  else
  {
    //not braking, set throttle
    double linearX = msg.linear.x; //desired speed, m/s
    desired_speed = msg.linear.x;
    double voltageX = ((0.252 * linearX) + 2.0); //speed -> voltage conversion.
    //threshold input
    if (linearX <= 0.05)
    {
      voltageX = 0;
    }

    int analogX = (int)(voltageX / 5.03 * 4095.0); //voltage -> DAC input conversion.
    if (analogX > 4095)
    {
      analogX = 4095;
    }
 
    dac_msg.angular.x = linearX;
    dac_msg.angular.y = voltageX;
    dac_msg.angular.z = analogX;

    dac.setVoltage(analogX, false);
  }

  //led stuff
  brake_mode = msg.linear.z;
  //turning_data = msg.angular.z;
  velocity = msg.linear.x;

  if (brake_mode != 0)
  {
    color_brake_Wipe(strip.Color(253, 0, 0));
  }
  else if(velocity < 0)
  {
    color_brake_Wipe(strip.Color(220,0,100));
  }
  else
  {
    color_brake_Wipe(strip.Color(0, 0, 0));
  }
  
  if (msg.angular.x == 0)
  {
    digitalWrite(acceleratorEnPin, LOW);
  }
  else
  {
    digitalWrite(acceleratorEnPin, HIGH);
  }
}

void LED_Callback(const std_msgs::Int16& predict_turning)
{
  Serial.println("enter led_callback");
  
  if (predict_turning.data == 1 && brake_mode == 0 && velocity >=0) //20*pi/180
  {
    color_left_Wipe(strip.Color(220, 0, 89));
    shift_left = 1;
    if (shift_right  == 1)
    {
      color_right_Wipe (strip.Color(0, 0, 0));
      shift_right = 0;
    }
    
  }
  else if (predict_turning.data == -1 && brake_mode == 0 && velocity >= 0) //20*pi/180 0.174
  {   
    color_right_Wipe(strip.Color(220, 0, 89));
    shift_right = 1;
    if (shift_left == 1)
    {
      color_left_Wipe (strip.Color(0, 0, 0));
      shift_left = 0;
    }
 
  }
  else
  {
    color_right_Wipe(strip.Color(0, 0, 0));
    color_left_Wipe(strip.Color(0, 0, 0));
  }
}


void speedCb(const std_msgs::Float64& speed_msg)
{
  /*
    // Speed of the vehicle is the pidInput
    pidInput = abs(speed_msg.data);
    pidOutput = computePID(pidInput);

    // to debug
    dac_msg.linear.x = pidInput;
    dac_msg.linear.y = pidOutput;
    dac_msg.linear.z = pidSetpoint;
  */

  dac_msg.linear.x = speed_msg.data;
  dac_msg.linear.y = 0;
  
  if(desired_speed > 0.1)
  {
    dac_msg.linear.z = fabs(desired_speed - speed_msg.data)/desired_speed;
  }
  else
  {
    dac_msg.linear.z = 0;
  }
  dac_info_pub.publish(&dac_msg);
}

/*
  double computePID(double input)
  {
  pidCurrentTime = millis();
  pidElapsedTime = pidCurrentTime - pidPreviousTime;

  error = pidSetpoint - input;
  cumError += error * pidElapsedTime;
  if (cumError > 500)
  {
    cumError = 500;
  }
  if (cumError < -500)
  {
    cumError = -500;
  }
  rateError = (error - lastError) / pidElapsedTime;
  double out = (Kp * error) + (Ki * cumError) + (Kd * rateError);

  lastError = error;
  pidPreviousTime = pidCurrentTime;

  if (out > 0.5)
  {
    out = 0.5;
  }
  if (out < -0.5)
  {
    out = -0.5;
  }

  return out;
  }
*/

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_out", &messageCb);
ros::Subscriber<std_msgs::Float64> speed_sub("current_speed_topic", &speedCb);
ros::Subscriber<std_msgs::Int16> led_sub("turn_signal", &LED_Callback);

void setup()
{
  // For Accelerator
  dac.begin(0x60);
  pinMode(acceleratorEnPin, OUTPUT);
  // For LED Strips
  strip.begin();
  strip.show();

  // For ROS
  nh.initNode();
  nh.advertise(dac_info_pub);
  nh.subscribe(sub);
  nh.subscribe(speed_sub);
  nh.subscribe(led_sub);
  //pidSetpoint = 0;
  //pidPreviousTime = millis();
}

int count = 0;

void loop()
{
  nh.spinOnce();
  delay(1);
  if (count == 0)
  {
    count ++;
    for (int j = 0; j < 100; j++)
    {
      for (int i = 0; i < 10; i++)
      {
        strip.setPixelColor(i, strip.Color(j + 10, j + 150, j + 50));
        delay(10);
      }
      strip.show();
      delay(40);
    }
    colorWipe(strip.Color(0, 0, 0));
  }

  // To timeout for joystick commands, does not work if vehicle is autonomous,
  // as vehicle will be publishing the callbacks at a certain rate
  currentTime = millis();
  if (currentTime >= timeForTimeOut)
  {
    dac.setVoltage(0, false);
  }
}
