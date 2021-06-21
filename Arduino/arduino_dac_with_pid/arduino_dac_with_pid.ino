

/*
 * Combined Arduino Code
 * Date: 27 June 2019
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#define ledPIN 6
#define acceleratorEnPin 4  // to accelerator transistor

Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, ledPIN, NEO_GRB + NEO_KHZ800);
ros::NodeHandle nh;

Encoder knob(18, 19);
std_msgs::Int64 encoder_count_msg;
ros::Publisher encoder_count_pub("raw_encoder_count_left", &encoder_count_msg);
long encoderCount = 0;

Adafruit_MCP4725 dac;

long currentTime = 0;
long timeForTimeOut = 0;

long pidCurrentTime, pidPreviousTime;
double pidElapsedTime;
double error = 0, lastError = 0, cumError = 0, rateError = 0;

double pidSetpoint = 0, pidOutput = 0, pidInput = 0;
double Kp = 0.125;
double Ki = 0.001;
double Kd = 0;

geometry_msgs::Twist dac_msg;
ros::Publisher dac_info_pub("dac_info", &dac_msg);

// encoder publish function
void Publish(){
  encoder_count_msg.data = encoderCount;
  encoder_count_pub.publish ( &encoder_count_msg );
}

void encoderTimerCallback(const std_msgs::Bool& msg) {
  //  encoderCount = knob.read();
  //  Publish();
}

void color_right_Wipe(uint32_t c) {
  for(int i=7; i<10; i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(1);
  }
}

void color_left_Wipe(uint32_t c) {
  for(int i=2; i>=0; i=i-1) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(0);
   
  }
}

void color_brake_Wipe(uint32_t c) {
  for(int i=3; i<7; i=i+1) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(1);
  }
}
void colorWipe(uint32_t c) {
  for(int i=0; i<10; i++) 
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(1);
  }
}



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
  if (msg.linear.z == 1)
  {
    // Hard brake logic
    dac.setVoltage(0, false);
    pidSetpoint = 0;
    cumError = 0; //reset I cumulative error when hard braking, to prevent sudden jump when disengaging from hard brake
    pidOutput = computePID(pidInput);
  }
  else
  {
    // No brake logic
    pidSetpoint = msg.linear.x;
    timeForTimeOut = millis() + 1000;
    pidOutput = computePID(pidInput);
    // double linearX = pidOutput;
    double linearX = (msg.linear.x * 0.85) + pidOutput;
    double voltageX = ((0.246 * linearX) + 2.03);
    if (linearX <= 0.1)
    {
      voltageX = 0;
    }
    int analogX = (int)(voltageX / 5.03 * 4095.0);
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
  
  if(msg.linear.z == 0)
  {
    color_brake_Wipe(strip.Color(0, 0, 0));
  }
  else
  {
    color_brake_Wipe(strip.Color(253, 0, 0));
  }

  if(msg.angular.z > 0.1744 && msg.linear.z == 0) //20*pi/180
  {

    color_right_Wipe(strip.Color(220, 0, 89));
    
  }
  else if(msg.angular.z < -0.1744 && msg.linear.x == 0) //20*pi/180
  {
    color_left_Wipe(strip.Color(220, 0, 89));
  
  }
  else{
    color_right_Wipe(strip.Color(0, 0, 0));
    color_left_Wipe(strip.Color(0, 0, 0));
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

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_out", &messageCb);
ros::Subscriber<std_msgs::Float64> speed_sub("current_speed_topic", &speedCb);
//ros::Subscriber<geometry_msgs::Twist> nav_sub("nav_cmd_vel", &ledCb);
//ros::Subscriber<std_msgs::Bool> encoder_timer_sub("encoder_timer", &encoderTimerCallback);

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
  //nh.subscribe(nav_sub);
  //  nh.advertise(encoder_count_pub);
  //  nh.subscribe(encoder_timer_sub);

  pidSetpoint = 0;
  pidPreviousTime = millis();
}
int count = 0;
void loop()
{
  nh.spinOnce();
  delay(1);
  if(count == 0) 
  {
    count ++;
    for(int j=0; j<100; j++) 
    {
    for(int i=0; i<10; i++)
     {
      strip.setPixelColor(i, strip.Color(j+150, j+150, j+50));
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
