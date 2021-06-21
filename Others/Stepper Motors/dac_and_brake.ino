/*
 * Arduino Mega1 code, for Accelerator and Brake, using 3 Arduinos
 * Date: 9 May 2019
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define BRAKE_MOTOR_PULSE_HIGH 4
#define BRAKE_MOTOR_PULSE_LOW 5
#define BRAKE_MOTOR_DIRECTION_HIGH 6
#define BRAKE_MOTOR_DIRECTION_LOW 7

#define acceleratorEnPin 8 //to accelerator transistor

#define BrakeStateLowerThreshold 4

ros::NodeHandle  nh;

Adafruit_MCP4725 dac;

int numPulsesForFullBrake = 5000;

int brakeState = 0;

long currentTime = 0;
long timeForTimeOut = 0;

void sendPulses(int pulses) {
  for(int i = 0 ; i < pulses ; i++) {
    delayMicroseconds(16);
    digitalWrite(BRAKE_MOTOR_PULSE_HIGH, HIGH);
    digitalWrite(BRAKE_MOTOR_PULSE_LOW, LOW);
    delayMicroseconds(16);
    digitalWrite(BRAKE_MOTOR_PULSE_HIGH, LOW);
    digitalWrite(BRAKE_MOTOR_PULSE_LOW, HIGH);
  }
}

void moveBrakeUp(int pulsesToSend) {
  digitalWrite(BRAKE_MOTOR_DIRECTION_HIGH, HIGH);
  digitalWrite(BRAKE_MOTOR_DIRECTION_LOW, LOW);
  sendPulses(pulsesToSend);
}

void moveBrakeDown(int pulsesToSend) {
  digitalWrite(BRAKE_MOTOR_DIRECTION_HIGH, LOW);
  digitalWrite(BRAKE_MOTOR_DIRECTION_LOW, HIGH);
  sendPulses(pulsesToSend);
}

/*
 * msg meaning:
 * linear.z == 1, hard brake, and turn acceleration to 0
 * linear.y == 1, soft brake (halfway), and turn accleration to 0
 * linear.x = amt to send to DAC
 * only acclerate if both linear.x and linear.y == 0
 */
void messageCb(const geometry_msgs::Twist& msg) {  
  if (msg.linear.z == 1) {
    if (brakeState == 0) {
      moveBrakeDown(5000);
      brakeState = 2;
    } else if (brakeState == 1) {
      moveBrakeDown(2000);
      brakeState = 2;
    }
    dac.setVoltage(0, false);
    // digitalWrite(acceleratorEnPin, HIGH);

  } else if (msg.linear.y == 1) {
    if(brakeState == 0) {
      moveBrakeDown(3000);
      brakeState = 1;
    } else if (brakeState == 2) {
      moveBrakeUp(2000);
      brakeState = 1;
    }
    dac.setVoltage(0, false);
    // digitalWrite(acceleratorEnPin, HIGH);

  } else {
    if(brakeState == 1) {
      moveBrakeUp(3000);
      brakeState = 0;
    } else if (brakeState == 2) {
      moveBrakeUp(5000);
      brakeState = 0;
    }
    timeForTimeOut = millis() + 1000;
    double linearX = msg.linear.x;
    double voltageX = ((0.252 * linearX) + 2.0);
    if (linearX <= 0.1 ) {
      voltageX = 0;
    }
    int analogX = (int) (voltageX / 3.3 * 4095.0);
    dac.setVoltage(analogX, false);
  }

    
  if (msg.angular.x == 0) {
    digitalWrite(acceleratorEnPin, HIGH);
  } else {
    digitalWrite(acceleratorEnPin, LOW);
  }

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_out", &messageCb);

void setup() {
  //For Brake
  pinMode(BRAKE_MOTOR_PULSE_HIGH, OUTPUT);
  pinMode(BRAKE_MOTOR_PULSE_LOW, OUTPUT);
  pinMode(BRAKE_MOTOR_DIRECTION_HIGH, OUTPUT);
  pinMode(BRAKE_MOTOR_DIRECTION_LOW, OUTPUT);  

  //For Accelerator
  dac.begin(0x60);
  pinMode(acceleratorEnPin, OUTPUT);

  //For ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);

  // To timeout for joystick commands, does not work if vehicle is autonomous,
  // as vehicle will be publishing the callbacks at a certain rate
  currentTime = millis();
  if (currentTime >= timeForTimeOut) {
    dac.setVoltage(0, false);
  }
}