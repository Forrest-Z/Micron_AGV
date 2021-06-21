/*
 * Arduino Mega2 code, for Steering Motor, using 3 Arduinos
 * Date: 28 March 2019
 */
// 40 000 dip switch
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define STEERING_MOTOR_PULSE_HIGH 4
#define STEERING_MOTOR_PULSE_LOW 5
#define STEERING_MOTOR_DIRECTION_HIGH 6
#define STEERING_MOTOR_DIRECTION_LOW 7

ros::NodeHandle  nh;

int numPulsesFor1Turn = 8000;

int currentState = 0;

void turnOnce() {
  for(int i = 0 ; i < numPulsesFor1Turn ; i ++) {
    delayMicroseconds(8);
    digitalWrite(STEERING_MOTOR_PULSE_HIGH, HIGH);
    digitalWrite(STEERING_MOTOR_PULSE_LOW, LOW);
    delayMicroseconds(8);
    digitalWrite(STEERING_MOTOR_PULSE_HIGH, LOW);
    digitalWrite(STEERING_MOTOR_PULSE_LOW, HIGH);
  }
}

void turnLeft() {
  if(currentState <= -10) { //-4
    return;
  } else {
    digitalWrite(STEERING_MOTOR_DIRECTION_HIGH, LOW);
    digitalWrite(STEERING_MOTOR_DIRECTION_LOW, HIGH);
    turnOnce();
    currentState--;
  }

}

void turnRight() {
  if(currentState >= 10) { //4
    return;
  } else {
    digitalWrite(STEERING_MOTOR_DIRECTION_HIGH, HIGH);
    digitalWrite(STEERING_MOTOR_DIRECTION_LOW, LOW);
    turnOnce();
    currentState++;
  }

}

void messageCb( const geometry_msgs::Twist& msg) {
  double button = msg.angular.z;
  if (button < 0 ) {
    turnLeft();
  } else if (button > 0) {
    turnRight();
  } else {
    return;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("arduino_messages", &messageCb);

void setup() {
  //For Steering
  pinMode(STEERING_MOTOR_PULSE_HIGH, OUTPUT);
  pinMode(STEERING_MOTOR_PULSE_LOW, OUTPUT);
  pinMode(STEERING_MOTOR_DIRECTION_HIGH, OUTPUT);
  pinMode(STEERING_MOTOR_DIRECTION_LOW, OUTPUT);

  //For ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
