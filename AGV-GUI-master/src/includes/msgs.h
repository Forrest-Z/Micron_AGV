#include <QDebug>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Joy.h"

class Joy_msg{
public:
    int buttonA = 0; // autonomous
    int buttonB = 0; // hard brake
    int buttonX = 0; // manual
    int new_state; // 0 for brake mode, 1 for autonomous mode, 2 for manual mode, -1 for faults

    void joyCallback(const sensor_msgs::Joy::ConstPtr & joy){
        buttonA = joy->buttons[0];
        buttonB = joy->buttons[1];
        buttonX = joy->buttons[2];
        if(buttonA == 1) {
            new_state = 1; // enter auto mode
            qDebug() << "auto mode *******";
        }
        else if (buttonB == 1) {
            new_state = 0; // enter brake mode
            qDebug() << "brake mode *******";
        }
        else if (buttonX == 1){
            new_state = 2; // enter manual mode
            qDebug() << "manual mode *******";
        }
        else {
            new_state = -1; // faulty
            //qDebug() << "ignore mode *******";
        }
    }
};

class Speed_msg{
public:
    double speedx;
    Speed_msg(): speedx(0){}
    void odometryCallback(const nav_msgs::Odometry::ConstPtr & odometry_filtered){
        speedx = odometry_filtered->twist.twist.linear.x;
    }
};

class Steering_msg{
public:
    double angle;
    Steering_msg(): angle(0){}
    void SteeringCallback(const std_msgs::Float64 & desired_steering_angle){
        angle = desired_steering_angle.data;
    }
};

class Brake_msg{
public:
    int brake_state;
    Brake_msg(): brake_state(0){}
    void BrakeCallback(const geometry_msgs::Twist & cmd_vel_out){
        if(cmd_vel_out.linear.y == 1.0){
            brake_state = 1; //soft brake
        }
        else if (cmd_vel_out.linear.z == 1.0) {
            brake_state = 2; //hard brake
        }
        else {
            brake_state = 0; //no brake
        }
    }
};

class Path_msg{
public:
    bool path_exist;
    nav_msgs::Path path;
    Path_msg(): path_exist(false){}
    void PathCallback(const nav_msgs::Path & current_path){
        path = current_path;
        if(path.poses.size() <= 0){
            path_exist = false;
            qDebug() << path.poses.size();
        }
        else {
            path_exist = true;
        }
    }
};

class Obstacle_msg{
public:
    double distance;
    Obstacle_msg(): distance(100.0){}
    void ObstacleCallback(const std_msgs::Float64 & closest_obstacle_dist){
        distance = closest_obstacle_dist.data;
    }
};

class Accelerator_msg{
public:
    double acceleration;
    Accelerator_msg(): acceleration(0.0){}
    void AcceleratorCallback(const geometry_msgs::Twist::ConstPtr & dac_info){
        acceleration = (dac_info->linear.y + 0.85*dac_info->linear.z);
    }
};
