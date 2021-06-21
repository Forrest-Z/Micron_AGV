#ifndef LISTENERNODE_H
#define LISTENERNODE_H
#include <QThread>
#include <QDebug>
#include <string>
#include "ros/ros.h"
#include "msgs.h"

class ListenerNode : public QThread{
    Q_OBJECT
public:
    ListenerNode();
    ~ListenerNode();

public slots:
    void slot_startListening();

signals:
    void finished();
    void stopped();
    void vehicle_not_moving();
    void speedOdom(double);
    void steeringAngle(double);
    void brakeState(int);
    void pathExist(bool);
    void obstacleDist(double);
    void acceleration(double);
    void vehicleMode(int);

private:
    ros::NodeHandle *n;

    ros::Subscriber *sub;
    ros::Subscriber *steering_sub;
    ros::Subscriber *brake_sub;
    ros::Subscriber *path_sub;
    ros::Subscriber *obstacle_sub;
    ros::Subscriber *dac_sub;
    ros::Subscriber *joy_sub;

    Speed_msg speed_msg;
    Steering_msg steering_msg;
    Brake_msg brake_msg;
    Path_msg path_msg;
    Obstacle_msg obstacle_msg;
    Accelerator_msg accelerator_msg;
    Joy_msg joy_msg;

    double a; //for debug
};

#endif // LISTENERNODE_H
