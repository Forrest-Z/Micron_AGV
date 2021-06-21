#ifndef TALKERTHREAD_H
#define TALKERTHREAD_H
#include <QThread>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
class TalkerNode : public QThread{
    Q_OBJECT
public:
    TalkerNode();
    ~TalkerNode();

public slots:
    void initNode();
    void slot_sendPathfile(std::string);
    void slot_sendJoyCommand(int);

signals:
    void finished();

private:
    ros::NodeHandle *n;
    ros::Publisher *pathfile_pub;
    ros::Publisher *joyctrl_pub;
};


#endif // TALKERTHREAD_H
