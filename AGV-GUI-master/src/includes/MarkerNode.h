#ifndef MARKERNODE_H
#define MARKERNODE_H
#include <QThread>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"

class Pose_{
public:
    geometry_msgs::Pose pose_;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL){
        pose_.position.x = msgAMCL->pose.pose.position.x;
        pose_.position.y = msgAMCL->pose.pose.position.y;
        pose_.position.z = msgAMCL->pose.pose.position.z;
        pose_.orientation.w = msgAMCL->pose.pose.orientation.w;
        pose_.orientation.x = msgAMCL->pose.pose.orientation.x;
        pose_.orientation.y = msgAMCL->pose.pose.orientation.y;
        pose_.orientation.z = msgAMCL->pose.pose.orientation.z;
    }
};

class MarkerNode : public QThread{
    Q_OBJECT
public:
    MarkerNode();
    ~MarkerNode();

public slots:
    void marker_display();

signals:
    void currentPos(float, float);
    void finished();

private:
    ros::NodeHandle *n;
    ros::Subscriber *pose_sub;
    ros::Publisher *marker_pub;
    Pose_ amcl_pose;
};




#endif // MARKERNODE_H
