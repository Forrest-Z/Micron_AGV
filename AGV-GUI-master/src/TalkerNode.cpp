#include "includes/TalkerNode.h"
#include <qdebug.h>

TalkerNode::TalkerNode(){

}

void TalkerNode::initNode(){
    n = new ros::NodeHandle();
    pathfile_pub = new ros::Publisher();
    *pathfile_pub = n->advertise<std_msgs::String>("path_filename_topic",1000,true);
    joyctrl_pub = new ros::Publisher();
    *joyctrl_pub = n->advertise<sensor_msgs::Joy>("joy",1000,true);
    ros::Duration(1).sleep();
}

void TalkerNode::slot_sendPathfile(std::string a){
        std::stringstream ss;
        ss << a;
        std_msgs::String msg;
        msg.data = ss.str();
        pathfile_pub->publish(msg);
}

void TalkerNode::slot_sendJoyCommand(int command){    //0: stop; 1: start
    sensor_msgs::Joy msg;
    if (command == 1){
        msg.buttons.emplace_back(1);
        msg.buttons.emplace_back(0);
    }
    else {
        msg.buttons.emplace_back(0);
        msg.buttons.emplace_back(1);
    }
    for(int i = 0; i < 9; i++)
        msg.buttons.emplace_back(0);
    for(int i = 0; i < 8; i++)
        msg.axes.emplace_back(0.0);
    joyctrl_pub->publish(msg);
}

TalkerNode::~TalkerNode(){
    ros::shutdown();
    delete pathfile_pub;
    delete joyctrl_pub;
    delete n;
    emit finished();
}
