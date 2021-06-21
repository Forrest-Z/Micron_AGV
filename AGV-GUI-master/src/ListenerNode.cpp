#include "includes/ListenerNode.h"
#include "includes/Settings.h"
#include <qdebug.h>

ListenerNode::ListenerNode():a(0){ }

void ListenerNode::slot_startListening(){
    Setting settings;
    const bool debugging = settings.debugging(); // activate or deactivate debug/Demo
    const double RAD2DEG = -57.3; // rad to degrees, and flip direction, left +
    const double MPS2KPH = 3.6;

    int argc = 0; char **argv = nullptr;
    ros::init(argc, argv, "UI_listener" /*, ros::init_options::AnonymousName*/);
    n = new ros::NodeHandle();

    // subscriber for speed info
    sub = new ros::Subscriber();
    *sub = n->subscribe("odometry/filtered", 1000, &Speed_msg::odometryCallback, &speed_msg);
    // subscriber for steering info
    steering_sub = new ros::Subscriber();
    *steering_sub = n->subscribe("/desired_steering_angle", 1000, &Steering_msg::SteeringCallback, &steering_msg);
    // subscriber for brake info
    brake_sub = new ros::Subscriber();
    *brake_sub = n->subscribe("/cmd_vel_out", 1000, &Brake_msg::BrakeCallback, &brake_msg);
    // subscriber for path info
    path_sub = new ros::Subscriber();
    *path_sub = n->subscribe("/current_path", 1000, &Path_msg::PathCallback, &path_msg);
    // subscriber for obstacle dist
    obstacle_sub = new ros::Subscriber();
    *obstacle_sub = n->subscribe("/closest_obstacle_distance", 1000, &Obstacle_msg::ObstacleCallback, &obstacle_msg);
    // subscriber for dac info
    dac_sub = new ros::Subscriber();
    *dac_sub = n->subscribe("/dac_info", 1000, &Accelerator_msg::AcceleratorCallback, &accelerator_msg);
    // subscriber for joy msg
    joy_sub = new ros::Subscriber();
    *joy_sub = n->subscribe("/joy", 1000, &Joy_msg::joyCallback, &joy_msg);

    ros::Rate loop_rate(10);
    while(ros::ok()){

        if(debugging){
            // send dummy speed to qml
            emit speedOdom(10*cos(a) + 10); // 0 ~ +20
            // send dummy steering angle to qml
            emit steeringAngle(20*sin(a)); // -20 ~ +20
            // send dummy speed to qml
            emit brakeState(static_cast<int>(static_cast<int>(-5*sin(a))%3)); // 0, 1, 2
            // send dummy obstacle info to display
            emit obstacleDist(10*cos(a)+ 10); // 0 ~ +20
            // send dummy dac info to qml
            emit acceleration(5*sin(a)); // 0 ~ +5
            // send dummy mode info to qml
            emit vehicleMode(static_cast<int>(static_cast<int>(5*sin(a))%4 + 5 - 1)); // -1, 0, 1, 2
            a += 0.02;
        }
        else {
            // send actual speed to qml
            emit speedOdom(speed_msg.speedx*MPS2KPH); // testing or actual data
            // send actual steering angle to qml
            emit steeringAngle(steering_msg.angle*RAD2DEG); //rad to degrees
            // send actual brake info to qml
            emit brakeState(brake_msg.brake_state);
            // send actual path info to buttons
            emit pathExist(path_msg.path_exist);
            // send actual obstacle info to display
            emit obstacleDist(obstacle_msg.distance);
            // send actual dac info to qml
            emit acceleration(accelerator_msg.acceleration);
            // send actual vehicle mode info to qml
            emit vehicleMode(joy_msg.new_state);
        }
        if(speed_msg.speedx < 0.1)
            emit vehicle_not_moving();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

ListenerNode::~ListenerNode(){
    ros::shutdown();
    delete n;
    emit finished();
}
