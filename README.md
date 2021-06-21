# f10agv

## Installation
1. Install ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu  
2. Create catkin_ws and src folder and cd to it: `mkdir -p catkin_ws/src && cd ~/catkin_ws/src`  
3. Clone this repository into catkin_ws/src  
4. Copy the obstacle detector and imu package into this directory
5. Return to the root folder of catkin_ws `cd ~/catkin_ws`  
6. Install dependencies: `rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y`  
7. Install Armadilo C++ needed for obstacle detector library `sudo apt install libarmadillo*`  
8. Install rosserial-arduino package: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup  
9. `catkin_make && source devel/setup.bash`  
10. Update udev rules and other configuration settings (refer to Documentation folder for more details)   


## Launch Files
There are 3 different launch files:
1. unit1/2/3.launch, which provides the ability for the robot to be driven
2. agvnavigation.launch, which provides autonomous capability for the robot
3. mavis1/2/3.launch, which launches both the above launch files in one go

To activate, roslaunch agv _____.launch  
e.g. roslaunch agv mavis1.launch
