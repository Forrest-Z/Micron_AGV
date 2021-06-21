cmake_minimum_required(VERSION 2.8.3)
project(agv)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  pcl_ros
  pcl_conversions
  dynamixel_sdk
  dynamic_reconfigure
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EncoderCount.msg
  DataReader.msg
  LaneInfo.msg
  Waypoint.msg
  FloatList.msg
  Behaviour.msg
  Maps.msg
  HealthMonitor.msg
  ActuatorHealth.msg
  ActuatorState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

generate_dynamic_reconfigure_options(
  cfg/behaviour_planner.cfg
  cfg/local_planner.cfg
  cfg/collision_detector.cfg
  cfg/lane_publisher.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  pcl_ros
  pcl_conversions
  dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

######################################################### Build Libraries #############################################################

add_library(common
  include/common/frenet.h
  include/common/frenet.cc
  include/common/lane.h
  include/common/lane.cc
  include/common/math_utils.h
  include/common/math_utils.cc
  include/common/motion_model.h
  include/common/motion_model.cc
  include/common/obstacle.h
  include/common/obstacle.cc
  include/common/polynomials.h
  include/common/polynomials.cc
  include/common/quartic_polynomial.h
  include/common/quartic_polynomial.cc
  include/common/quintic_polynomial.h
  include/common/quintic_polynomial.cc
  include/common/spline.h
  include/common/spline.cc
  include/common/vehicle_state.h
  include/common/vehicle_state.cc
  include/common/vehicle.h
  include/common/vehicle.cc
)

add_library(behaviour_planner
  include/behaviour_planner/behaviour_planner_helper.h
  include/behaviour_planner/behaviour_planner_helper.cpp
)
target_link_libraries(behaviour_planner common)
add_dependencies(behaviour_planner common)

add_library(collision_detector
  include/collision_detector/collision_detector_helper.h
  include/collision_detector/collision_detector_helper.cpp
)
target_link_libraries(collision_detector common)
add_dependencies(collision_detector common)

add_library(local_planner
  include/local_planner/frenet_optimal_trajectory_planner.h
  include/local_planner/frenet_optimal_trajectory_planner.cc
)
target_link_libraries(local_planner common)
add_dependencies(local_planner common)

add_library(mpc
  include/mpc/MPC.h
  include/mpc/MPC.cc
)
target_link_libraries(mpc ipopt common)
add_dependencies(mpc ipopt common)

######################################################## Build Nodes ##################################################################

add_executable(encoder_odometry_system src/encoder_odometry_system.cpp)
target_link_libraries(encoder_odometry_system ${catkin_LIBRARIES})

add_executable(arduino_encoder_timer src/arduino_encoder_timer.cpp)
target_link_libraries(arduino_encoder_timer ${catkin_LIBRARIES})

add_executable(data_reader src/data_reader.cpp)
target_link_libraries(data_reader ${catkin_LIBRARIES})

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})

add_executable(dynamixel_brake_controller src/dynamixel_brake_controller.cpp)
target_link_libraries(dynamixel_brake_controller ${catkin_LIBRARIES})
add_dependencies(dynamixel_brake_controller ${catkin_EXPORTED_TARGETS})

add_executable(dynamixel_steering_controller_proplus src/dynamixel_steering_controller_proplus.cpp)
target_link_libraries(dynamixel_steering_controller_proplus ${catkin_LIBRARIES})

add_executable(stanley_steering_controller src/stanley_steering_controller.cpp)
target_link_libraries(stanley_steering_controller ${catkin_LIBRARIES})

add_executable(path_publisher src/path_publisher.cpp)
target_link_libraries(path_publisher ${catkin_LIBRARIES})

add_executable(nav_cmd_constructor src/nav_cmd_constructor.cpp)
target_link_libraries(nav_cmd_constructor ${catkin_LIBRARIES})

add_executable(rviz_baselink_visualization src/rviz_baselink_visualization.cpp)
target_link_libraries(rviz_baselink_visualization ${catkin_LIBRARIES})

add_executable(lane_publisher src/lane_publisher.cpp)
target_link_libraries(lane_publisher ${catkin_LIBRARIES} common)
add_dependencies(${catkin_EXPORTED_TARGETS} ${agv_EXPORTED_TARGETS} agv_gencfg common)

add_executable(sensor_fusion src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion ${catkin_LIBRARIES})

add_executable(deprojection src/deprojection.cpp)
target_link_libraries(deprojection ${catkin_LIBRARIES})

add_executable(behaviour_planner_node src/behaviour_planner_node.cpp)
target_link_libraries(behaviour_planner_node ${catkin_LIBRARIES} local_planner behaviour_planner)
add_dependencies(${catkin_EXPORTED_TARGETS} ${agv_EXPORTED_TARGETS} agv_gencfg local_planner behaviour_planner)

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node ${catkin_LIBRARIES} ${frenet_planner_LIBRARY} local_planner)
add_dependencies(local_planner_node ${catkin_EXPORTED_TARGETS} agv_gencfg local_planner)

add_executable(collision_detector_node src/collision_detector_node.cpp)
target_link_libraries(collision_detector_node ${catkin_LIBRARIES} collision_detector)
add_dependencies(${catkin_EXPORTED_TARGETS} ${agv_EXPORTED_TARGETS} agv_gencfg collision_detector)

add_executable(mpc_controller_node src/mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node ${catkin_LIBRARIES} ipopt mpc)
add_dependencies(mpc_controller_node ${catkin_EXPORTED_TARGETS} ipopt mpc)

add_executable(health_monitor src/health_monitor.cpp)
target_link_libraries(health_monitor ${catkin_LIBRARIES})
add_dependencies(health_monitor ${catkin_EXPORTED_TARGETS} ${agv_EXPORTED_TARGETS})