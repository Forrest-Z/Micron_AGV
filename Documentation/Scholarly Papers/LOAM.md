important frames:
"camera_init" is similar to ros "odom"
"camera" is similar to ros "base_link"

important topics:
/aft_mapped_to_init : final pose estimate after some form of filtering, publish at slow rate of 0.5Hz
/integrated_to_init : fast intermediary pose estimate, publish at 10Hz

it subscribes to imu/data which can mess with reading, current fix is to change it to subsribe to unpublished topic called imu/loam

Velodyne has different axis orientation as compared to ros
Need to change source code

To make it work with IMU, use default program (axis unmodified) and use the imu with ENU config
