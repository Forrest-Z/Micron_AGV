# HOW TO ROSBAG GMAPPING

### For GMapping you should to delete everything from line 326 in the unitX.launch file. This is to remove the localisation node.

## Step1: type into the terminal the following stuff, then drive around the area that you wanna map.
```bash
roslaunch agv unitXXX.launch
cd bagfiles
rosbag record -a
```

## Step2: if using different computer, check rosbag with  

```bash
rosbag info <mybag>.bag  
```

if it says need to reindex then run: 

```bash
rosbag reindex <mybag>.bag  
```



## Step3: We have the lidar and odometry information in the bag file. Now we can do gmapping by playing back this information, and letting the gmapping Ros node do its job:

```bash
roscore  
rosrun gmapping slam_gmapping scan:=velodyne_laserscan _linearUpdate:=0.1 _angularUpdate:=0.1 _map_update_interval:=0.5 _particles:=200  
rosbag play -r 0.1 mybag.bag  
rviz  
```

After the mapping is done

```bash
rosrun map_server map_saver -f mymap  
```

