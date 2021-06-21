
# Creating paths (rosbag, csv)  

```bash
cd bagfiles
rosbag record /amcl_pose
```

Extract data from the rosbag and store in another file:

```
rostopic echo -b file.bag -p /topic > data.csv
```

Note the need to change "file.bag" and "/topic" and "data.csv"

Open .csv file, copy coordinate pose.pose.position.x and coordinate pose.pose.position.y into a txt file

```
rostopic pub /path_filename_topic (tab) (tab) '(enter path name here)'"
```

