# Rosbag Record Topics / Path

= Storing Data (rosbag, csv)

Last updated 10 May 2019  

## Record Topic

. Store data in a rosbag: http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data  

. Extract data from the rosbag and store in another file: 

```bash
rostopic echo -b <mybag>.bag -p /topic > data.csv
```


Note the need to change "mybag.bag" and "/topic" and "data.csv"  



>  References
>
> Source: https://answers.ros.org/question/9102/how-to-extract-data-from-bag/?answer=13122#post-id-13122  

