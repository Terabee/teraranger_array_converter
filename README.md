# ROS package for converting messages from TeraRanger array solutions by Terabee

* [Teraranger Array package](https://github.com/Terabee/teraranger_array)

## Requirements
WARNING : To be able to convert toward _laser\_scan_ and _point\_cloud_, you need to have transforms setup correctly. The easiest is to use URDF xacros defined in the __teraranger\_description__ package:
* [Teraranger\_description](https://github.com/Terabee/teraranger_description)


## Building and running the package from source
 
To clone and build the package in your workspace follow these steps:

* If you have ssh key setup for your github account:

```
cd ~/ros_ws/src
git clone git@github.com:Terabee/teraranger_array_converter.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

* If you prefer to use https use this set of commands:

```
cd ~/ros_ws/src
git clone https://github.com/Terabee/teraranger_array_converter.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## Multiple modes
There are multiple conversion modes available for the "converter_mode" parameter:

* laser_scan
* point_cloud
* individual_ranges : 8 topics, 1 range per topic
* sequential_ranges : 1 topic, 8 ranges per topic

```
rosrun teraranger_array_converter teraranger_array_converter _converter_mode:=<laser_scan>|<point_cloud>|<individual_ranges>|<sequential_ranges>
```

## Topics
All these topics are local :
* Input : /ranges
* Outputs (depending on the mode): /laser_scan, /point_cloud, /range_0|/range_1.../range_n, /ranges

## Sensor masking
You can mask sensor using the "sensor_mask" parameter :
```
<rosparam>
      sensor_mask: [true,false,true,false,true,false,true,false]
</rosparam>
```
