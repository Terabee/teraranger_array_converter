# ROS package for converting messages from TeraRanger array solutions by Terabee

|[teraranger_array package](https://github.com/Terabee/teraranger_array)|

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
There are multiple conversion modes:

```
rosrun teraranger_array_converter teraranger_array_converter _converter_mode:=<laser_scan>|<point_cloud>|<individual_ranges>|<sequential_ranges>
```
