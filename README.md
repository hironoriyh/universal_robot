universal_robot
======

[ROS-Industrial](http://wiki.ros.org/Industrial) universal_robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot


__Digital IO__

ur_msgs/SetIO
	int8 FUN_SET_DIGITAL_OUT=1
	int8 FUN_SET_FLAG=2
	int8 FUN_SET_ANALOG_OUT=3
	int8 FUN_SET_TOOL_VOLTAGE=4
	int8 fun
	int8 pin
	float32 state

example
```rosservice call /ur_driver/set_io "fun: 1 pin: 4 state: 1.0" ```




__Installation from Source__  
There are releases available for ROS Hydro and ROS Indigo. However, for the latest features and developments you might want to install from source.

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/universal_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.  

---

__Usage with real Hardware__  
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

``` roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=192.162.0.20```


```roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch limited:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur3_moveit_config moveit_rviz.launch config:=true```



NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_gazebo ur5.launch limited:=true```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


