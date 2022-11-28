# ROS Workshop - Tutorial 7 - Turtlebot3 Brown Hall
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

## Overview:
In this tutorial you will import a map of Brown Hall to use with the navigation stack and the turtlebot3 simulations. Read more [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#ros-1-navigation) and [here](http://wiki.ros.org/navigation/Tutorials).
	
## System Requirements:

- **ROS+OS**: This tutorial is intended for a system with ROS noetic installed on the Ubuntu 20.04 LTS operating system. Alternate versions of ROS (i.e. - Kinetic, Noetic, etc.) may work but have not been tested. Versions of ROS are tied to versions of Ubuntu.
- **ROS**: Your computer must be connected to the internet to proceed. Update the system before you begin.
- **Workspace Setup**: The Turtlebot3 Simulator from tutorial 5 must be operational before completing tutorial 7.  

	
## Before Beginning:
	
- **ROSLAUNCH:** This tutorial involves using the roslaunch command which runs a muliple of nodes at once as described in the launch file. We will learn more about this later. 

- **Mouse for 3D viewing:** This simulator view is much easier use if you have a three button mouse plugged in, but this is not required.
	
		
## Part 1 - Custom World Configuration

Configure a custom gazebo world representing Brown Hall. The simulated world is generated from the files in the `/models/turtlebot3_brown/meshes` directory.


move the source directory of the workspace 

```
cd ~/catkin_ws/src
```

download a copy of the required package from github
```
git clone https://github.com/thillRobot/turtlebot3_brownhall.git
```

source the proper setup files for ROS (these shoud be in `~/.bashrc`)
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

export environment variables for gazebo simulator (add these to `~/.bashrc`)

```
export TURTLEBOT3_MODEL=waffle_pi

export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/turtlebot3_brownhall/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=~/catkin_ws/src/turtlebot3_brownhall/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/turtlebot3_brownhall/models:${GAZEBO_RESOURCE_PATH}
```


## Part 2 - Turtlebot3 Simulations in Brown Hall


It is convenient to use a lauch to combine the required commands to run a robot application. 

The launch file `turtlebot3_brownhall.launch` is in the `turtlbot3_brownhall/launch/` directory. This file confiures and starts the turtlebot3 simulations in a custom Gazebo world `turtlebot3_brown`.


```
<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="-2.0"/>
  <arg name="y_pos" default="-0.5"/>
  <arg name="z_pos" default="0.0"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_brownhall)/worlds/turtlebot3_brown3.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf"  args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
</launch>
``` 

### Part 2, Step 1 - Launch turtlebot3 in custom world

Use the following command to run the launch file. A new core will be started unless a previous roscore is running. The command line arguments are not set so the default values are used.

```
roslaunch turtlebot3_brownhall turtlebot3_brownhall.launch
```

The robot should appear in Brown Hall. A three button mouse is very helpful here.


### Part 2, Step 2 - Test robot with keyboard teleop
Turn on a keyboard node to test the virtual robot and map. 

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

The robot should react to keyboard commands in the teleop terminal. 

## Part 3 Turtlebot3 navigation in custom world

### Part 3, Challenge 1
Use ROS navigation plan a path from one place in the building to the next. 

install local planner used by turtlebot3
```
sudo apt update
sudo apt install ros-noetic-dwa-local-planner
```


#### make a new map of the virtual brown hall

follow the steps from tutorial 6 


Start gmapping
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Drive the robot around
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

```
cd ~/catkin_ws/src/turtlebot3_brownhall/maps

rosrun map_server map_saver -f bh3_map0  
```

first attempt produced a very bad map... 


#### start turtlebot3 navigation new map

with the `turtlebot3_brownhall.launch` command still running, execute the following to start navigation  


```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:='$(find turtlebot3_brownhall)/maps/bh3_map0.yaml'
```



### Part 3, Challenge 2
Use the the goal status topic and a custom C++ node to plan a mission containing multiple waypoints. The robot should wait at each waypoint for 5 seconds before proceeding with the mission.


### Tutorial Complete: 
After completing _Tutorial 7 - Turtlebot3 Simulator_, you are ready to learn about ... more ROS!
	

		
