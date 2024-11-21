# ROS Workshop - Tutorial 6 - Turtlebot3 Navigation
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

## Navigation:

What do we mean by navigation? This means different things in different places. Here, we mean the navigation stack in ROS noetic. This tutorial comes from [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation) 

<img src="turtlebot3_models.png" alt="drawing" width="400"/>

(Image: [emanual.robotis.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#features) )

	
## Overview:
In this tutorial you will learn to use the navigation stack with the turtlebot3 simulations. Read more [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#ros-1-navigation) and [here](http://wiki.ros.org/navigation/Tutorials).
	
## System Requirements:

- **ROS+OS**: This tutorial is intended for a system with ROS noetic installed on the Ubuntu 20.04 LTS operating system. Alternate versions of ROS (i.e. - Kinetic, Noetic, etc.) may work but have not been tested. Versions of ROS are tied to versions of Ubuntu.
- **ROS**: Your computer must be connected to the internet to proceed. Update the system before you begin.
- **Workspace Setup**: The Turtlebot3 Simulator from tutorial 5 must be operational before completing tutorial 6.  

	
## Before Beginning:
	

- **ROSLAUNCH:** This tutorial involves using the roslaunch command which runs a muliple of nodes at once as described in the launch file. We will learn more about this later. 

- **Mouse for 3D viewing:** This simulator view is much easier use if you have a three button mouse plugged in, but this is not required.
	
		
## Part 1 - Install navigation and gmapping packges
	

### Step 1 Install `navigation` and `gmapping` 
Install the packages required for autonomous navigation if you have not already. It will not hurt to run the install command again.

```
sudo apt update && sudo apt upgrade

sudo apt install ros-noetic-navigation ros-noetic-gmapping
```

### Step 2 -  Set the robot model. This only needs to be done once. Modify the .bashrc file If you want to change models.

```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc

source ~/.bashrc
```

### Step 3 - Create a package `tutorial6` to use for this exercise. Also create a directory to store maps.

```
cd ~/catkin_ws/src

catkin_create_pkg tutorial6 std_msgs roscpp rospy

mkdir ~/catkin_ws/src/tutorial6/maps
```



## Part 2 - Generate a map of the virtual space:



### Step 1 Start the turtlebot3 simulator

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Step 2 Start gmapping SLAM
 
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Step 3 Collect sensor data

Drive the robot around with the keyboard as the turtlesim automatically collects pose and Lidar data. The teleop node can be stopped when you have collected enough data to generate a map of the entire area. Leave the `gmapping` process running.

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Step 4 Save Map

Use the `-f` option to set the filename. 

_Option 1_ Use the absolute paths to the map file in new directory. 

```
rosrun map_server map_saver -f ~/catkin_ws/src/tutorial6/maps/demo_world  
```

_Option 2_ Navigate to package directory and use relative paths. 

```
cd ~/catkin_ws/src/tutorial6/maps

rosrun map_server map_saver -f demo_world  
```

Two new map files both with the same filename (`demo_world.pgm` and `demo_world.yaml`) will appear in the `maps` folder after Step 4. These map files can be moved or copied for for backup, but both files are required and the file names must match. After saving the map, stop the `gmapping` node.


## Part 3 - Navigate Virtual space:
Now that navigation is installed and there is a map saved to file, the robot can perform autonomous point to point navigation with dynamic obstacle avoidance using the map and RVIZ as the user interface. 

### Step 1 - Start the turtlebot3 simulator
``` 
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Step 2 - Turn on navigation and RVIZ. 
Use the name of the map you created for the map_file option in the command. If you navigate to the package with the maps the absolute paths will work.


_Option 1_ Navigate to the package directory and then use relative paths to the map files. 

```
cd ~/catkin_ws/src/tutorial6

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=maps/demo_world.yaml

```

_Option 2_ Use the built-in ROS Change Directory tool `roscd` to quickly navigating to a ROS package on the system. This method is portable because it is not specfic to the location of the package containing the maps. 

```
roscd tutorial6

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=maps/demo_world.yaml

```

_Option 3_ (**preffered method**) Use `find` to access the package containing the maps without changing the current directory. Use relative paths from the package directory in the find command.

```
 roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:='$(find tutorial6)/maps/demo_world.yaml'
```


The gazebo window will open containing your robot, and you will also see the rviz window open separately. Find and test the following features of navigation in RVIZ. 
	
- _Pose Estimate_ - Click and drag the direction to set the current pose estimate of the robot.
	
- _2D Nav Goal_ - Click and drag the direction to define a goal point for the robot in the map.

<img src="turtlebot3_rviz.png" alt="drawing" width="400"/>


Check the available topics with rostopic.
``` 
rostopic list
```

```
rosrun rqt_graph rqt_graph 
```

## Part 4 - Issues Loading Maps Files:

You may have run into an issue in which `turtlebot3_navigation` cannot load the map file. A typical error message is shown below, along with the preferred solution.

```
[ERROR] [1604667817.760623311]: Map server could not open /demo_world.yaml.
```

### Step 1 
Check that the top line of `map.yaml` contains the name of the map with the `.pgm` extension. The path to the map file should not be included in this line. The file generated in the this tutorial is shown below.

```
image: demo_world.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Step 2 
Check that `map.yaml` and `map.pgm` exist in a directory named maps/ in a package in the ROS workspace.

### Step 3 
Check that the workspace will compile by running
 `catkin_make` from the top of the workspace. You should see in the terminal output, that your package was successfully built. 

### Step 4

Use the `find` command to point the node to the maps directory of your package where the map files are stored. Repeat Part 3 with the addition of the `find` command in the map argument. 

Start the simulator
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
 
Turn on the navigation nodes and RVIZ. Use the name of the map you created, and it should be recogonized and displayed in RVIZ. 

```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:='$(find tutorial6)\maps\tutorial6.yaml'
```

## Part 5 - Goals and Status

The _Pose Estimate_ and _2D Nav_ buttons in RVIZ are useful for testing but are not suitable for most robotic applications. This section will show how to publish and subcribed to the relevant topics using a custom C++ node as the interface to ROS navigation.

### Step 1 - Create Goal Publisher Node
This node will publish goal point data to the appropriate topic to move the robot. Create the node in the previously used package _tutorial6_ source folder.

```
$ gedit ~/catkin_ws/src/tutorial6/src/publish_goal.cpp
```
Copy the following code into the editor and save the file.
```c++
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_goal");
    ros::NodeHandle n;
    ros::Publisher ttu_publisher =
    n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Rate loop_rate(10);
    geometry_msgs::PoseStamped msg;
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id="map";
    
    int count = 0;   
    while ((ros::ok())&&(count<5))
    {
        msg.pose.position.x = 3.0;
        msg.pose.position.y = 2.0;
        msg.pose.position.z = 0;
        msg.pose.orientation.w = 1.0;
        ttu_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}
```

Edit the CMakeLists.txt file so the new node can be compiled.

```
gedit ~/catkin_ws/src/tutorial6/CMakeLists.txt
```

Copy the following lines into the bottom of the file and save.

```
add_executable(publish_goal src/publish_goal.cpp)
target_link_libraries(publish_goal ${catkin_LIBRARIES})
```

Move to the top of the workspace and compile to generate an executable from the source code.

```
cd ~/catkin_ws
catkin_make
```

The workspace should compile without errors before continuing.


### Step 2 - Test the Goal Publisher Node

Start the turtlebot3 robot simulation with commands from _Tutorial6 - Turtlebot3 Navigation_

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
In a second terminal, turn on navigation and RVIZ using the custom map
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:='$(find tutorial6)\maps\demo_world.yaml'
```

Run the publish goal node in a third terminal.
```
rosrun tutorial6 publish_goal
``` 
The robot should begin planning a path to the goal. If a valid path is found, the robot will begin to move toward the goal.

### Step 3 - Create Status Subscriber Node

Create a new file for the subcriber node source code in the same package 
```
gedit ~/catkin_ws/src/tutorial6/src/subscribe_status.cpp
```
Copy and paste the following code into the file and save

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"

void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    ROS_INFO("Subscriber Callback Executed");
    if (!msg->status_list.empty())
    {
        actionlib_msgs::GoalStatus goalStatus = msg->status_list[0];
        ROS_INFO("Status Recieved: %i",goalStatus.status);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_status");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/move_base/status", 1000, statusCB);
    ros::spin();
    return 0;
}
```
Edit the CMakeLists.txt file so the new node can be compiled.

```
gedit ~/catkin_ws/src/tutorial6/CMakeLists.txt
```

Copy the following lines into the bottom of the file and save.

```
add_executable(subscribe_status src/subscribe_status.cpp)
target_link_libraries(subscribe_status ${catkin_LIBRARIES})
```

Move to the top of the workspace and compile to generate an executable from the source code.

```
cd ~/catkin_ws
catkin_make
```

The workspace should compile without errors before continuing.


### Step 4 - Test Subscribe Status Node
Start the simulator with the same commands as before and run the subscribe_status node. Read about the GoalStatus message [here](https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html)

```
rosrun tutorial6 subscribe_status
```

The status topic should contain an integer 0-9 indicating one of the following states.

```
GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text
```



### Tutorial Complete: 
After completing _Tutorial 6 - Turtlebot3 Simulator_, you are ready to learn about ... more ROS!
	
<img src="../../charlie_robot.jpeg" alt="drawing" width="400"/>

NOW, YOU KNOW ABOUT ROS! GOOD JOB!
	
	
		
