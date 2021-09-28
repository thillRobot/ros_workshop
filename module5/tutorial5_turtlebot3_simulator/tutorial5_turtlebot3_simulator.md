# ROS Workshop - Turtorial 5 - Turtlebot3 Simulator
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

<img src="turtlebot3_simulations.png" alt="drawing" width="400"/>

## Overview:
After completing _Tutorial 4 - Create Package_, You have learned some basics of ROS, and you are ready for a more advanced robot. You can read more about the Turtlebot3 [here](https://www.turtlebot.com/) and [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).
	
## System Requirements:
- ROS+OS: This tutorial is intended for a system with ROS Melodic installed on the Ubuntu 18.04 LTS operating system. Alternate versions of ROS (i.e.  Kinetic, Noetic, etc.) may work but have not been tested. Versions of ROS are tied to versions of Ubuntu.
- ROS: Your computer must be connected to the internet to proceed. Update the system before you begin.
- Workspace Setup: You must have successfully setup a Catkin Workspace in tutorial 4.  

## Before You Begin:
	
- Backup the System: If you are using a virtual machine, it is recommend to make a snaphot of your virtual machine before you start each module. In the event of an untraceable error, you can restore to a previous snapshot. 
- ROSLAUNCH: This tutorial involves using the roslaunch command which runs a multiple of nodes at once as described in the launch file. We will learn more about this later. 
	

## Part 1 - Turtlebot3 Package Installation:

### Step 1 Update Repository List
Update the list of avilable packages before you get started. 
```
sudo apt update
```	
### Step 2 Install Turtlebot3 Packages 
Install the neccessary packages into your ROS system. This tutorial comes from [here]http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation}.
    	
**turtlebot3**
```
sudo apt install ros-melodic-turtlebot3
```

**turtlebot3_simulations**
```
sudo apt install ros-mnelodic-turtlebot3-simulations
```

**turtlebot3_gazebo**
```
sudo apt install ros-meloidic-turtlebot3-gazebo
```

### Step 3 Test Package Installation
The required packges should now be installed. Test that the packages are recognized in your workspace with the `roscd` command.
```
roscd turtlebot3
```
If the workspace is configured correctly the current directory should change to the location where the `turtlebot3` package was installed. 

## Part 2 - Turtlebot3 Testdrive:

### Step 1 Configure the simulator

The environment variable `TURTLEBOT_MODEL` must be set to define the robot type. Use `echo` to add this line to the `.bashrc` script so you do not have to do it for each terminal. 
``` 
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```
Now that run that script with `source`. It will also run each time you open a new terminal.	
```	
source ~/.bashrc
```	

### Step 2 - Start the simulator

The following command uses `roslaunch` to configure and run multilple ROS nodes with a single command. The gazebo simulator will open containing the robot in the example environment. 

```	 
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Enter the following command in a new terminal to add keyboard control to the system. Test that the keyboard drives the robot. 

``` 
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Now turn on the node to produce robot data in the simulated world.  \\

``` 
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```

Open RVIZ to view the data. This is a very useful tool. 	
```
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

## Tutorial Complete:
After completing _Tutorial 5 - Turtlebot3 Simulator_, you are ready to learn about robot navigation with SLAM and GMAPPING ! See the tutorial referenced above if you are ready to proceed.