# ROS Workshop - Tutorial 9 - Pinoeer LX Robot
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

## Overview:
In this tutorial you will interact with the LX robot using your laptop as a remote computer.

## System Requirements:

- **ROS+OS:** This tutorial is intended for a system with ROS Noetic installed on the Ubuntu 20.04 LTS operating system. You the branch selector above for alternate versions. Versions of ROS are generally tied to versions of Ubuntu.
- **Internet:** Your computer must be connected to the internet to proceed. 
	
## Before You Begin:

- **Workspace Setup:** This tutorial requires a Catkin Workspace as your working directory for creating packages. This was completed in [tutorial 4](https://github.com/thillRobot/ros_workshop/blob/main/module4/tutorial4_create_package/tutorial4_create_package.md) and can be used again for this tutorial.
	
## Important Note on Naming: 

The following names will be used in this tutorial.

- `control_computer`: users remote control computer 
	- `control_ip`: ipv4 address of `control_computer`
	- `catkin_ws`: workspace name on `control_computer`

	- `tutorial9` : custom package created in this tutorial in `catkin_ws` on `control_computer` 
	 	- `lx_publisher` : custom publisher node  
	 	- `lx_subscriber` : custom subscriber node  

- `lx_robot`: robot embedded computer
	- `robot_ip`: ipv4 address of `lx_robot`
	- `catkin_ws`: workspace name on `lx_robot` and `control_computer`

	- `lx_navigation`: lx navigation package in `catkin_ws` on `lx_robot`	

Follow the general guidelines for [naming in ROS](http://wiki.ros.org/ROS/Patterns/Conventions) if using different names.

- choose descriptive names, very long or very short names are hard to read
- **do not** include the < > symbols
- **do not** use spaces, UPPER CASE letters, or special characters (@,$,*, etc.)
- the underscore _ character **is** allowed 


## Network Setup

The `control_computer` will use WLAN to comminicate with the `lx_robot`. Both devices must be connected to the network and be able to resolve each others ip addresses. Read the ROS [network setup](http://wiki.ros.org/ROS/NetworkSetup) docs for more information.

Use `ping` to test connectivity.  




## Instructions for Creating a Custom Package and Node

### Update the control computer system before you begin

Run the following command on the `control_computer`
```
sudo apt update
```

```
sudo apt upgrade
``` 