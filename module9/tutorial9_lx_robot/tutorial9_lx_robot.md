# ROS Workshop - Tutorial 9 - Pinoeer LX Robot
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

## Overview:
In this tutorial you will interact with the LX robot using your laptop as a remote computer.

## System Requirements:

- **ROS+OS:** This tutorial is intended for a system with ROS Noetic installed on the Ubuntu 20.04 LTS operating system. Use the branch selector above for alternate versions. Versions of ROS are generally tied to versions of Ubuntu.
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
	 	- `tutorial9.launch`: launch to start custom process for this tutorial

- `lx_robot`: robot embedded computer
	- `robot_ip`: ipv4 address of `lx_robot`
	- `catkin_ws`: workspace name on `lx_robot` and `control_computer`

	- `lx_navigation`: lx navigation package in `catkin_ws` on `lx_robot`	
		- `lx_remote_drive.launch`: launch to receive and drive from remote `cmd_vel`
		- `lx_drive.launch`: launch to receive and drive from local `cmd_vel`
		- `lx_navigation.launch`: launch to start `navigation` and load map

Follow the general guidelines for [naming in ROS](http://wiki.ros.org/ROS/Patterns/Conventions) if using different names.

- choose descriptive names, very long or very short names are hard to read
- **do not** include the < > symbols
- **do not** use spaces, UPPER CASE letters, or special characters (@,$,*, etc.)
- the underscore _ character **is** allowed 


## Part 1 - Network Setup

The `control_computer` will use WLAN to comminicate with the `lx_robot`. Both devices must be connected to the network and be able to resolve each others ip addresses. Read the ROS [network setup](http://wiki.ros.org/ROS/NetworkSetup) docs for more information.

### Step 1 - obtain ip addresses

The ip addresses of both computers must be known to begin. Power on and log into `lx_robot` and run the following command to obtain the ipv4 address. This step is only required if the address of the robot changes which may happen occasionally when using DHCP(dynamic host configuration protocol).

```
ip a
```

The ipv4 address of `lx_robot` is `10.104.66.X` as shown in the command output. 

If using a VirtualBox VM, the network settings must be adjusted so the VM is assigned an ip address that is accessible from the network. With the VM powered off, change the Virtualbox network settings. Set network > adapter type to `bridged`. The name field below should automatically show the network adapter name which will differ based on the specific hardware.

Turn on the `control_computer` VM. Log in and run the following command to obtain the ipv4 address.

```
ip a
```

The ipv4 address of `control_computer` is `10.104.66.Y` as shown in the command output. 


### Step 2 - test connectivity

Test connectivity from `control_computer` to `lx_robot`. Run the following from `control_computer`.

```
ping <robot_ip>
```

for example:

```
ping 10.104.66.X

```



Test connectivity from `lx_robot` to `control_computer`. Run the following command from `lx_robot`.

```
ping <control_ip>
```

for example:
```
ping 10.104.66.Y
```

If both tests are successful, proceed. Otherwise troubleshooting network connectivity...


### Step 3 - remote access

The address of `lx_robot` can be used to open a remote terminal from the `control_computer`. Run the following command from `control_computer` to open a remote termial (secure shell) on `lx_robot`. Notice the text left on the left side of the terminal shows the current user and computer name after starting the remote terminal.

```
ssh researcher@10.104.66.X
```

Now test a simple command on `lx_robot` through the remote terminal on `control_computer`.

for example:
```
ls
```

The home directory of `lx_robot` is shown in the command output. 

To log out of the remote terminal use the following.
```
exit
```


### Step 4 - ROS networking 

The ip addresses of both computers must be known by each other for ROS to function across the two. Set the ip addresses of the computers in the appropriate environment variables in the `/.bashrc` files for each computer.

Add the following lines to the bottom of `~/.bashrc` on `lx_robot`.

```
export ROS_MASTER_URI=http://10.104.66.X:11311
export ROS_IP=10.104.66.X
```

Save the file and apply the changes with the following command.

```
source ~/.bashrc
```

Add the following lines to the bottom of `~/.bashrc` on `control_computer`.

```
export ROS_MASTER_URI=http://10.104.66.X:11311
export ROS_IP=10.104.66.Y
```

Save the file and apply the changes with the following command.

```
source ~/.bashrc
```

Perform a simple test of ROS across the network. In this tutorial, the roscore will always run from `lx_robot.`  Start a roscore on `lx_robot`.

```
roscore
```

Test that `control_computer` can access the roscore by running a simple ROS command on `control_computer`. Close any previously running ROS processes on `control_computer` and run the following.

```
rostopic list
```

If you see a short list of topics, then ROS is working across the network. If the network is configured incorrectly, roscore errors will be shown.


## Part 2 - LX Robot Teleop Control


Run the remote drive node on the `lx_robot`. Make sure the estop is not pressed before running the following command on `lx_robot`.

```
roslaunch lx_navigation lx_remote_drive.launch robot_ip:=10.104.66.X control_ip:=10.104.66.Y

```

Run the teleop twist keyboard node on the `control_computer`. 

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Keyboard commands entered in the terminal should publish a `cmd_vel` topic to be read by the subscriber node on `lx_robot`. The lx robot can move fast, so it is advisable to turn the command speeds down before driving. 

Turn on RVIZ to visualize the available data.

```
rosrun rviz rviz
```

Add topics using the add button in the bottom left. The selected topics and screen settings can be saved to an rviz configuration file (.rviz) and loaded to save time. It is the convention to save rviz config files in a directory called `rviz` in the associated package. example: `~/catkin_ws/src/tutorial9/rviz/tutorial9.rviz`



## Part 3 - LX Robot Navigation