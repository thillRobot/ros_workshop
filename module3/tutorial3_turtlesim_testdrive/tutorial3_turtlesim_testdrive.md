# ME4140 - Introduction to Robotics - Fall 2021

# ROS Workshop - Tutorial 3 - Turtlesim Testdrive

## Overview
After completing _Tutorial 2 - Install ROS_ , your system is setup. You are ready to begin with Turtlesim, a simplistic robot model and simulator that serves as the {\it Hello World of ROS}. You can read more about turtlesim [here](http://wiki.ros.org/turtlesim) the ROS wiki. 

## System Requirements
**ROS+OS:** This tutorial is intended for a system with ROS Melodic installed on the Ubuntu 18.04 LTS operating system. Alternate versions of ROS (i.e. - Kinetic, Noetic, etc.) may work but have not been tested. Versions of ROS are tied to versions of Ubuntu.
**Internet:** Your computer must be connected to the internet to proceed. Downloading and installing the turtlesim package will only take a few minutes

## Disclaimer

- **Copy and Paste Errors:** Be careful if you use copy and paste for commands. Make sure to copy the entire command.
    
- **Practice with the Terminal:** The commands in this tutorial are relatively short, and it may help improve understanding to type them manually. Press **Tab** for _auto-completion_!
    
## Installation Instructions

Press **Ctrl+Alt+t** to open a new terminal, then carefully enter each command into the terminal then press **Enter**. The terminal commands are shown in gray boxes. _You will have multiple terminals open at once during this tutorial_. 

### Update your Ubuntu package list. 

It is recommended to do this before you install something new.

```
sudo apt update
```
### Install `turtlesim` package for ROS Melodic
Also, install a keyboard controller node `teleop-twist-keyboard`. This will take a few moments. 

```
sudo apt install ros-melodic-turtlesim ros-melodic-teleop-twist-keyboard
```
