# using moveit

This is my notes for using moveit with ROS Melodic. I can't quite call it a tutorial yet.

## Resources

Read the docs! It looks like they are in two places...

 - [moveit](https://moveit.ros.org/)

 - [moveit_tutorials](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) 


## Installation

It is assumed that you have install ROS melodic and sourced the setup file.

```
source /opt/ros/melodic/setup.bash
```


Follow the instructions [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) to install `moveit`. The commands are copied here for convenience. 

Upgrade system

```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
```

Install neccesary packages

```
sudo apt-get install ros-melodic-catkin python-catkin-tools
```

Install moveit

```
sudo apt install ros-melodic-moveit 
```

Create a workspace for moveit

```
mkdir -p ~/ws_moveit/src
```

Download the moveit_tutorials package into the melodic workspace.
```
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel

```

Prepare the workspace and compile with `catkin build`. 

```
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin build
```

Install the Franka Robot Description Package (because tutorial said so...)

```
sudo apt-get install ros-melodic-franka-description
```

Download the aubo_robot package into the workspace so that it can be loaded by the `moveit setup assistant`

```
git clone https://github.com/AuboRobot/aubo_robot.git ~/Downloads/aubo_robot
```

Run the `moveit setup assistant` 

Follow the tutorial here to generate a  Gazebno compatible URDF from the URDF in the aubo package. This sounds promising.

I used the the file `/aubo_robot/aubo_description/urdf/aubo_i3.urdf` to generate the urdf `aubo_i3_gazebo.urdf` and a package by the same name. We are no there yet, but this seems like progress.


