# generate URDF

The goal is to generate a URDF (Unified Robot Description Format) for a custom robot.

#### Create a workspace for testing, or use a pre-existing workspace of your choice. This tutorial will use `catkin build'

```
cd ~/catkin_build_ws
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin build
```
After compiling, source the workspace setup file. 

```
source ~/catkin_build_ws/devel/setup.bash
```

#### Install neccesary ROS packages with `apt` 

```
sudo apt ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
```

#### Clone `generate_urdf` from thillRobot on github into your workspace

```
cd catkin_build_ws/src
git clone https://github.com/thillRobot/generate_urdf.git
```

Compile the package with `catkin build`

```
cd catkin_build_ws
catkin build
```

Now, use this launch file to load the example STL from URDF and show it in rviz

```
roslaunch generate_urdf display.launch model:='$(find generate_urdf)/urdf/me4140-example.urdf'
```

There is still much to do, but this is a start!

```
roslaunch generate_urdf display.launch model:='$(find generate_urdf)/urdf/me4140-example.xacro'
```

I have made some real progress, but now the frames need to be setup correctly. 

```
roslaunch generate_urdf rviz.launch
```

The first two links should show in rviz and you can control the joint angle with the slider bar in the separate window. The joint is not in the right location, but at least I figured out the XACRO stuff! Woop Woop!

