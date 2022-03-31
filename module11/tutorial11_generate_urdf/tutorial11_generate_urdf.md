# generate URDF

The goal is to generate a URDF (Unified Robot Description Format) for a custom robot.

#### Create a workspace for testing, or use a pre-existing workspace of your choice. This tutorial will use `catkin_make`

```
mkdir -p catkin_make_ws/src
cd catkin_make_ws
catkin_make
```

#### Install neccesary ROS packages with `apt` 

```
sudo apt ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
```

#### Clone `generate_urdf` from thillRobot on github into your workspace

```
cd catkin_make_ws/src
git clone https://github.com/thillRobot/generate_urdf.git
```

Compile the package with `catkin_make`

```
cd catkin_make_ws
catkin_make
```

Now, use this launch file to load the example STL from URDF and show it in rviz

```
roslaunch generate_urdf display.launch model:='$(find generate_urdf)/urdf/me4140-example.urdf'
```

There is still much to do, but this is a start!


I have made some real progress, but now the frames need to be setup correctly. 

```
roslaunch generate_urdf rviz.launch
```

The first two links should show in rviz and you can control the joint angle with the slider bar in the separate window. The joint is not in the right location, but at least I figured out the XACRO stuff! Woop Woop!

#### generate URDF in docker 

The process described above can be completed using docker. For the graphics to work this requires docker-CE and nvidia-docker2 

First, build the image using the Dockerfile.

```
docker build -t generate_urdf
```

First adjust the xauth permission settings.

```
xhost +
```

Run the example with the .bash script. 
```
./generate_urdf_rviz.bash
```




#### spawning the robot in the gazebo simulator

```
roslaunch generate_urdf gazebo.launch
```


I was running into the error: `This robot has a joint named "base_link__link_01" which is not in the gazebo model.` Through some digging around I learned that this can be fixed by adding inertia `base_link`. This may have been from the kinetic example I wasa using from the `construct` (see link above). While trying to fix this, I read that the robot should be stored in two packages `<ROBOTNAME>_gazebo` and `<ROBOTNAME>_description` in the official Gazebo docs (http://gazebosim.org/tutorials/?tut=ros_urdf), so I created these as separate repositories. I want to combine them in a metapackage, but the launch files were not recognized when I tried that...try again later. 

First clone the new packages from giuthub. 

```
cd ~/catkin_build_ws/src
git clone https://github.com/thillRobot/examplerobot_gazebo.git -b noetic
git clone https://github.com/thillRobot/examplerobot_description.git -b noetic

catkin build
source devel/setup.bash
```

Now spawn the robot in the example world

```
roslaunch examplerobot_gazebo examplerobot.launch
```

You can see that it worked perfectly... well not exactly. The links did load into the example world, but there are some serious issues. The links are not defined in the correct frames! DH TIME!

![examplerobot in gazebo](https://github.com/thillRobot/examplerobot_description/blob/noetic-devel/images/examplerobot_gazebo_fig1.png)

![examplerobot in gazebo](https://github.com/thillRobot/examplerobot_description/blob/noetic-devel/images/examplerobot_gazebo_fig1.png)

