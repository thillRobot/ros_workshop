# generate URDF

The goal is to generate a URDF (Unified Robot Description Format) for a custom robot.

#### Create a workspace for testing, or use a pre-existing workspace of your choice. 
This tutorial will use `catkin build`, but `catkin_make` should work also.

```
mkdir -p catkin_build_ws/src
cd catkin_build_ws
catkin make
```

#### Install neccesary ROS packages with `apt` 

```
sudo apt ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
```

#### Clone `examplerobot_description` and `examplerobot_gazebo` from thillRobot on github into your workspace

```
cd ~/catkin_build_ws/src
git clone https://github.com/thillRobot/examplerobot_gazebo.git
git clone https://github.com/thillRobot/examplerobot_description.git
```

Compile the package with `catkin build`

```
cd catkin_build_ws
catkin_build
```

Show the model in RVIZ

```
roslaunch examplerobot_gazebo rviz.launch
```

I have made some progress, but now the frames need to be setup correctly. 


The first two links should show in rviz and you can control the joint angle with the slider bar in the separate window. The joint is not in the right location, but at least I figured out the XACRO stuff! Woop Woop!


#### spawn the robot in the gazebo simulator 

Spawn the robot in the default world 'empty_world'
```
roslaunch examplerobot_gazebo gazebo.launch
```
Spawn the robot in the world defined in `examplerobot_description/worlds`. You should see the gas station from the Gazebo tutorials. 

```
roslaunch examplerobot_gazebo examplerobot.launch
```

You can see that it worked perfectly... well not exactly. The links did load into the example world, but there are some serious issues. The links are not defined in the correct frames! DH TIME! This is not a great choice of robots anyway, it needs to be modified to have usable kinematics. Either way, this shows an example of how to import STL files as links and define the joints. 

![examplerobot in gazebo](https://github.com/thillRobot/ros_workshop/blob/noetic-devel/module11/tutorial11_generate_urdf/images/examplerobot_gazebo_fig1.png)

![examplerobot in gazebo](https://github.com/thillRobot/ros_workshop/blob/noetic-devel/module11/tutorial11_generate_urdf/images/examplerobot_gazebo_fig2.png)

I was running into the error: `This robot has a joint named "base_link__link_01" which is not in the gazebo model.` Through some digging around I learned that this can be fixed by adding inertia `base_link`. This may have been from the kinetic example I wasa using from the `construct` (see link above). While trying to fix this, I read that the robot should be stored in two packages `<ROBOTNAME>_gazebo` and `<ROBOTNAME>_description` in the official Gazebo docs (http://gazebosim.org/tutorials/?tut=ros_urdf), so I created these as separate repositories. I want to combine them in a metapackage, but the launch files were not recognized when I tried that...try again later. 

#### NEW! - 6DOF 'examplerobot'

```
roslaunch examplerobot_gazebo rviz.launch
```
![examplerobot in gazebo](https://github.com/thillRobot/ros_workshop/blob/noetic-devel/module11/tutorial11_generate_urdf/images/examplerobot_rviz_fig1.png)

The forward kinematics is working properly (with weird eccentric rotation at first bug!) as you can see in the figure below.

![examplerobot in gazebo](https://github.com/thillRobot/ros_workshop/blob/noetic-devel/module11/tutorial11_generate_urdf/images/examplerobot_rviz_fig2.png)

```
roslaunch examplerobot_gazebo gazebo.launch
```
![examplerobot in gazebo](https://github.com/thillRobot/ros_workshop/blob/noetic-devel/module11/tutorial11_generate_urdf/images/examplerobot_gazebo_fig3.png)




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