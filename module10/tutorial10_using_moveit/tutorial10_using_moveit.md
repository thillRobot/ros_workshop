# using moveit

This is my notes for using moveit with ROS. This is pretty cool.

## ROS Version

This is the `melodic` branch. 

Select the version of ROS by choosing the appropriate repository branch.


## Resources

Read the docs! It looks like they are in two places...

 - [moveit](https://moveit.ros.org/)

 - [moveit_tutorials](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) 

 - [Stephen Zuccaro on Youtube](https://www.youtube.com/channel/UCofPudSKrNzT3vccJD6VDyA)


## Installation

This has been tested on Ubuntu 18.04 (minimal) in a VM (see module 1)

It is assumed that [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) is installed 

Source the ROS setup file before starting. This is probably in your `~/.bashrc`

```
source /opt/ros/$ROS_DISTRO/setup.bash
```

### Install Moveit 

Follow the instructions [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) to install `moveit` with `apt` (aka _binary installation_). The commands are copied here for convenience. The _source installation_ is not needed unless you want to modify the Moveit package.  

Upgrade system

```
rosdep update
sudo apt update
sudo apt upgrade  # tutorial above uses `dist-upgrade` but I do not think that matters
```

Install neccesary packages (These are available in `venv`, test that next)

```
sudo apt install ros-$ROS_DISTRO-catkin python-catkin-tools
```

Install moveit

```
sudo apt-get install ros-$ROS_DISTRO-moveit
```

Create a workspace for moveit. This will be a `catkin build` workspace, but `catkin_make` should work also.

```
mkdir -p ~/catkin_build_ws/src
```

Prepare the workspace and compile with `catkin build`. Read about `catkin_build` in the [catkin_tools docs](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)

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
Put this in `~/.bashrc` for convinience. This is optional but reccomended.
```
echo "source ~/catkin_build_ws/devel/setup.bash" >> ~/.bashrc
```

### Import a robot from URDF

Import a robot of your choice. At minimum the model .stl files and a robot .urdf (Universal Robot Descriptor File) are required. This example uses the _Aubo i5_ from [AuboRobot](https://github.com/AuboRobot/aubo_robot)

Clone the `aubo_robot` package into a temporary location like `~/Downloads`. It will not compile in ROS melodic, so the full package should not be in `ws_moveit`. Also, the `UpdateMoveitLib` patch is not reccommend because it modifies the system wide libraries in an unknown and non-standard way. Don't bork the deps!

```
cd ~/Downloads
git clone https://github.com/AuboRobot/aubo_robot.git -b $ROS_DISTRO
```

Copy the `aubo_description` package from inside the `aubo_robot` package into the `ws_moveit`. (`-r` is the recursive flag for copying directories )


```
cp -r ~/Downloads/aubo_robot/aubo_description ~/catkin_build_ws/src/aubo_description
```

The `aubo_description` package must be in an workspace that it can be found by the `moveit setup assistant`. Verify that your workspace compiles, and then source the workspace setup files so that changes are recognized.

```
cd ~/catkin_build_ws
catkin build
source devel/setup.bash
```

Run the `moveit setup assistant` from the [tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)  
```
roslaunch moveit_setup_assistant setup_assistant.launch
```

Complete steps 1-12 to generate a Gazebo compatible URDF from the URDF in the aubo package. Follow the instructions in thr tutorial above.

I used the the file `/aubo_robot/aubo_description/urdf/aubo_i5.urdf` to generate the urdf `aubo_i5_gazebo.urdf` and a package named `aubo_i5_moveit_config`


Create a directory in the new package for the gazebo urdf.

```
mkdir ~/catkin_build_ws/src/aubo_i5_moveit_config/gazebo
gedit ~/catkin_build_ws/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf
```

Paste in the urdf XML from the clipboard and save the file. If you lost the data on the clipboard, you do not have to start over. Launch the setup assistant again, and choose `edit existing configuration`. Load the config package you made and go get the XML again.  


Compile the package with catkin build. The new moveit config package should compile without errors.

```
cd ~/catkin_build_ws
catkin build

source devel/setup.bash
```

### Testing Robot in Moveit

#### Test 1 - Gazebo Import 

Now start the Gazebo simulator with an empty world

```
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
```

Add the robot to the simulator using the urdf for gazebo. This should be improved with `find` or something similar. 

```
rosrun gazebo_ros spawn_model -file ~/catkin_build_ws/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf -urdf -x 0 -y 0 -z 1 -model aubo_i5
```

It looks like it worked. Woop Woop! 

<img src="png_images/aubo_i5_gazebo.png" alt="drawing" width="700"/>


#### Test 2 - RVIZ Demo

The `aubo_i5_moveit_config` package contains a collection of launch files. The example `demo.launch` displays the robot in RVIZ and allows the user to plan and execute arm motions using the the Moveit panel.

```
roslaunch aubo_i5_moveit_config demo.launch
```

<img src="png_images/aubo_i5_demo_launch01.png" alt="drawing" width="700"/> <img src="png_images/aubo_i5_demo_launch02.png" alt="drawing" width="700"/>

Configure the display menu to show the start and goal locations by selecting _Planning Requests > Query Start State_ in the displays menu on the left.

<img src="png_images/aubo_i5_demo_launch03.png" alt="drawing" width="500"/> <img src="png_images/aubo_i5_demo_launch04.png" alt="drawing" width="500"/>

Choose the the start and goal locations by dragging the end effector to the desired position. Animate the robot by pressing _plan_ or _plan and execute_ in the Moveit panel on the bottom left.

<img src="png_images/aubo_i5_demo_launch05.png" alt="drawing" width="800"/> 

Show the intermediate poistions by selecting _Planned Path > Show Trail_, and disable the infinite loop by deselecting _Planned Path > Loop Animation_. The settings are described in more detail in the Moveit [getting started](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) tutorial.

 
