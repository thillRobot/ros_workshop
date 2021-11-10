# using moveit

This is my notes for using moveit with ROS. I can't quite call it a tutorial yet.

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

Source the ROS setup file before starting. This is probably in your `~/.bashjrc`

```
source /opt/ros/$ROS_DISTRO/setup.bash
```

### Install Moveit 

Follow the instructions [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) to install `moveit` with `apt` (aka _binary installation_). The commands are copied here for convenience. The _source installation_ is not needed unless you want to modify the Moveit package.  

Upgrade system

```
rosdep update
sudo apt update
sudo apt upgrade  # tutorial above uses `dist-upgrade`
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
mkdir -p ~/ws_moveit/src
```

### Download and compile moveit_tutorials and moveit_examples

Clone the package into the workspace  
```
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b $ROS_DISTRO-devel
```

Clone the `thillRobot/moveit_examples` package for into the workspace.
```
git clone https://github.com/thillRobot/moveit_examples.git
```

Prepare the workspace and compile with `catkin build`. Read about `catkin_build` in the [catkin_tools docs](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)

```
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin build
```
After compiling, source the workspace setup file. 

```
source ~/ws_moveit/devel/setup.bash
```
Put this in `~/.bashrc` for convinience. This is optional.
```
echo "source ~/ws_moveit/devel/setup.bash" >> ~/.bashrc
```

### Import a robot from URDF

Import a robot of your choice. At minimum the model .stl files and a robot .urdf (Universal Robot Descriptor File) is required. This example uses the _Aubo i5_ from [AuboRobot](https://github.com/AuboRobot/aubo_robot)

Clone the `aubo_robot` package into a different workspace so that it can be loaded by the `moveit setup assistant`. It will not compile, so it should not be in `ws_moveit`. The `UpdateMoveitLib` patch is not reccommend because is modifies the system wide libraries in an unknown and non-standard way. Don't bork the deps!

```
mkdir -p ~/ws_aubo/src
cd ~/ws_aubo
catkin build 

git clone https://github.com/AuboRobot/aubo_robot.git -b $ROS_DISTRO src/aubo_robot

source ~/ws_aubo/devel/setup.bash # just for now
cd ~ 
```


Run the `moveit setup assistant` from the [tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)  
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
Complete steps 1-12 to generate a Gazebo compatible URDF from the URDF in the aubo package.

I used the the file `/aubo_robot/aubo_description/urdf/aubo_i5.urdf` to generate the urdf `aubo_i5_gazebo.urdf` and a package named `aubo_i5_moveit_config`


Create a directory in the new package for the gazebo urdf.

```
mkdir ~/ws_moveit/src/aubo_i5_moveit_config/gazebo
vim ~/ws_moveit/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf
```

Paste in the urdf XML from the clipboard.


Compile the package with catkin build

```
cd ~/ws_moveit
catkin build

source ~/ws_moveit/devel/setup.bash
```

### Testing Robot in Moveit

#### Test 1 - Gazebo Import 

Now start the Gazebo simulator with an empty world

```
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
```

Add the robot to the simulator using the urdf for gazebo. This should be improved with `find` or something similar. 


```
rosrun gazebo_ros spawn_model -file ws_moveit/src/moveit_examples/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf -urdf -x 0 -y 0 -z 1 -model aubo_i5
    
    [INFO] [1636155628.986157]: Loading model XML from file ws_moveit/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf
    [INFO] [1636155628.992780]: Waiting for service /gazebo/spawn_urdf_model
    [INFO] [1636155628.997679]: Calling service /gazebo/spawn_urdf_model
    [INFO] [1636155629.301420]: Spawn status: SpawnModel: Successfully spawned entity
```

It looks like it worked. Woop Woop! This first example tends to crash on the virtual box used for testing.

<img src="png_images/aubo_i5_gazebo.png" alt="drawing" width="700"/>


If the `ws_aubo` workspace is not sourced the error below is shown.  This can be fixed by moving the `aubo_description` package into the Moveit workspace.

```
[rospack] Error: package 'aubo_description' not found
[librospack]: error while executing command
[FATAL] [1636155476.819294674]: Package[aubo_description] does not have a path
^C[gazebo_gui-3] killing on exit
```

I do not want both workspaces sourced. So I copied the missing package from ws_aubo to ws_moveit. Read the [catkin_tools docs](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) about chained workspaces.It seems like it is OK to have both.

```
mkdir -p ~/ws_moveit/src/moveit_examples/aubo_robot
cp -r ~/ws_aubo/src/aubo_robot/aubo_description ~/ws_moveit/src/moveit_examples/aubo_description

cd ~/ws_moveit
catkin build
```

Now close both terminals and do not source the aubo workspace. Repeat the test.

#### Test 2 - RVIZ Demo

The `aubo_i5_moveit_config` package contains a collection of launch files. The example `demo.launch` displays the robot in RVIZ and allows the user to plan and execute arm motions using the the Moveit panel.

```
roslaunch aubo_i5_moveit_config demo.launch
```

<img src="png_images/aubo_i5_demo_launch01.png" alt="drawing" width="700"/> <img src="png_images/aubo_i5_demo_launch02.png" alt="drawing" width="700"/>

Configure the display menu to show the start and goal locations by selecting _Planning Requests > Query Start State_ in the displays menu on the left.

<img src="png_images/aubo_i5_demo_launch09.png" alt="drawing" width="700"/> <img src="png_images/aubo_i5_demo_launch09.png" alt="drawing" width="700"/>

Choose the the start and goal locations by dragging the end effector to the desired position. Animate the robot by pressing _plan_ or _plan and execute_ in the Moveit panel on the bottom left.

<img src="png_images/aubo_i5_demo_launch09.png" alt="drawing" width="700"/> <img src="png_images/aubo_i5_demo_launch09.png" alt="drawing" width="700"/>

Show the intermediate poistions by selecting _Planned Path > Show Trail_, and disable the infinite loop by deselecting _Planned Path > Loop Animation_. The settings are described in more detail in the Moveit [getting started](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) tutorial.

 
