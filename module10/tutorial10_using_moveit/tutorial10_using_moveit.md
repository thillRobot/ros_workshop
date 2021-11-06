# using moveit

This is my notes for using moveit with ROS Melodic. I can't quite call it a tutorial yet.

## Resources

Read the docs! It looks like they are in two places...

 - [moveit](https://moveit.ros.org/)

 - [moveit_tutorials](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) 


## Installation

It is assumed that [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) is installed.

Source the setup file before begin

```
source /opt/ros/melodic/setup.bash
```


Follow the instructions [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin) to install `moveit`. The commands are copied here for convenience. 

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

Create a workspace for moveit

```
mkdir -p ~/ws_moveit/src
```

Download the moveit_tutorials package into the workspace.
```
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b $ROS_DISTRO-devel
```


Prepare the workspace and compile with `catkin build`. 

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

Download the aubo_robot package into a different workspace so that it can be loaded by the `moveit setup assistant`. It will not compile, so it should not be in `ws_moveit`.

```
mkdir -p ~/ws_aubo/src
cd ~/ws_aubo
catkin build 

git clone https://github.com/AuboRobot/aubo_robot.git -b $ROS_DISTRO src/aubo_robot

source ~/ws_aubo/devel/setup.bash # just for now
cd ~ 
```



Run the `moveit setup assistant` from the tutorial [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) to generate a Gazebo compatible URDF from the URDF in the aubo package. This sounds promising.

```
roslaunch moveit_setup_assistant setup_assistant.launch
```

I used the the file `/aubo_robot/aubo_description/urdf/aubo_i5.urdf` to generate the urdf `aubo_i5_gazebo.urdf` and a package named `aubo_i5_moveit_config`


Create a directory in the new package to put the gazebo urdf in

```

mkdir ~/ws_moveit/src/aubo_i5_moveit_config/gazebo
vim ~/ws_moveit/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf
```


Paste in the urdf XML from the clipboard (lol) 


compile the package with catkin build

```
cd ~/ws_moveit
catkin build

source ~/ws_moveit/devel/setup.bash
```



Now start the Gazebo simulator with an empty world

```
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
```

Add the robot to the simulator using the urdf for gazebo. 


```
rosrun gazebo_ros spawn_model -file $(find aubo_i5_moveit_config)/gazebo/aubo_i5_gazebo.urdf -urdf -x 0 -y 0 -z 1 -model aubo_i5
```

Find did not work, lets try that again later. Use the full path for now.

```
rosrun gazebo_ros spawn_model -file ws_moveit/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf -urdf -x 0 -y 0 -z 1 -model aubo_i5
    
    [INFO] [1636155628.986157]: Loading model XML from file ws_moveit/src/aubo_i5_moveit_config/gazebo/aubo_i5_gazebo.urdf
    [INFO] [1636155628.992780]: Waiting for service /gazebo/spawn_urdf_model
    [INFO] [1636155628.997679]: Calling service /gazebo/spawn_urdf_model
    [INFO] [1636155629.301420]: Spawn status: SpawnModel: Successfully spawned entity
```

Well it looks like it worked. Woop Woop!

<img src="png_images/aubo_i5_gazebo.png" alt="drawing" width="700"/>



If the `ws_aubo` workspace is not sourced the error below is shown. This should be fixed.


```
[rospack] Error: package 'aubo_description' not found
[librospack]: error while executing command
[FATAL] [1636155476.819294674]: Package[aubo_description] does not have a path
^C[gazebo_gui-3] killing on exit
```

I do not really like have both workspaces sourced. So I copied the missing package from ws_aubo to ws_moveit

```
mkdir -p ~/ws_moveit/src/moveit_examples/aubo_robot
cp -r ~/ws_aubo/src/aubo_robot/aubo_description ~/ws_moveit/src/moveit_examples/aubo_description

cd ~/ws_moveit
catkin build
```

Now close both terminals and do not source the aubo workspace.

The `aubo_i5_moveit_config` package contains a collection of launch files. This should be very useful.

```
roslaunch aubo_i5_moveit_config demo.launch
```


<img src="png_images/aubo_i5_demo_launch01.png" alt="drawing" width="700"/>

 



 
 
Download the `thillRobot/moveit_examples` package for pre-configured robot examples
```
git clone https://github.com/thillRobot/moveit_examples.git
```
 
 
 
