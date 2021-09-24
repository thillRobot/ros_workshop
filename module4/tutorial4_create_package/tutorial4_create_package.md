# ROS Workshop - Tutorial 4 - Create Custom Package
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

## Overview:
After completing _Tutorial 3 - Turtlesim_  You have begun learning ROS and you are ready to create a custom C++ package. You can read more [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on the wiki.

## System Requirements:

- **ROS+OS:** This tutorial is intended for a system with ROS Melodic installed on the Ubuntu 18.04 LTS operating system. Alternate versions of ROS (i.e. - Kinetic, Noetic, etc.) may work but have not been tested. Versions of ROS are tied to versions of Ubuntu.
- **Internet:** Your computer must be connected to the internet to proceed. 
- **Ubuntu Updates:** Update the system before you begin the tutorial. This can be done with the _Software Updater_ found in the _Launcher_ or the following command. This will update the list of available packages and apply any available upgrades to the previously installed packages.  

```
sudo apt update && sudo apt upgrade
``` 
	
## Before You Begin:
	
-  **TUTORIAL READY** - This file has just been converted, and it should be working. Minor edits will be made as it is tested.  

- **Backup the System:** If you are using a virtual machine, it is recommend to make a snaphot of your virtual machine before you start each module. In the event of an untraceable error, you can restore to a previous snapshot. 
		
- **Workspace Setup:** In Part I you will setup a Catkin Workspace as your working directory for creating packages. _This only needs to be done once_.  
	
## Important Note on Naming: 
	
In this tutorial you will replace several <fields> with names of your choice. These are general guidlines for [naming in ROS](http://wiki.ros.org/ROS/Patterns/Conventions).

- use descriptive names, very long or very short names are hard to read
- **do not** include the < > symbols
- **do not** use spaces, UPPER CASE letters, or special characters (@,$,*, etc.)
- the underscore _ character **is** allowed 

    <workspace_name> - name of your workspace (default: `catkin_ws` will be used in this tutorial)
    <package_name> - name of your package  
    <node_name> - name of your node  
	<user_name> - ubuntu user name 	

## Instructions for Creating a Custom Package and Node

### Part I Setup the [Workspace:](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) ( Part I only needs to be done once Fall2021. )  

In Part I you will setup a {\it catkin workspace} as the working directory for your ROS system. Catkin is the program that manages your custom nodes and compiles your .cpp source code into executable programs. 
	

#### Step 1: Source the installation files
This runs a script `setup.bash` needed to use ROS (this line should already be in `~/.bashrc`).
```
source /opt/ros/melodic/setup.bash
```

#### Step 2: Navigate Parent Directory
Open a new terminal and navigate to the future location of your workspace. It is reccomended to choose `/~` as the directory location.  
```
cd /home/$USER      # Note: the variable $USER contains your user name
```
Alternatively you can use the command below. This is a shorthand for the command above.

```		
cd ~ 				# this is just a shortcut					
```
Look carefully and you can see that the current directory of the terminal session has changed. Use `ls` to list the directoy contents.

#### Step 3: Choose Name and Create Workspace Directory
Choose a workspace name and create a workspace and source directory with `mkdir`. Follow the naming rules described above when choosing a workspace name. The default name `catkin_ws` is commonly used, and will be used as the <workspace_name> in this tutorial. The `catkin_ws/src` folder is where custom ROS packages are stored. (Note: You can add a package from a friend or from Github by copying it into this directory and building.) 

```
mkdir -p catkin_ws/src
```

#### Step 4: Build the Workspace 
Navigate to the top of your workspace directory (`~/catkin_ws`) and build your workspace with `catkin_make`. This will configure your workspace directory, and compile any source code that is ready to be built. Your workspace has no source code yet so nothing will be compiled.
```
cd catkin_ws
```
Now, the terminal is in the top directory of the ROS workspace. Build the workspace with the `catkin_make` this may take several minutes depending on your system.

```
catkin_make
```

The terminal output should look similar to what is shown below. 

```
Base path: /home/******/catkin_ws
Source space: /home/******/catkin_ws/src

 ... 

####
#### Running command: "cmake /home/******/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/******/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/******/catkin_ws/install -G Unix Makefiles" in "/home/******/catkin_ws/build"
####
-- The C compiler identification is GNU 7.5.0
-- The CXX compiler identification is GNU 7.5.0
-- Check for working C compiler: /usr/bin/cc

 ...

-- catkin 0.7.29
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- Configuring done
-- Generating done
-- Build files have been written to: /home/******/catkin_ws/build
####
#### Running command: "make -j16 -l16" in "/home/******/catkin_ws/build"
####
``` 

#### Step 5: Add workspace directory to `.bashrc` and source the script

The command below appends the line `source ~/catkin_ws/devel/setup.bash` to the file `.bashrc` which contains terminal configuration commands. 

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

The `.bashrc` script runs automatically each time you open a new terminal, so ROS will now look for custom packages in the directory `~/catkin_ws/devel/setup.bash`. For the changes to take affect close and re-open a terminal, or `source` the script manually with the following commad. 		

```		 
source ~/.bashrc 
```		
		
Open the `.bashrc` file with the gedit text editor (or use `vim`). You can see the lines you have added with `echo` at the bottom of the file. Verify that the paths are correct. Make any neccessary edits, and save then source file if it has changed. This file is used to customize the user's terminal session environment.   

```
gedit ~/.bashrc
```

		
#### Step 6: Test ROS system before continuing 
		
```		
echo $ROS_PACKAGE_PATH
```		

You should see something like this in the terminal. This is the path where ROS is installed. Do not enter this as a command.
```
%/home/<user_name>/catkin_ws/src:/opt/ros/<ros_distro>/share
```

### Part II - Create A [Publisher Node](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c++)) 
	   
In Part II you will create, compile, and test a custom node written in C++ to control the turtle simulator. ROS supports custom nodes written in C++, Python, or Lisp, but these documents will primarily support C++. 

#### Step 1: [Create a new package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) in your workspace.

Navigate to the source directory of the ROS workspace. Custom packages are stored here.

```
cd ~/catkin_ws/src
```
Initialize a new package with the `catkin_create_pkg` command. Choose the `<package_name>` and dependecies `std_msgs`, `rospy`, and `roscpp`. More dependecies can be added later. The name should not inlcude capital letters or special characters. See the naming rules discussed above for more information.  	

```
catkin_create_pkg <pkg_name> std_msgs rospy roscpp
```
	            
	            
#### Step 2: Compile your package with [catkin_make](http://wiki.ros.org/catkin/Tutorials/using_a_workspace#Building_Packages_in_a_catkin_Workspace) 

Back out to the top of the workspace directory then compile using the `catkin_make` command. This step is not required until later, but it should verify that you completed the previous parts correctly. 	

```  
cd ~/catkin_ws 	OR 	cd ..
```

```
catkin_make
```	            

Look at the output to see that the workspace has compiled again including the custom package. If you get here with no errors, the ROS workspace is setup and ready for the C++ code. 
	            
	            

#### Step 3: Create a new file for your C++ {\bf publisher node}
Use _gedit_ from the command line to create and open a new file named `<node_name>` in the directory shown.

	
``` 
gedit ~/catkin_ws/src/<package_name>/src/<node_name>.cpp
```

Copy the C++ ode below into the source file. This script will publish a topic similar that will be subscribed to by the turtlesim simulator node. Inside the while loop the linear velocity command in increased incrementally which should case the turtlesim to move in a spiral pattern. 
	
```c++	 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "<node_name>");
    ros::NodeHandle n;
    ros::Publisher ttu_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 2+0.01*count;
        msg.angular.z = 2;
        ttu_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}
```
	
Save the file as a <node_name>.cpp in the src directory of the package your created in previously in {\bf Part I}.The sample code shown below. 

#### Step 4: Configure `CMakeLists.txt` and compile node
	
Open the `CMakeList.txt` file with the text editor. This file contains configuration commands related to the custom package `<package_name>` as indicated by the preceding directort location `~/catkin_ws/src/<package_name>/` .

```
gedit ~/catkin_ws/src/<package_name>/CMakeLists.txt
```

Add the following two lines to the bottom of the file and save. This file will not be used again in this exercise so it can be closed. Each additional node added to the workspace requires both an `add_executable` and a `target_link_libraries` line in the package `CMakeLists.txt` file. These commands configure the workspace to compile the c++ source code added above.
	
```
add_executable(<node_name> src/<node_name>.cpp)
target_link_libraries(<node_name> ${catkin_LIBRARIES}) 
```

The C++ code must be compiled before the node can be run. The `catkin_make` command will compile and build your source code as well as check for errors throughout the ROS workspace. Remember to navigate to the workspace directory before compiling. 
```
cd ~/catkin_ws
```

```
catkin_make
```

If the workspace compiled correctly, the output will look similar to Step 2 with additional entries indicating custom node has been succefully added to the workspace. If there are errors in the workspace configuration or C++ code, an error message will be shown in the output. 

The `catkin_make` command must be run from the top of the workspace directory each time the C++ code is edited for the changes to take affect. This is not the case with Python based nodes because Python is an _interpretive_ language like MATLAB meaning the code is run one line at a time, instead of compiled as a whole before execution. 

#### Step 5: Test the new publisher node

Start ROS with the `roscore` command. 
``` 
roscore
```

Open a new terminal or terminal tab and start a turtle simulator node.

```	
rosrun turtlesim turtlesim_node
```

Open a third terminal or terminal tab and start your custom publisher node.
```
rosrun <package_name> <node_name>
```
	
Use rostopic to view the available topics. 
```  
rostopic list
```	

Stop the publisher node and start it again with the `cmd_vel` topic mapped to `turtle1/cmd_vel` as in the previous tutorial.

```
rosrun <package_name> <node_name> cmd_vel:=/turtle1/cmd_vel
```

If the turtlesim node subscribes to the published topic, the turtle will begin to move in a spiral motion as the velocity is published and updated in the while loop. Close the terminals or use the `ctrl-c` to stop the individual processes when you are done testing. 

### Part III - Create A [Subscriber Node](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c++))
	 	
Next, create a subscriber node in the same package as the previous node. 
	

#### Step 1: Use the code below called `turtlesim_subscriber.cpp` to start.
	
```	
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
/*
* This tutorial demonstrates simple receipt of messages over the ROS system.
*/
void dataCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("I heard: [%f]", msg->linear.x);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlesim_subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, dataCallback);
	ros::spin();
	return 0;
}
```	
	
#### Step 2: Modify the CMakeLists.txt and compile 

Remeber, each C++ node in the package requires an entry in the appropriate `CMakeList.txt` file.

```
gedit ~/catkin_ws/src/<package_name>/CMakeLists.txt
```

Add the following lines to the file as done previously. This time <node_name> should refer to the new subscriber node.

```
add_executable(<node_name> src/<node_name>.cpp)
target_link_libraries(<node_name> ${catkin_LIBRARIES}) 
```

Change to the top of the ROS workspace and compile. 	
```
cd ~/catkin_ws
```
	
```
catkin_make
```
Addtional entries in the output should be shown associated with the changes to the workspace. 

	
#### Step 4: Test the new node. 


Repeat the process used in Part II to start the turtle simulator and publisher node. Does the subscriber node recieve the intended topic? How do you know?\\

Start ROS with the `roscore` command. 
``` 
roscore
```

Open a new terminal or terminal tab and start a turtle simulator node.

```	
rosrun turtlesim turtlesim_node
```

Open a third terminal or terminal tab and start your custom publisher node.
```
rosrun <package_name> <node_name>

```
	
	
## Tutorial Complete: After completing _Tutorial 4 - Create Package_, you are ready for a more advanced robot simulator.
	

### Bonus Excercise: Install the [Joystick Teleop Node](http://wiki.ros.org/joy/Tutorials/WritingTeleopNode) to drive the turtle with a USB joystick.
	
	


