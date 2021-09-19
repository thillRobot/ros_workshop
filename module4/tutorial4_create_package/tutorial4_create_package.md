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
	
-  **STOP: TUTORIAL NOT READY** - This file is being converted at the moment. Check back soon for updates. It is not recommended to use this tutorial in the current state. Use the PDF version while you wait. - TH 

- **Backup the System:** If you are using a virtual machine, it is recommend to make a snaphot of your virtual machine before you start each module. In the event of an untraceable error, you can restore to a previous snapshot. 
		
- **Workspace Setup:** In Part I you will setup a Catkin Workspace as your working directory for creating packages. _This only needs to be done once_.  
	
## Important Note on Naming: 
	
In this tutorial you will replace several <fields> with names of your choice. These are general guidlines for [naming in ROS](http://wiki.ros.org/ROS/Patterns/Conventions).

- use descriptive names, very long or very short names are hard to read
- **do not** include the < > symbols
- **do not** use spaces, UPPER CASE letters, or special characters (@,$,*, etc.)
- the underscore _ character **is** allowed 

    <workspace_name> - name of your workspace 
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


sadfaSDF

ASDFASDF
ASDFASDF


``` 


		\end{minted}
		
		
		\item [Step 5:]  Now add your workspace directory to .bashrc and source the script.
		\begin{minted}{text} 
echo "source ~/|\wspname|/devel/setup.bash" >> ~/.bashrc
		\end{minted}
		
		\begin{minted}{text} 
source ~/.bashrc 
		\end{minted}
		
		\begin{multicols}{2}
		Open the {\bf .bashrc} file with the gedit text editor. You can see the lines you have added with {\bf echo} >> at the bottom of the file. Close the file.  
		\begin{minted}{text}  
gedit |\home|.bashrc
		\end{minted}
		\end{multicols}
		
		%\item [Step 6:]Before continuing test that your ROS system is setup correctly.
		%\begin{minted}{text} 
		%echo |\$|ROS_PACKAGE_PATH
		%\end{minted}
		%
		%You should see something like this in the terminal. This is the path where ROS is installed. Do not enter this as a command.
		%\begin{minted}{text} 
		%/home/<user_name>/|\wspname|/src:/opt/ros/|\rosdistro|/share
		%\end{minted}
		
	\end{description}
	
	\newpage
	\item[\textbf{\underline{Part II - Create A \href{http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c++)}{{\bf Publisher }}Node:}}] \hfill \vspace{0mm}
	   
	   You can write custom nodes for your ROS system in C++, Python, or Lisp. These documents will support C++.
	         \begin{description}    				
	          \item [Step 1:] \href{http://wiki.ros.org/ROS/Tutorials/CreatingPackage}{Create a new package} in your workspace for your new node to belong to. Make sure to do this in the correct parent directory .
	\begin{minted}{text} 
cd |\home\wspname|/src
	\end{minted}
	
	\begin{minted}{text} 
catkin_create_pkg |\pkgname| std_msgs rospy roscpp
	\end{minted}
	            
	            
	\item [Step 2:] Back out to the workspace directory then compile your package with \href{http://wiki.ros.org/catkin/Tutorials/using_a_workspace#Building_Packages_in_a_catkin_Workspace}{catkin\_make} 
	
	\begin{minted}{text}  
cd |\home\wspname| 	OR 	cd ..
	\end{minted}
	             
	\begin{minted}{text}  
catkin_make
	\end{minted}
	            
	
	If you get here with no errors, your workspace is setup, and you are ready to write some code and test your new package!
	            
	            
	            %\end{enumerate}
	\newpage
	\item [Step 3:] Create a new file for your C++ {\bf publisher node} from the command line. The text editor {\it gedit} will create and open a new file named \nodname in the current directory.
	
	%Make sure it is saved in the source directory of the package your created in previously in {\bf step 1}.
	%Write a {\bf publisher node} in C++. It will start as C++ code and then it will be compiled into an executable. Create a  %the sample code shown below. 
	
	\begin{minted}{text}  
gedit |\home\wspname|/src/|\pkgname|/src/|\nodname|.cpp
	\end{minted}
	  
	 Copy the code below into the source file. . \vspace{1mm}
	
	%\begin{minted}{cpp}
	 
	\begin{lstlisting}
	#include "ros/ros.h"
	#include "geometry_msgs/Twist.h"
	#include <sstream>
	
	int main(int argc, char **argv)
	{
	    ros::init(argc, argv, "replace_with_your_node_name");
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
	\end{lstlisting}
	
	%\end{minted}
	
	Save and close the file. It must be saved as a \nodname.cpp in the src directory of the package your created in previously in {\bf Part I}
	
	\item[Step 4:] Before we can compile the node we have to modify the file below.
	
	\begin{minted}{text}  
gedit |\home\wspname|/src/|\pkgname|/CMakeLists.txt
	\end{minted}
	
	Add the following lines to the bottom of the file and save.
	
	\begin{minted}[bgcolor=white]{text}
add_executable(|\nodname| src/|\nodname|.cpp)
target_link_libraries(|\nodname| |\$|{catkin_LIBRARIES}) 
	\end{minted}
	\newpage
	 
	
	\item[Step 5:] Compile and test the new publisher node. This will compile and build your source code as well as check for errors in your entire workspace.
	\begin{minted}{text}  
cd |\home\wspname|
	\end{minted}
	
	\begin{minted}{text}  
catkin_make
	\end{minted}
	
	Start a core
	\begin{minted}{text} 
roscore
	\end{minted}
	
	Turn on a turtle.
	\begin{minted}{text} 
rosrun turtlesim turtlesim_node
	\end{minted}
	
	Start your new node
	\begin{minted}{text} 
rosrun |\pkgname\hspace{3mm}\nodname|
	\end{minted}
	
	Use rostopic to view current topics. 
	\begin{minted}{text}  
rostopic list
	\end{minted}
	
	
	Close your node and start it again with the cmd\textunderscore vel topic mapped to the turtle like we did previously.
	\begin{minted}{text} 
rosrun |\pkgname\hspace{3mm}\nodname \hspace{1mm}|/cmd_vel:=/turtle1/cmd_vel
	\end{minted}
	 
	\end{description}
	 
	 
	\newpage
	
	
\item[\textbf{\underline{Part III - Create A \href{http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c++)}{{\bf Subscriber }}Node:}}] \hfill \vspace{0mm}
	 	
	Now create a {\bf subscriber node} in the same package as the previous node. 
	
	\begin{description}
	
	\item [Step 1:] Use the code below called {\bf turtlesim\_subscriber.cpp} to start.\\
	
	\begin{lstlisting}
	
	#include "ros/ros.h"
	#include "std_msgs/String.h"
	#include "geometry_msgs/Twist.h"
	/**
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
	
	\end{lstlisting}
	
	
	\item [Step 2:] Modify the appropriate CMakeLists.txt file as you did previously. \\\\
	%\begin{minted}{text}
	%gedit |\home\wspname/src/\pkgname/CMakeLists.txt|
	%\end{minted}
	
	\item [Step 3:] Compile the new subscriber node using catkin. \\\\
	
	%\begin{minted}{text} 
	%|cd \home\wspname|
	%\end{minted}
	
	%\begin{minted}{text} 
	%catkin_make
	%\end{minted}
	
	\item [Step 4:] Test the new node. Does it work? How do you know?\\
	
	%\begin{minted}{text} 
	%rosrun |\pkgname\hspace{3mm}\nodname| 
	%\end{minted}
	

	\vspace*{5mm}
	


	\end{description}	
	
	
	

	\item[\textbf{\underline{Tutorial Complete:}}] \hfill \vspace{3mm}\\ After completing {\it Tutorial 4 - Create Package}, you are finally ready for a more advanced robot simulator.
	
	\hfill \vspace{3mm}\\
	
	\item [Bonus Excercise:] Install the \href{http://wiki.ros.org/joy/Tutorials/WritingTeleopNode}{JoyStick Teleop Node} to drive the turtle with a USB joystick.
	
	
	\end{description}

\end{document}

