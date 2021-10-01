# ROS Workshop - Turtorial 6 - Turtlebot3 Navigation
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics 

## Navigation:

What do we mean by navigation? This means different things in different places. Here, we mean the navigation stack in ROS melodic. This tutorial comes from [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation) 

<img src="turtlebot3_models.png" alt="drawing" width="400"/>

(Image: [emanual.robotis.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#features) )

	
## Overview:
After completing _Tutorial 5 - Turtlebot Simulator_, You have learned some basics of ROS, and you have a more advanced robot. Next you are going to learn to use the navigation stack with the turtlebot3 simulator. Read more [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#ros-1-navigation}{here} and \href{http://wiki.ros.org/navigation/Tutorials}).
	
## System Requirements:

- **ROS+OS**: This tutorial is intended for a system with ROS Melodic installed on the Ubuntu 18.04 LTS operating system. Alternate versions of ROS (i.e. - Kinetic, Noetic, etc.) may work but have not been tested. Versions of ROS are tied to versions of Ubuntu.
- **ROS**: Your computer must be connected to the internet to proceed. Update the system before you begin.
- **Workspace Setup**:} The Turtlebot3 Simulator from tutorial 5 must be operational before completing tutorial 6.  

	
## Before Beginning:
	
- **Backup the System:** If you are using a virtual machine, it is recommend to make a snaphot of your virtual machine before you start each module. In the event of an untraceable error, you can restore to a previous snapshot. 

- **ROSLAUNCH:** This tutorial involves using the roslaunch command which runs a muliple of nodes at once as described in the launch file. We will learn more about this later. 

- **Mouse for 3D viewing:** This simulator view is much easier use if you have a three button mouse plugged in, but this is not required.
	
		
## Part 1 - Install navigation and gmapping packges
	

### Step 1 Install `navigation` and `gmapping` 
Install the packages required for autonomous navigation if you have not already. It will not hurt to run the install command again.

```
sudo apt install ros-melodic-navigation ros-melodic-gmapping
```

### Step 2 -  Set the robot model. This only needs to be done once. Modify the .bashrc file If you want to change models.

```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

## Part 2 - Generate a map of the virtual space:



### Step 1 Start the turtlebot3 simulator

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Step 2 Start gmapping SLAM
 
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Step 3 Collect sensor data

Drive the robot around with the keyboard to collect pose and Lidar data

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Step 4 Save Map

When you are finished collecting data save the map. (`-f` allows the filename to be set)
 
```
rosrun map_server map_saver -f map
```

Two new map files both with the same filename (<map_name>.pgm and <map_name>.yaml) will appear in the current folder after Step 4. If you move the map to a new directory, keep both files together and the file names must match.




%    \item Next install the physical 'turtlebot' drivers into your ROS system. This step is only necessary if you are using a real turtlebot. \href{http://wiki.ros.org/Robots/TurtleBot} {Link Here} 
%   \begin{minted}{text}  
%(sudo apt-get install ros-|\rosdistro|-turtlebot ros-|\rosdistro|-turtlebot-apps
%ros-|\rosdistro|-turtlebot-interactions ros-|\rosdistro|-turtlebot-simulator 
%ros-|\rosdistro|-kobuki-ftdi ros-|\rosdistro|-rocon-remocon 
%ros-|\rosdistro|-rocon-qt-library ros-|\rosdistro|-ar-track-alvar-msgs})
%\end{minted}
%    
\newpage
\item[\textbf{\underline{Part 3 - Navigate the virtual space using the map and RVIZ:}}] \hfill \vspace{2mm}	\\
Now that navigation is installed and there is a map saved to file, the robot can perform\\ autonomous point to point navigation with dynamic obstacle avoidance. \\
\begin{enumerate}
\item {\bf   Start the turtlebot3 simulator.}
\begin{minted}{text} 
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
\end{minted}
\item {\bf  Turn on the navigation nodes and RVIZ. Use the name of the map you created. }
\begin{minted}{text} 
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=map.yaml
\end{minted}

	The gazebo window will open containing your robot, and you will also see the rviz window open separately. Find and test the following features of navigation in RVIZ. \\
	
	1) \underline{Pose Estimate} - Click and drag the direction to set the current pose estimate of the robot. \vspace{2mm} \\
	
	2) \underline{2D Nav Goal} - Click and drag the direction to define a goal point for the robot in the map. \vspace{2mm} \\

\includegraphics[scale=.350]{turtlebot3_rviz.png}



\item {\bf Check the available topics with rostopic. }
\begin{minted}{text} 
rostopic list
\end{minted}
\begin{minted}{text} 
rosrun rqt_graph rqt_graph 
\end{minted}

\end{enumerate}

\newpage
\item[\textbf{\underline{Part 4 - Issues Loading Maps Files:}}] \hfill \vspace{2mm}	\\

You may have run into an issue in which {\it turtlebot3\_navigation} cannot load the map file. A typical error message is shown below, along with the preferred solution.


\color{red}
\begin{minted}{text} 

[ERROR] [1604667817.760623311]: Map server could not open /map.yaml.

\end{minted}
\color{black}

\begin{itemize}

\item Leave the top line of {\it map.yaml } as is when the file is generated. Do not add the full path to map file. The filename is sufficient. 

\item Copy {\it map.yaml } and {\it map.pgm } to the maps/ directory of a package in the ROS workspace.

\item Ensure that the workspace will compile and build with {\it catkin\_make}. You should see in the terminal output, that your package was successfully built. 

\item Finally, use the {\it find} command to point the node to the maps directory of your package.

\end{itemize}

Now repeat the commands from before with the addition of the find command in the map argument. 

\begin{enumerate}


\item {\bf   Start the turtlebot3 simulator.}
\begin{minted}{text} 
  roslaunch turtlebot3_gazebo turtlebot3_world.launch
\end{minted}
\item {\bf  Turn on the navigation nodes and RVIZ. Use the name of the map you created. }

\begin{framed}
\begin{verbatim}

roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
map_file:='$(find <YOUR PKG>)\maps\<YOUR MAP>.yaml'

\end{verbatim}
\end{framed}




\end{enumerate}

\vspace{100mm}

\item[\textbf{\underline{Tutorial Complete:}}] \hfill \vspace{3mm}\\ 
	After completing {\it Tutorial 6 - Turtlebot3 Simulator}, you are ready to learn about ... more ROS!
	
	\hspace {30mm}	
	
	
	
	\includegraphics[scale=.350]{../../charlie_robot.jpeg} \vspace{5mm}\\
    \LARGE \textbf{NOW, YOU KNOW ABOUT ROS! GOOD JOB!}
	
	
		
