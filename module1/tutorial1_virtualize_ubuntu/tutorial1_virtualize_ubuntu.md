# ROS Workshop - Tutorial 1 - Virtualize Ubuntu
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics

## Overview

In this tutorial you will first download and install VirtualBox from Oracle which is an application for _virtualizing_ operating systems inside of an existing one. Next, you will download the Ubuntu installation .iso file and setup a virtual operating system for learning ROS.  After completing this exercise, you will be ready to install the ROS Melodic software package in Ubuntu which is described in detailed in the next module.

<img src="images/CaptureA.png" alt="drawing" width="400"/>

## Watch on Youtube 
You can watch this tutorial on [Youtube](https://youtu.be/E7Ga6tbY0Iw).


The remainder of the this tutorial is being repaired and will be volatile until 08/25/2022.


## What is a [Virtual Machine](https://en.wikipedia.org/wiki/Virtual_machine) ?
- A virtual machine is an operating system that is installed or _virtualized_ inside another operating system.
- This is useful for learning and testing, but it is resource intensive and is not ideal for permanent use. 
- [VirtualBox](https://www.virtualbox.org/) is a trusted application from Oracle widely used for this process. [VMware](https://www.vmware.com/) is alternative option for virtualization, but it will not be supported in this course.

## System Requirements
- **CPU:** Most modern notebook or desktop computers will work well. If you are using a very old computer it may be slow. A tablet or Chromebook is not supported.
- **Memory:** At least 8Gb of RAM is recommended.        
- **Storage:** Approximately 25Gb of free space on a hard drive is required. This space will remain in its current partition, and you are free to delete the files later. USB 2.0 or slower connection to the hard drive is not recommended. 
- **GPU:** A dedicated graphics prcocessing unit is **not required**. This process has been tested with Intel embedded graphics.

## Before You Begin
- It is a good idea to back up any important files before you begin a project. Hard disk drives fail. Solid state drives can also fail. 
- Some students may have to adjust a computer BIOS setting to allow virtualization. This setting can be easily reverted after completing the workshop.   
- It is recommended to have your computer's power supply plugged-in before you begin this installation process.

## Detailed Setup Process

### Part 1 - Install VirtualBox Application

- Download the VirtualBox installation file using the link on ilearn. Choose the link that matches your computer type. If you are using a Linux computer already you do not need this tutorial. 
    
- Click the VirtualBox installation file you downloaded and install the application. You may need to provide admistrator access and click allow). 

<img src="images/Capture1_edited.png" alt="drawing" width="500"/> <img src="images/Capture2_edited.png" alt="drawing" width="500"/>
<img src="images/Capture3_edited.png" alt="drawing" width="500"/> <img src="images/Capture4_edited.png" alt="drawing" width="500"/>
<img src="images/Capture5_edited.png" alt="drawing" width="500"/> <img src="images/Capture6_edited.png" alt="drawing" width="500"/>
<img src="images/Capture7_edited.png" alt="drawing" width="500"/> <img src="images/Capture8_edited.png" alt="drawing" width="500"/>

### Part 2 - Virtual Machine Configuration
Before proceeding make sure you have an internet connection and access to a power supply or battery.

#### Open the VirtualBox application installed in Part 2

#### Create New Virtual Machine

<img src="images/Capture9.png" alt="drawing" width="500"/> <img src="images/Capture11.png" alt="drawing" width="500"/>

- Click the **new** button.

- Click **Expert Mode** to view basic settings in single window


#### Define Virtual Machine Virtual Hard Drive Parameters

<img src="images/Capture12.png" alt="drawing" width="500"/>

- choose the **computer name**: this is your choice (0-9, a-z, and hyphens (-) allowed)
- choose the **operating system** type: _Linux_
- choose the **version**: _Ubuntu 64-bit_ 
- choose the amount of RAM you want to allocate to the VM (while running only)  
	- If your computer has 8GB total I suggest no more than 6GB for for the virtual machine. If your computer has 16GB, then 8-10Gb is reccomended for the VM.
- choose **create a virtual hard disk now**
- click **create**


#### Define Virtual Hard Drive Parmeters

 <img src="images/Capture13.png" alt="drawing" width="500"/> <img src="images/Capture14.png" alt="drawing" width="500"/> 

- choose Hard disk file type: _VDI (Virtual Disc Image)_. 
- choose Storrage on physical hard disk: **Fixed Size** virtual hard drive. 
- choose the size of your virtual hard drive       
- select 25 Gb VDI size if available, 20Gb to 15Gb may work 
	- Ubuntu 20.04 minimal: 9Gb
	- ROS-Desktop-Full: 3Gb

#### Complete Virtual Machine and Hard Drive Setup
<img src="images/Capture16.png" alt="drawing" width="500"/>

You can now see the virtual machine you created in the list on the left and the defined settings on the right.

### Part 3 - Virtual Machine Performance Settings

Click _Settings_ to adjust the performance settings the virtual machine that you have created. 

<img src="images/Capture19.png" alt="drawing" width="500"/> <img src="images/Capture22.png" alt="drawing" width="500"/> 

- increase _Processor(s)_ to 4, 6, or more if available in green 

- increase the _Video Memory_ to the maximum value in green

- select _Enable 3D Acceleration_
          

### Part 4 - Ubuntu OS Installation and Setup

<img src="images/Capture24.png" alt="drawing" width="500"/> 

- Choose your new VM and Click _Start_ to turn on the virtual machine. The initial boot is an involved process.

<img src="images/Capture26.png" alt="drawing" width="500"/> <img src="images/Capture27.png" alt="drawing" width="500"/>

- Click the folder icon to open the Optical Disc Selector menu. 

- Click _add_

<img src="images/Capture28.png" alt="drawing" width="500"/> <img src="images/Capture29.png" alt="drawing" width="500"/> 

- Open the file _ubuntu-20.04.4-desktop-amd64_. 

- Click choose to continue

<img src="images/Capture30.png" alt="drawing" width="500"/> <img src="images/Capture31.png" alt="drawing" width="500"/>

- Click start to begin the initial boot of the virtual operating system

<img src="images/Capture32.png" alt="drawing" width="500"/> <img src="images/Capture33.png" alt="drawing" width="500"/> 

<img src="images/Capture36.png" alt="drawing" width="500"/> <img src="images/Capture37.png" alt="drawing" width="500"/> 

<img src="images/Capture38.png" alt="drawing" width="500"/> <img src="images/Capture39.png" alt="drawing" width="500"/> 

<img src="images/Capture40.png" alt="drawing" width="500"/> <img src="images/Capture41.png" alt="drawing" width="500"/> 

<img src="images/Capture42.png" alt="drawing" width="500"/> <img src="images/Capture48.png" alt="drawing" width="500"/>

<img src="images/Capture42.png" alt="drawing" width="500"/> <img src="images/Capture48.png" alt="drawing" width="500"/>


<img src="images/Capture49.png" alt="drawing" width="500"/> <img src="images/Capture50.png" alt="drawing" width="500"/>

<img src="images/Capture51.png" alt="drawing" width="500"/> <img src="images/Capture52.png" alt="drawing" width="500"/>

<img src="images/Capture53.png" alt="drawing" width="500"/> <img src="images/Capture54.png" alt="drawing" width="500"/>

<img src="images/Capture55.png" alt="drawing" width="500"/> <img src="images/Capture56.png" alt="drawing" width="500"/>

<img src="images/Capture57.png" alt="drawing" width="500"/> <img src="images/Capture58.png" alt="drawing" width="500"/>

<img src="images/Capture59.png" alt="drawing" width="500"/> <img src="images/Capture60.png" alt="drawing" width="500"/>

<img src="images/Capture61.png" alt="drawing" width="500"/> <img src="images/Capture62.png" alt="drawing" width="500"/>

<img src="images/Capture70.png" alt="drawing" width="500"/> <img src="images/Capture73.png" alt="drawing" width="500"/> 




<img src="Capture10.png" alt="drawing" width="1000"/>

- choose to select media from a local folder

#### Choose the installation media
<img src="Capture11.png" alt="drawing" width="1000"/>

- choose the Ubuntu .iso file that you downloaded.
- it is recommended that the media is on the local machine, and not a remote or cloud drive
- click **open**

#### Boot Ubuntu Installation Image
<img src="Capture12.png" alt="drawing" width="1000"/>

- click **start**

#### Ubuntu Installation
<img src="Capture13.png" alt="drawing" width="1000"/>

- click **Install Ubuntu** - This is **harmless** _if using VirtualBox_
- **try** is for temporary or _live_ session which can be used for computer maintence 

#### Ubuntu Installation (Continued)
<img src="Capture15.png" alt="drawing" width="1000"/>

- choose **Minimal Installation**
- choose **Download Updates**
- choose **Install third-party software**  
- click **continue**

#### Ubuntu Installation (Continued)
<img src="Capture16.png" alt="drawing" width="1000"/>

- choose **Erase Everything and Install Ubuntu** 
- this is _HARMLESS IF INSIDE VIRTUALBOX_
- this could results in _PERMANENT DATA LOSS IF NOT IN VIRTUALBOX_
- click **Install Now**

#### Confirm Hard drive partitioning
<img src="Capture17.png" alt="drawing" width="1000"/>

- you are confirming to partition your virtual hard drive 
- this will not affect your files outside of VirtualBox
- click **continue**

#### Choose the correct timezone
<img src="Capture18.png" alt="drawing" width="1000"/>

#### Define you user settings
<img src="Capture19.png" alt="drawing" width="1000"/>

- choose a simple user name and computer name
- choose a simple password or leave it blank
- click **continue**

#### Wait for installation to complete
<img src="Capture20.png" alt="drawing" width="1000"/>

- Make sure you are plugged into a power source or have a good battery
- This will take several minutes depending on your system and network connection. _Be patient_, you are almost done!

#### Restart the VM
<img src="Capture21.png" alt="drawing" width="1000"/>

- click **restart now**

#### Restart to complete installation
<img src="Capture22.png" alt="drawing" width="1000"/>

- The installation of Ubuntu is now complete. Press the {\bf enter} key to shut down the machine. 
- If it does not shut down click {\bf Machine $\rightarrow $ ACPI Shutdown}.

#### Finally! Installation Complete
<img src="CaptureY.png" alt="drawing" width="1000"/>

- You should now see your new virtual operating system in the list on the left. 
- Click the **start** button to turn it on. 

#### Ubuntu Login Screen
<img src="Capture23.png" alt="drawing" width="1000"/>

- You should see the username you chose 
- Login with the credential you created previously.

#### Ubuntu Welcome
<img src="Capture24.png" alt="drawing" width="1000"/>

- the default selections are fine
- click **next**

#### Fresh New Ubuntu - Focal Fossa
<img src="Capture25.png" alt="drawing" width="1000"/>

- Install the updates. Do not upgrade to Ubuntu 20 unless you want fly solo.
- Updating requires the internet, but you knew that already.

#### Make a Backup

- Now, it is a good idea to make a **backup** of your fresh install. VirtualBox can do this for you but you have to shut it down first.  
- Find the **shutdown** button in Ubuntu, or open a terminal and type `sudo shutdown`. You can also use the _ACPI shutdown_ button in VirtualBox. An unexpected shutdown should not hurt the system unless it is updating at the time, and if that happens it can usually repair itself. This snaphot will save you alot of time, in the event you have to restart.


<img src="images/Capture74.png" alt="drawing" width="500"/> <img src="images/Capture75.png" alt="drawing" width="500"/> 

<img src="images/Capture76.png" alt="drawing" width="500"/> <img src="images/Capture77.png" alt="drawing" width="500"/> 

<img src="images/Capture78.png" alt="drawing" width="500"/> <img src="images/Capture79.png" alt="drawing" width="500"/> 

<img src="images/Capture80.png" alt="drawing" width="500"/> <img src="images/Capture81.png" alt="drawing" width="500"/> 


#### Back to VirtualBox
<img src="CaptureZ.png" alt="drawing" width="1000"/>

- Select your VM so that it is highlighted blue
- click **Machine->Tools->Snapshots**             

#### Take a Snapshot of the VM for Backup

- Snapshots can be used as a backup. This will save you all those steps if you if you ever need to start over.  
- click **Take** to save a snapshot of the current state of your virtual machine. Whew... that was a lot of steps. 
- **You did it!** _Welcome to the world of Linux. Have fun!_


