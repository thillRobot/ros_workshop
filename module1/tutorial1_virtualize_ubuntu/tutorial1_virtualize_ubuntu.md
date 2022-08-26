# ROS Workshop - Tutorial 1 - Virtualize Ubuntu
## ME4140 - Introduction to Robotics, ME6640 - Advanced Robotics

## Overview
In this tutorial you will first download and install VirtualBox from Oracle which is an application for _virtualizing_ operating systems inside of an existing one. Next, you will download the Ubuntu installation .iso file and setup a virtual operating system for learning ROS.  After completing this exercise, you will be ready to install the ROS Melodic software package in Ubuntu which is described in detailed in the next module.

<img src="images/CaptureA.png" alt="drawing" width="400"/>

## Watch on Youtube 
You can watch this tutorial on [Youtube](https://youtu.be/E7Ga6tbY0Iw).


The remainder of the this tutorial is being repaired and will be volatile until 08/25/2022.

## What is a [Virtual Machine](https://en.wikipedia.org/wiki/Virtual_machine)
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

<img src="images/Capture19.png" alt="drawing" width="500"/> <img src="images/Capture22.png" alt="drawing" width="500"/> 

Click _Settings_ to adjust the performance settings the virtual machine that you have created. 
 
- increase _Processor(s)_ to 4, 6, or more if available in green 

- increase the _Video Memory_ to the maximum value in green

- select _Enable 3D Acceleration_
          

### Part 4 - Ubuntu OS Installation and Setup

<img src="images/Capture24.png" alt="drawing" width="500"/> 

- Choose your new VM and Click _Start_ to turn on the virtual machine. The initial boot may take 15 to 30 minutes.


#### Choose the installation media
<img src="images/Capture26.png" alt="drawing" width="500"/> <img src="images/Capture27.png" alt="drawing" width="500"/>

- Click the folder icon to open the Optical Disc Selector menu. 

- Click **add**

<img src="images/Capture28.png" alt="drawing" width="500"/> <img src="images/Capture29.png" alt="drawing" width="500"/> 

- Open the file _ubuntu-20.04.4-desktop-amd64_. 

- Click **choose** to continue

#### Boot Ubuntu Installation Image
<img src="images/Capture30.png" alt="drawing" width="500"/> <img src="images/Capture31.png" alt="drawing" width="500"/>

- Click **start** to begin the initial boot of the virtual operating system


#### Ubuntu Installation
<img src="images/Capture32.png" alt="drawing" width="500"/> <img src="images/Capture33.png" alt="drawing" width="500"/> 

- click **Install Ubuntu**
- **try** is for temporary or _live_ session and can be used for computer maintence 

<img src="images/Capture36.png" alt="drawing" width="500"/> <img src="images/Capture37.png" alt="drawing" width="500"/> 

- choose **Minimal Installation**
- choose **Download updates while installing Ubuntu**
- choose **Install third-party software...**  
- click **continue**

- choose **Erase Everything and Install Ubuntu**, this is _HARMLESS_ because we are inside a virtual machine
- click **Install Now**,this will not affect your files outside of VirtualBox

#### Confirm Hard drive partitioning
<img src="images/Capture38.png" alt="drawing" width="500"/> 

- click **continue** to confirm partitioning your virtual hard drive


#### Choose User Account Settings 
<img src="images/Capture40.png" alt="drawing" width="500"/> <img src="images/Capture41.png" alt="drawing" width="500"/> 

- Choose the correct timezone

- choose _Your name_ and _Your computer's name_ (0-9, a-z, and hyphens (-) allowed)
- choose a simple _username_ and _password_ that you can remember
- select _Require my password to log in_
- unselect _Log in automatically_ and _Use Active Directory_ if previously selected
- click **continue** 

#### Complete Installation
<img src="images/Capture42.png" alt="drawing" width="500"/> <img src="images/Capture48.png" alt="drawing" width="500"/>

- Make sure you are plugged into a power source or have a good battery
- This step may take 15-20 minutes depending on your system and network connection. _Be patient_, you are almost done!
- When prompted, click _Restart Now_

#### Restart Virtual Machine 
<img src="images/Capture49.png" alt="drawing" width="500"/> <img src="images/Capture50.png" alt="drawing" width="500"/>

- Press **enter** to restart the virtual machine

<img src="images/Capture51.png" alt="drawing" width="500"/> <img src="images/Capture52.png" alt="drawing" width="500"/>

- You should see the username you chose 
- Login with the password you created previously

#### Ubuntu Welcome Screen
<img src="images/Capture53.png" alt="drawing" width="500"/> <img src="images/Capture54.png" alt="drawing" width="500"/>

- click **Skip**
- click **Next**

<img src="images/Capture55.png" alt="drawing" width="500"/> <img src="images/Capture56.png" alt="drawing" width="500"/>

- if prompted **Don't Upgrade** to Ubuntu 22.04
- click OK

<img src="images/Capture57.png" alt="drawing" width="500"/> <img src="images/Capture58.png" alt="drawing" width="500"/>

- click **next**
- choose to send info or not and click **next**

<img src="images/Capture59.png" alt="drawing" width="500"/> <img src="images/Capture60.png" alt="drawing" width="500"/>

- leave **Location Services** unchecked 
- click **Next**
- click **Done**

#### System Update
<img src="images/Capture61.png" alt="drawing" width="500"/> <img src="images/Capture62.png" alt="drawing" width="500"/>

- when prompted by Software Updater, click **Install Now**
- if not prompted press the Windows key and type _Software Updater_, open the app and install the updates

<img src="images/Capture70.png" alt="drawing" width="500"/> <img src="images/Capture73.png" alt="drawing" width="500"/> 

- click **Restart Now** to restart the virtual machine to complete the process

- If the machine does not shut down automatically find the **Power Off** button in Ubuntu in the upper right and restart

- You can also use the _ACPI shutdown_ button in VirtualBox. An unexpected shutdown should not hurt the system unless it is updating at the time, and if that happens it can usually repair itself. 

<img src="images/Capture74.png" alt="drawing" width="500"/> <img src="images/Capture75.png" alt="drawing" width="500"/> 

<img src="images/Capture76.png" alt="drawing" width="500"/> 


#### Take a Snapshot of the VM for Backup
  
<img src="images/Capture77.png" alt="drawing" width="500"/> <img src="images/Capture78.png" alt="drawing" width="500"/> 

- click **Machine->Tools->Snapshots**  
- Select your VM so that it is highlighted blue
- Click **Take** to save the current state of the virtual machine 
- This snaphot will save you alot of time, in the event you have to restart
- This may not work if your physical storage device is full 

<img src="images/Capture80.png" alt="drawing" width="500"/> <img src="images/Capture81.png" alt="drawing" width="500"/> 

- Choose a name and write a brief description of the snapshot 
           
## Tutorial 1 - Virtualize Ubuntu 20.04 - Complete 

Great Job. You have completed the tutorial.