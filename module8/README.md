# Module 8 - Turtlebot3 Setup 

The main objective is to get the turtlebot3 robots working with ros navigation for ME4440. We are following the turtlebot3 manual here. (https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup). 

### Hardware
  * turtlebot3 waffle pi full robot kit (x2)
    * Rasp Pi 3B+ - (ROBOT) 
    * openCR 1.0 - (MCU)
    * RPLidar A1 - (SENSOR)
  * Laptop for ME4444 - (REMOTE COMPUTER #1)
  * Lenovo ThinkCentreM73 i3 - (REMOTE COMPUTER #2) - note: on loan from ME Robotics Lab

### Required Software
  * Mate 18.04 LTS - (ROBOT COMPUTER) 
  * Ubuntu 18.04 LTS - (REMOTE COMPUTER) 
  * ROS Melodic
  * OpenSSH
  * pi-imager
  
### Turtlebot3 Setup Overview
  #### (6.1 - PC Setup) REMOTE COMPUTER Software Installation 
  #### (6.2 - SBC Setup) ROBOT COMPUTER Software Installation  
  #### (6.3 - OpenCR Setup) Embedded Controller Firmware Update 
  #### (6.4 Hardware Setup)
  #### (6.5 Compatible Devices)

### (6.1 - PC Setup) REMOTE COMPUTER Software Installation 
The control computer requires the same OS version as the robot computer, but the flavor can be different. 

#### i) Download Ubuntu 
Ubuntu 18.04.5 LTS Desktop 64bit image (https://wiki.ubuntu.com/Releases?_ga=2.126560777.568362595.1604554678-2034758377.1604554678)
#### ii) Create Bootable USB 
Use `Startup Disk Creator` or `Rufus` to make bootable USB disk with image from step 1. 
#### iii) Install Ubuntu 
Use boot disk from step 2 to install the Ubuntu operating system. Setup a user account, and update while installing. 
#### iv) Install ROS-Melodic
Follow the instructions on the ROS wiki here (http://wiki.ros.org/melodic/Installation/Ubuntu) OR see detailed steps below.

### (6.2 - SBC Setup) ROBOT COMPUTER Software Installation
These steps come from here:(https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup).
**NOTE:** Using the Mate Desktop is not reccomended. Ubuntu 18.04 on the RasPi 3B+ tends to run out of memory. I reccomend using the terminal (ctrl+alt+f1 before loggin in), and in the future we will look into using a RaspPI 4 which claims to have increased memory.

#### i) Install MATE 18.04 on Rasp Pi -
Since this is a rasp pi we are not really installing the OS on the pi. Instead we are copying and image onto the pi.
* download Mate 18.04 64bit image for pi 3B + from Ubuntu Mate website (https://ubuntu-mate.org/download/arm64/)
  this is currently not available, get it here instead (https://www.dropbox.com/sh/sl7p8ccff6ofv3n/AAB6gt3jbKkXziI_uIC-9wS6a?dl=0)
  - current image file(compressed): `ubuntu-mate-18.04.2-beta1-desktop-arm64+raspi3-ext4.img.xz`
* download and/or install `pi-imager` (https://www.raspberrypi.org/downloads/)
* use `pi-imager` to load the image to SD card
* use GUI to setup user accounts
* do not login into desktop, press: 'ctrl+alt+f1' to open a terminal
* login as your user, and test internet connection with `sudo apt update`

#### ii) Setup SSH connection between CONTROL COMPUTER and ROBOT COMPUTER
This will make the rest of the installation on the SBC much simpler.

* update the repository list - no need to upgrade

  `sudo apt update`

* install SSH for remote connection - this is already installed on Mate image
  while you are doing installs, you should install a terminal text editor like VIM

  `sudo apt install openssh-server vim`

* check that you get a valid ip address and record the IP address - this will change when you move between buildings

  `ip a`

 * Try to ping the `robot` from the `remote` and vice versa.

  `ping 192.168.xxx.yy`

 * You should get a message about bytes transferred as shown below.

  ```
  PING 192.168.254.22 (192.168.254.22) 56(84) bytes of data.
  64 bytes from 192.168.254.22: icmp_seq=1 ttl=64 time=0.599 ms
  64 bytes from 192.168.254.22: icmp_seq=2 ttl=64 time=0.620 ms
  64 bytes from 192.168.254.22: icmp_seq=3 ttl=64 time=0.624 ms
  ```

  Try to ssh into the ROBOT COMPUTER from the REMOTE COMPUTER. Make sure openssh-server is installed on both machines.
  
  `sudo apt install openssh-server`

  `ssh <user on pi>@<ip of pi>`

  `connection refused port 22 closed yada yada`

  SSH may work not on a pi with a fresh image of mate18-arm64, but the solution is shown below.  
  You may read that you have to make some changes to `/etc/ssh/sshd_config` and THIS IS NOT THE FIX that worked for me. 
  Instead the fix is much easier. You must reconfigure the ssh server on the ROBOT COPMUTER (pi) and you should be good to go.
  
  `sudo dpkg-reconfigure openssh-server`
  
  Start the ssh server with the following line.
 
  `sudo systemctl start ssh`
  
  Check status of the ssh server, and you will see that is is not working ( Active: inactive (dead) )
  
  `sudo service ssh status`
  
  And then you can check the status again and see if ssh is running. ( Server listening on 0.0.0.0 port 22 ). 

   
  You may run into the fingerprint issue shown below if the hosts key changes. This will happen if you re-configure a pi after it has gone through an initial ssh handshake. This is just a security warning, but it should not happen unless you caused it to.
  ```
  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  @    WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!     @
  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  IT IS POSSIBLE THAT SOMEONE IS DOING SOMETHING NASTY!
  Someone could be eavesdropping on you right now (man-in-the-middle attack)!
  It is also possible that a host key has just been changed.
  The fingerprint for the ECDSA key sent by the remote host is
  SHA256:pfMmexrK8i2wUQBmbKuj5DDQnaAoqK2WSyg1nE8VnoE.
  Please contact your system administrator.
  Add correct host key in /home/thill/.ssh/known_hosts to get rid of this message.
  Offending ECDSA key in /home/thill/.ssh/known_hosts:1
    remove with:
    ssh-keygen -f "/home/thill/.ssh/known_hosts" -R "192.168.254.22"
  ECDSA host key for 192.168.254.22 has changed and you have requested strict checking.
  Host key verification failed.
  ```

  This is because my computer (sh client) has already talked to this pi with different keys. 
  So run this to delete the old keys and ...

  `ssh-keygen -R [hostname-or-IP]` 

  alternatively you could manually delete the offending key from the 

  `rm /etc/ssh/ssh_host*`

  finally, try again to connect to the ROBOT COMPUTER from the REMOTE COPMUTER
  notice how it shows that you have changed computers. The first time you will have to type `yes`

  `thill@T1600-brwn305:~$ ssh <user on pi>@<ip of pi>`

  ```  
  robot_team2@192.168.254.22's password: 
  Welcome to Ubuntu 18.04.2 LTS (GNU/Linux 4.15.0-1032-raspi2 aarch64)

  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage


  0 packages can be updated.
  0 updates are security updates.

  New release '20.04.1 LTS' available.
  Run 'do-release-upgrade' to upgrade to it.

  Last login: Sat Nov  7 23:33:14 2020 
  ```
  
  this means that you are in, woop woop!

  now would be a good time to make a backup image... lol

  #### iii) Install ROS Melodic 
  These steps come from the ROS wiki here (http://wiki.ros.org/melodic/Installation/Ubuntu). I just noticed that the tutorial is using `ros-melodic-base` and I and using `ros-melodic-desktop-full`
  1) Setup your sources.list

  `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

  2) Set up your keys (if you have issues see the link above)

  `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

  `curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -`

  3) Installation (This step will take several minutes depending on your connection speed and computer)

  `sudo apt update`

  `sudo apt install ros-melodic-desktop-full`

  4) Environment setup

  ```
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

  5) Dependencies for building packages

  `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`

  Initialize rosdep

  `sudo apt install python-rosdep` (this line is redundant)

  ```
  sudo rosdep init
  rosdep update
  ```



  I think that we should just stay headless and I predict that issue will go away, but we will see.

  ##### Setup ROS Workspace 
  Setup a workspace for ros called `pi_ros`. Compile your workspace with `catkin_make`.

  ``` 
  mkdir -p ~/pi_ros/src
  cd pi_ros/
  catkin_make
  echo "source ~/pi_ros/devel/setup.bash" >> ~/.bashrc
  source ~/pi_ros/devel/setup.bash
  ```

  

  now install the turtlebot3 packeges 

   
   
  #### 3) Install Dependent Packages on TurtleBot PC(Download and Compile TurtleBot3 Packges on pi)
  This follows the (SBC setup) tutorial here (https://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/)
  
  download the drivers from github, make sure you are in `~/pi_ros/src`  before you clone the repo

  ```
  cd ~/pi_ros/src
  git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
  git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
  ```

  go into turtlbot3 and delete some stuff (I am not sure why I am just following)

  ``` 
  cd ~/catkin_ws/src/turtlebot3
  rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
  ```

  install some Packages (changed kinetic to melodic)

  
  `sudo apt install ros-melodic-rosserial-python ros-melodic-tf`


  backout to the top of the workspace and build with catkin_make (what is -j1 ?)


  `cd ~/catkin_ws && catkin_make -j1`

  Everything seemed to work just fine - good news
 
 #### III) USB Setting and Network Configuration

 #### 4) USB Settings
 Run this on the PI to setup persitant USB connection to the OpenCR controller. I did this on the separate pi,but I think it needs to be repeated once the controller is plugged in.

 `rosrun turtlebot3_bringup create_udev_rules`

 


 #### 5) Network Configuration


### (6.3 - OpenCR Setup) Embedded Controller Firmware Update 
 
I ran into some problems with the installation script in this step (6.3.1.1 - OpenCR Firmware Update for TB3), and this has been resolved. One line must be added to the `update_opencr/update.sh` script and the script must be run again. This is documented here on Github: https://github.com/ROBOTIS-GIT/turtlebot3/issues/455

Add the following line the the `update_opencr/update.sh` file.

```
case $(uname -m) in
    i386)   architecture="386" ;;
    i686)   architecture="386" ;;
    x86_64) architecture="amd64" ;;
    armv7l) architecture="arm" ;;
    aarch64) architecture="arm";;  # <-- add this line
    arm)    dpkg --print-architecture | grep -q "arm64" && architecture="arm64" || architecture="arm" ;;
esac
```

Now run the installation script again, but do not to re run the `wget` because you do not need to download the update again.

```
cd ./opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr && cd ..
```

### (6.4 Hardware Setup)
### (6.5 Compatible Devices)

**Software Installation Complete** 
Is it finally time to test the motors in **Module 9 - Turtlebot3 - Bringup** ?
