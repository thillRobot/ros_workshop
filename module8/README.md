# Module 8 - Turtlebot3 Setup 

The main objective is to get the turtlebot3 robots working with ros navigation for ME4440. We are following the turtlebot3 manual here. (https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup). 

### Hardware
  * turtlebot3 waffle pi full robot kit (x2)
    * Rasp Pi 3B+ - (ROBOT) 
    * openCR 1.0 - (MCU)
    * RPLidar A1 - (SENSOR)
  * Laptop for ME4444 - (REMOTE COMPUTER #1)
  * Lenovo ThinkCentreM73 i3 - (REMOTE COMPUTER #2) - note: on loan from ME Robotics Lab
  * Lenovo ThinkCentreM73 i5 (ubuntu18-m73i5) - (REMOTE COMPUTER #3) - note: on loan from ME Robotics Lab

### Required Software
  * Mate 18.04 LTS - (ROBOT COMPUTER) 
  * Ubuntu 18.04 LTS - (REMOTE COMPUTER) 
  * ROS Melodic
  * OpenSSH
  * pi-imager
  
### Turtlebot3 Setup Overview
  - [x] (6.1 - PC Setup) REMOTE COMPUTER Software Installation 
  - [x] (6.2 - SBC Setup) ROBOT COMPUTER Software Installation  
  - [x] (6.3 - OpenCR Setup) Embedded Controller Firmware Update 
  - [ ] (6.4 Hardware Setup)
  - [ ] (6.5 Compatible Devices)

### (6.1 - PC Setup) REMOTE COMPUTER Software Installation 
The control computer requires the same OS version as the robot computer, but the flavor can be different. 

#### (6.1.1 - Install Ubuntu on Remote PC)
DOwnload Ubuntu 18.04.5 LTS Desktop 64bit image (https://wiki.ubuntu.com/Releases?_ga=2.126560777.568362595.1604554678-2034758377.1604554678)
Use `Startup Disk Creator` or `Rufus` to make bootable USB disk with image.
Use boot disk from to install the Ubuntu operating system. Setup a user account, and update while installing. 
#### (6.1.2 - Install ROS 1 on Remote PC)
Follow the instructions on the ROS wiki here (http://wiki.ros.org/melodic/Installation/Ubuntu) OR see detailed steps below.
#### (6.1.3 - Install Dependent ROS 1 Packages) 
I JUST REALIZED THAT I THINK WE SKIPPED THIS STEP - now I did it on the NUC and Jared J, did it on the laptop
notice I had to change the kinetics to melodics as you can see below

```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

#### (6.1.4 - Network Configuration)
Setup `ssh` connection. Use `ip a` and `ping` to test.

### (6.2 - SBC Setup) ROBOT COMPUTER Software Installation
These steps come from here:(https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup).
**NOTE:** Using the Mate Desktop is not reccomended. Ubuntu 18.04 on the RasPi 3B+ tends to run out of memory. I reccomend using the terminal (ctrl+alt+f1 before loggin in), and in the future we will look into using a RaspPI 4 which claims to have increased memory.

#### (6.2.1 - Raspberry Pi 3 Setup)

##### 1) Install Ubuntu MATE on TurtleBot PC -
Since this is a rasp pi we are not really installing the OS on the pi. Instead we are copying and image onto the pi.
* download Mate 18.04 64bit image for pi 3B + from Ubuntu Mate website (https://ubuntu-mate.org/download/arm64/)
  this is currently not available, get it here instead (https://www.dropbox.com/sh/sl7p8ccff6ofv3n/AAB6gt3jbKkXziI_uIC-9wS6a?dl=0)
  - current image file(compressed): `ubuntu-mate-18.04.2-beta1-desktop-arm64+raspi3-ext4.img.xz`
* download and/or install `pi-imager` (https://www.raspberrypi.org/downloads/)
* use `pi-imager` to load the image to SD card
* use GUI to setup user accounts
* do not login into desktop, press: 'ctrl+alt+f1' to open a terminal
* login as your user, and test internet connection with `sudo apt update`

##### 1+) Setup SSH connection between CONTROL COMPUTER and ROBOT COMPUTER
This will make the rest of the installation on the SBC much simpler.

update the repository list - no need to upgrade

  `sudo apt update`

setup SSH for remote connection 

This is already installed on Mate image, but make sure openssh-server is installed on both machines.
While you are doing installs, you should install a terminal text editor like `vim`.

  `sudo apt install openssh-server vim`

Check that you get a valid ip address and record the IP address - this will change when you move between networks

  `ip a`

Try to ping the `robot` from the `remote` and vice versa.

  `ping 192.168.xxx.yy`

You should get a message about bytes transferred as shown below.

  ```
  PING 192.168.254.22 (192.168.254.22) 56(84) bytes of data.
  64 bytes from 192.168.254.22: icmp_seq=1 ttl=64 time=0.599 ms
  64 bytes from 192.168.254.22: icmp_seq=2 ttl=64 time=0.620 ms
  64 bytes from 192.168.254.22: icmp_seq=3 ttl=64 time=0.624 ms
  ```

Try to ssh into the ROBOT COMPUTER from the REMOTE COMPUTER. 
  
  `ssh <user on pi>@<ip of pi>`

  `connection refused port 22 closed yada yada`
  
SSH may work not on a pi3b+ with a fresh image of mate18-arm64, but the solution is shown below.  
You may read that you have to make some changes to `/etc/ssh/sshd_config` and THIS IS NOT THE FIX that worked for me. 
Instead the fix is much easier. You must reconfigure the ssh server on the ROBOT COPMUTER (pi) and you should be good to go.
  
  `sudo dpkg-reconfigure openssh-server`
  
Start the ssh server with the following line.
 
  `sudo systemctl start ssh`
  
Check status of the ssh server, and you will see that is is not working ( Active: inactive (dead) )
  
  `sudo service ssh status`
  
You should get something that contains `Server listening on 0.0.0.0 port 22` 

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
So run this to delete the old keys.

  `ssh-keygen -R [hostname-or-IP]` 

Alternatively you could manually delete the offending key by removing the following file. DON'T DO BOTH.

  `rm /etc/ssh/ssh_host*`

Finally, try again to connect to the ROBOT COMPUTER from the REMOTE COPMUTER
Notice it shows that you have changed computers. The first time you `ssh` will have to type `yes`

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
This means that you are in, woop woop!

  ##### 2) Install ROS on TurtleBot PC 
  These steps come from the ROS wiki here (http://wiki.ros.org/melodic/Installation/Ubuntu). I just noticed that the tutorial is using `ros-melodic-base` and I and using `ros-melodic-desktop-full`
  
  Setup your sources.list

  `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

  Set up your keys (if you have issues see the link above)

  `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

  `curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -`

  Installation (This step will take several minutes depending on your connection speed and computer)

  `sudo apt update`

  `sudo apt install ros-melodic-desktop-full`

  Environment setup

  ```
  echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

   Dependencies for building packages

  `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`

  Initialize rosdep

  `sudo apt install python-rosdep` (this line is redundant)

  ```
  sudo rosdep init
  rosdep update
  ```

  ##### 2+) Setup ROS Workspace 
  Before installing the install the turtlebot3 packages, setup a workspace for ros called `pi_ros`. Compile your workspace with `catkin_make`.

  ``` 
  mkdir -p ~/pi_ros/src
  cd pi_ros/
  catkin_make
  echo "source ~/pi_ros/devel/setup.bash" >> ~/.bashrc
  source ~/pi_ros/devel/setup.bash
  ```
   
  ##### 3) Install Dependent Packages on TurtleBot PC   
  Download the packages from github, make sure you are in `~/pi_ros/src`  before you clone the repository.

  ```
  cd ~/pi_ros/src
  git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
  git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
  ```

  Go into turtlbot3 and delete some stuff (I am not sure why I am just following).

  ``` 
  cd ~/catkin_ws/src/turtlebot3
  rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
  ```

  install some packages (change kinetic to melodic)
 
  `sudo apt install ros-melodic-rosserial-python ros-melodic-tf`

  backout to the top of the workspace and build with catkin_make (what is -j1 ?)

  `cd ~/catkin_ws && catkin_make -j1`

  Everything seemed to work just fine - good news
 

 ##### 4) USB Settings
 Run this on the PI to setup persitant USB connection to the OpenCR controller. The `pi` must be `OpenCR` for this to work.

 `rosrun turtlebot3_bringup create_udev_rules`

 ##### 5) Network Configuration
 
 See `ssh` section above. Use `ip a` and `ping` to test connection.
 
 #### (6.2 - COMPLETE ) - Image and Clone SD Card for Backup!

We have made tremendous progress, so now would be a good time to make a backup image. There are many ways to do this but I like to use `dd` which is commonly available in linux. The install below is most likely not needed, but it will not hurt.

`sudo apt install dd` 

There is a post here (https://askubuntu.com/questions/227924/sd-card-cloning-using-the-dd-command) that clearly explains how to use `dd` to clone an SD card.

Copying from an SD card to a HDD or SSD takes 16GB (or size of card) of free space. Copying to an SD card from an HDD or SSD takes a long time. You can also copy from one SD directly to another with `dd`. What a hammer.


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

### (6.4 - Hardware Setup)

There does not appear to be any required steps here. See this section for more info about the chassis and other hardware.

### (6.5 - Compatible Devices)

See this section for more information about optional components and upgrades. 

### Software Installation Complete - AWESOME 
Is it finally time to test the motors in **Module 9 - Turtlebot3 Bringup** 

On Tuesday Nov 24 we successfully tested **keyboard teleop** - Good Job team


