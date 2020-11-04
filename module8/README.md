# Module 8 - Turtlebot3 Robot Installation

The main objective is to get the turtlebot3 robots working with ros navigation for ME4440. We are loosely following the turtlebot3 manual here. (https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup). 

### Hardware
  * turtlebot3 waffle pi full robot kit 
    * Rasp Pi 3B+ - (ROBOT) 
    * openCR 1.0 - (MCU)
    * RPLidar A1 - (SENSOR)
  * Mystery Laptop for ME4444 - (CONTROL)
  * Lenovo ThinkCentreM73 i3 - (CONTROL) - note: on loan from ME Robotics Lab

### Required Software
  * Mate 18.04 LTS - (ROBOT) - local 
  * Ubuntu 18.04 LTS - (CONTROL) - remote
  * ROS Melodic
  * OpenSSH
  * pi-imager
  
### CONTROL Software Installation 

### ROBOT Software Installation 

#### Install MATE 18.04 on Rasp Pi - USING DESKTOP
Since this is rasp pi we are not really installing the OS on the pi. Instead we are copying an image onto the pi.
1) download Mate 18.04 64bit image for pi 3B + from Ubuntu Mate website (https://ubuntu-mate.org/download/arm64/)
2) download and/or install `pi-imager` (https://www.raspberrypi.org/downloads/)
3) use `pi-imager` to load the image to SD card
4) setup user accountwith Mate installation GUI (comes with boot disk)
5) login to MATE desktop and test internet connection with `sudo apt update`

  **NOTE:** this all worked but the grapics are terribly slow. 

#### Install ROS Melodic 
These steps come from the ROS wiki here (http://wiki.ros.org/melodic/Installation/Ubuntu).
1) Setup your sources.list

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

2) Set up your keys (if you have issues see the link above)

`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

`curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -`

3) Installation (This step will take several minutes depending on your connection speed and computer)

`sudo apt update`

`sudo apt install ros-melodic-desktop-full`





I think that we should just stay headless and I predict that issue will go away, but we will see.

#### Install MATE 18.04 on Rasp Pi - USING TERMINAL
Again, since this is a rasp pi we are not really installing the OS on the pi. Instead we are copying and image onto the pi.
* download Mate 18.04 64bit image for pi 3B + from Ubuntu Mate website (https://ubuntu-mate.org/download/arm64/)
* download and/or install `pi-imager` (https://www.raspberrypi.org/downloads/)
* use `pi-imager` to load the image to SD card
* use GUI to setup user accounts
* do not login into desktop, press: 'ctrl+alt+f1' to open a terminal
* login as your user, not root for now

We are going to complete the installation through the terminal.

* update the repository list - no need to upgrade

  `sudo apt update`

* install SSH for remote connection - this is already installed on Mate image

  `sudo apt install openssh-server`

* test connectivity

  `ip a`

  check that you get a valid ip address - take a picture of the terminal!

  try to ping the pi from a remote computer (on the network)

  `ping 192.168.xxx.yy`

  you should get byte transferred as shown below

  `PING 192.168.254.22 (192.168.254.22) 56(84) bytes of data.
  64 bytes from 192.168.254.22: icmp_seq=1 ttl=64 time=0.599 ms
  64 bytes from 192.168.254.22: icmp_seq=2 ttl=64 time=0.620 ms
  64 bytes from 192.168.254.22: icmp_seq=3 ttl=64 time=0.624 ms`

  next try to ssh in from the remote computer. Make sure openssh-server is on both machines

  `ssh <user on pi>@<ip of pi>`

  ssh was not working, so... I made some changes to /etc/ssh/sshd_config THIS WAS NOT THE FIX
  the fix is much easier, just run this on the host (the pi) and you should be good to go

  `sudo dpkg-reconfigure openssh-server`

  finally test that you can connect to the pi from the control computer
  notice how it shows that you have changed computers

  `thill@T1600-brwn305:~$ ssh thill@192.168.254.22`

```  thill@192.168.254.22's password:
  Welcome to Ubuntu 18.04.2 LTS (GNU/Linux 4.15.0-1032-raspi2 aarch64)

   * Documentation:  https://help.ubuntu.com
   * Management:     https://landscape.canonical.com
   * Support:        https://ubuntu.com/advantage


  0 packages can be updated.
  0 updates are security updates.

  New release '20.04.1 LTS' available.
  Run 'do-release-upgrade' to upgrade to it.

  Last login: Sat Oct 24 01:09:38 2020 from 192.168.254.45
  thill@mate18-pi3bp:~$
```
  this means that you are in, woop woop!

  now would be a good time to make a backup image... lol

  install ROS following  the steps here http://wiki.ros.org/melodic/Installation/Ubuntu but do them through ssh

  then setup a workspace for ros called 'pi_ros'

``` ~$ mkdir -p ~/pi_ros/src
  ~$ cd pi_ros/
  ~/pi_ros$ catkin_make
  ~/$ echo "~/pi_ros/devel/setup.bash" >> ~/.bashrc
  source ~/pi_ros/devel/setup.bash
  ```



  your workspace should compile without errors

  now install the turtlebot3 packeges following this https://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/ (SBC setup)

   SKIP 1) Install Ubuntu MATE on TurtleBot PC - we already did this 'manually'

   SKIP 2) Install ROS on TurtleBot PC - we already did this 'manually'

   START 3) Install Dependent Packages on TurtleBot PC - replace all instances of 'kinetic' with 'melodic'


      download the drivers from github, make sure you are in `~/pi_ros/src`  before you clone the repo

      ```
      $ cd ~/pi_ros/src
      $ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
      $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
      $ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
      ```

      go into turtlbot3 and delete some stuff (I am not sure why I am just following)

      ``` $ cd ~/catkin_ws/src/turtlebot3
      $ rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
      ```

      install some Packages (changed kinetic to melodic)


      $ sudo apt install ros-melodic-rosserial-python ros-melodic-tf


      backout to the top of the workspace and build with catkin_make (what is -j1 ?)


      `$ cd ~/catkin_ws && catkin_make -j1`

      Everything seemed to work just fine - good news

      DO step 4,5 next

      4) USB Settings

      5) Network Configuration
