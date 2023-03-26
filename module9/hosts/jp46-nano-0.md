# jp46-nano-0 

this file contains system setup notes for the host jp46-nano-0

board: Jetson Nano 2G, Model 3541
os: jetpack 4.6.1 (jetson-nano-jp461-sd-card-image.zip)
image downloaded from Nvidia 03/25/2023

# network information 

static ip address: 192.168.253.220/24
gateway: 192.168.253.253

using NetworkManager (GUI) pre-installed in Jetpack
(consider using netplan instead - not pre-installed)

# system updates

```
sudo apt update
sudo apt upgrade
```

Choose the default (N) to both prompts about updating tegra and nv-oem

There is an error:
```
Setting up libreoffice-writer (1:6.0.7-0ubuntu0.18.04.12) ...
Processing triggers for mime-support (3.60ubuntu1) ...
Processing triggers for desktop-file-utils (0.23-1ubuntu3.18.04.2) ...
Processing triggers for libglib2.0-0:arm64 (2.56.4-0ubuntu0.18.04.9) ...
Processing triggers for menu (2.1.47ubuntu2.1) ...
Processing triggers for initramfs-tools (0.130ubuntu3.13) ...
update-initramfs: Generating /boot/initrd.img-4.9.253-tegra
WARNING: missing /lib/modules/4.9.253-tegra
Ensure all necessary drivers are built into the linux image!
depmod: ERROR: could not open directory /lib/modules/4.9.253-tegra: No such file or directory
depmod: FATAL: could not search modules: No such file or directory
Warning: couldn't identify filesystem type for fsck hook, ignoring.
I: The initramfs will attempt to resume from /dev/zram3
I: (UUID=587b65a4-fb75-4140-8e69-9d0d965a2a92)
I: Set the RESUME variable to override this.
depmod: WARNING: could not open /var/tmp/mkinitramfs_43GQbA/lib/modules/4.9.253-tegra/modules.order: No such file or directory
depmod: WARNING: could not open /var/tmp/mkinitramfs_43GQbA/lib/modules/4.9.253-tegra/modules.builtin: No such file or directory
/sbin/ldconfig.real: Warning: ignoring configuration file that cannot be opened: /etc/ld.so.conf.d/aarch64-linux-gnu_EGL.conf: No such file or directory
/sbin/ldconfig.real: Warning: ignoring configuration file that cannot be opened: /etc/ld.so.conf.d/aarch64-linux-gnu_GL.conf: No such file or directory
Processing triggers for gnome-icon-theme (3.12.0-3) ...
Processing triggers for bamfdaemon (0.5.3+18.04.20180207.2-0ubuntu1) ...
Rebuilding /usr/share/applications/bamf-2.index...
Processing triggers for libc-bin (2.27-3ubuntu1.6) ...
Processing triggers for systemd (237-3ubuntu10.57) ...
Processing triggers for man-db (2.8.3-2ubuntu0.1) ...
Processing triggers for shared-mime-info (1.9-2) ...
Processing triggers for gnome-menus (3.13.3-11ubuntu1.1) ...
Processing triggers for hicolor-icon-theme (0.17-2) ...
Setting up nautilus (1:3.26.4-0~ubuntu18.04.6) ...
Processing triggers for fontconfig (2.12.6-0ubuntu2) ...
Processing triggers for ca-certificates (20211016ubuntu0.18.04.1) ...
Updating certificates in /etc/ssl/certs...
0 added, 0 removed; done.
Running hooks in /etc/ca-certificates/update.d...
done.
Processing triggers for nvidia-l4t-kernel (4.9.299-tegra-32.7.3-20221122092935) ...
Errors were encountered while processing:
 nvidia-l4t-bootloader
E: Sub-process /usr/bin/dpkg returned an error code (1)
```

The workaround from this link https://forums.developer.nvidia.com/t/solution-dpkg-error-processing-package-nvidia-l4t-bootloader-configure/208627
seems to work, but it seems like a hack.

 sudo apt update
Get:1 file:/var/cuda-repo-l4t-10-2-local  InRelease
Ign:1 file:/var/cuda-repo-l4t-10-2-local  InRelease
Get:2 file:/var/visionworks-repo  InRelease
Ign:2 file:/var/visionworks-repo  InRelease
Get:3 file:/var/visionworks-sfm-repo  InRelease
Ign:3 file:/var/visionworks-sfm-repo  InRelease
Get:4 file:/var/visionworks-tracking-repo  InRelease
Ign:4 file:/var/visionworks-tracking-repo  InRelease
Get:5 file:/var/cuda-repo-l4t-10-2-local  Release [564 B]
Get:6 file:/var/visionworks-repo  Release [2,001 B]
Get:7 file:/var/visionworks-sfm-repo  Release [2,005 B]
Get:8 file:/var/visionworks-tracking-repo  Release [2,010 B]
Get:5 file:/var/cuda-repo-l4t-10-2-local  Release [564 B]
Get:6 file:/var/visionworks-repo  Release [2,001 B]
Get:7 file:/var/visionworks-sfm-repo  Release [2,005 B]     
Get:8 file:/var/visionworks-tracking-repo  Release [2,010 B]                 
Hit:9 http://ports.ubuntu.com/ubuntu-ports bionic InRelease                  
Hit:11 https://repo.download.nvidia.com/jetson/common r32.7 InRelease               
Hit:12 http://ports.ubuntu.com/ubuntu-ports bionic-updates InRelease     
Hit:13 http://ports.ubuntu.com/ubuntu-ports bionic-backports InRelease                                      
Hit:14 https://repo.download.nvidia.com/jetson/t210 r32.7 InRelease                                         
Hit:15 http://ports.ubuntu.com/ubuntu-ports bionic-security InRelease    
Reading package lists... Done                      
Building dependency tree       
Reading state information... Done
1 package can be upgraded. Run 'apt list --upgradable' to see it.
thill@jp46-nano-0:~$ sudo apt upgrade
Reading package lists... Done
Building dependency tree       
Reading state information... Done
Calculating upgrade... Done
The following packages were automatically installed and are no longer required:
  apt-clone archdetect-deb bogl-bterm busybox-static cryptsetup-bin dpkg-repack gir1.2-timezonemap-1.0 gir1.2-xkl-1.0 grub-common kde-window-manager kinit kio
  kpackagetool5 kwayland-data kwin-common kwin-data kwin-x11 libdebian-installer4 libkdecorations2-5v5 libkdecorations2private5v5 libkf5activities5 libkf5attica5
  libkf5completion-data libkf5completion5 libkf5declarative-data libkf5declarative5 libkf5doctools5 libkf5globalaccel-data libkf5globalaccel5
  libkf5globalaccelprivate5 libkf5idletime5 libkf5jobwidgets-data libkf5jobwidgets5 libkf5kcmutils-data libkf5kcmutils5 libkf5kiocore5 libkf5kiontlm5
  libkf5kiowidgets5 libkf5newstuff-data libkf5newstuff5 libkf5newstuffcore5 libkf5package-data libkf5package5 libkf5plasma5 libkf5quickaddons5 libkf5solid5
  libkf5solid5-data libkf5sonnet5-data libkf5sonnetcore5 libkf5sonnetui5 libkf5textwidgets-data libkf5textwidgets5 libkf5waylandclient5 libkf5waylandserver5
  libkf5xmlgui-bin libkf5xmlgui-data libkf5xmlgui5 libkscreenlocker5 libkwin4-effect-builtins1 libkwineffects11 libkwinglutils11 libkwinxrenderutils11
  libqgsttools-p1 libqt5designer5 libqt5help5 libqt5multimedia5 libqt5multimedia5-plugins libqt5multimediaquick-p5 libqt5multimediawidgets5 libqt5opengl5
  libqt5quickwidgets5 libqt5sql5 libqt5test5 libxcb-composite0 libxcb-cursor0 libxcb-damage0 os-prober python3-dbus.mainloop.pyqt5 python3-icu python3-pam
  python3-pyqt5 python3-pyqt5.qtsvg python3-pyqt5.qtwebkit qml-module-org-kde-kquickcontrolsaddons qml-module-qtmultimedia qml-module-qtquick2 rdate tasksel
  tasksel-data
Use 'sudo apt autoremove' to remove them.
Get more security updates through Ubuntu Pro with 'esm-apps' enabled:
  libavformat57 libopenjp2-7 libavfilter6 libgroupsock8 ffmpeg libswresample2
  libzmq5 libopenmpt0 libpostproc54 libopenmpt-modplug1
  libbasicusageenvironment1 libsoundtouch1 libavcodec57 libjs-jquery-ui
  libavutil55 libavdevice57 libswscale4 libsdl2-2.0-0 libmysofa0
  liblivemedia62 libavresample3 libusageenvironment3
Learn more about Ubuntu Pro at https://ubuntu.com/pro
The following packages have been kept back:
  tensorrt
0 upgraded, 0 newly installed, 0 to remove and 1 not upgraded.
```
Hopefully the kept back package will not cause any major issues. 


# install ROS

double check OS version 
```
lsb_release -a

ubuntu18.04
```

follow the standard instructions for melodic

setup sources
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

setup keys
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

install ROS
```
sudo apt update

sudo apt install ros-melodic-desktop-full
```
note: desktop-full is probably not needed because this is intended to be the robot computer, but it will not hurt

environment setup

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

install build deps
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

initialize rosdep
```
sudo rosdep init
rosdep update
```

finally, open a new terminal and test the installation 

```
roscore
```
# create a workspace  

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
source the workspace files in ~/.bashrc 
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# install Roboteq ROS driver
follow the instructions here:
https://github.com/Roboteq-Inc/ROS-Driver/tree/FW2.1/ROS-Driver-Update

```
cd ~/catkin_ws/src
git clone https://github.com/Roboteq-Inc/ROS-Driver.git
cd ..
catkin_make
```
the is an error related to the package 'serial', try installing the offending package for ros-melodic

```
sudo apt update
sudo apt install ros-melodic-serial
```
this install without errors
now try to build to workspace again

```
cd ~/catkin_ws
catkin_make
```
the workspace built with one warning from the roboteqdriver, no errors and all targets built

```
[ 81%] Building CXX object ROS-Driver/ROS-Driver-Update/roboteq_motor_controller_driver/CMakeFiles/config_client.dir/src/config_client.cpp.o
/home/thill/catkin_ws/src/ROS-Driver/ROS-Driver-Update/roboteq_motor_controller_driver/src/roboteq_motor_controller_driver_node.cpp: In member function ‘bool RoboteqDriver::configservice(roboteq_motor_controller_driver::config_srv::Request&, roboteq_motor_controller_driver::config_srv::Response&)’:
/home/thill/catkin_ws/src/ROS-Driver/ROS-Driver-Update/roboteq_motor_controller_driver/src/roboteq_motor_controller_driver_node.cpp:117:7:
warning: unknown escape sequence: '\c'
    << "%\clsav321654987";
       ^~~~~~~~~~~~~~~~~~
```
soon it is time to test the motors

# install lm-sensors to check cpu temps
```
sudo apt update
sudo apt install lm-sensors
```

test it
```
 sensors
thermal-fan-est-virtual-0
Adapter: Virtual device
temp1:        +26.8°C
```

very cool, lol




