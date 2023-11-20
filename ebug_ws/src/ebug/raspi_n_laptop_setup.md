
# Setting Up Raspberry Pi and laptop with Ubuntu 22.04

## Installation of Ubuntu/Linux Operating System on Raspberry PI

First, you need to download and install [Raspberry Pi
Imager](https://www.raspberrypi.com/software/). Afterwards, you are
ready to install the operating system. You can use either of the
following methods (download page is
[here](https://ubuntu.com/download/raspberry-pi):

Make sure your raspi and laptop uses the same timezone and time (also
make sure that they are synced), **a lot of pain** will be endured if this
is not done propely check with the command

```
timedatectl
```

### Desktop Edition

If you have a display, keyboard and mouse to connect to your R-pi, you
can use the "desktop edition"
* Insert the SD card and write Ubuntu 22.04 Desktop to the SD card 
* Remove SD card and insert into raspberry pi
* Connect raspberry pi to the monitor, mouse and keyboard
* Setup accordingly
* Remember to connect to Wi-Fi (this could be tricky if you are going
  to use Eduroam) (further info to be added)
* Install the recently updated packages: 
```
        sudo apt update
        sudo apt upgrade
```

### Server Edition
* Required: An SD card reader
* Download Raspberry Pi Imager
* Connect the SD card reader to your PC and insert the SD card and
  write Ubuntu 22.04 Server onto the SD card Raspberry PI Imager
  "advanced options" help to setup wifi connection, if you set the
  hostname, username and password, and your WiFi router supports DHCP
  you can login through "slogin XXX.local" command (XXX being the
  hostname).
* Remove the SD card and insert into your Raspberry PI
* You are ready for ROS installation

Recommended to connect to a mobile hotspot so that ip address of raspi
can be obtained without connecting to a mouse and screen

Run
```
sudo apt update
sudo apt upgrade
```

### Connecting to WiFi in ubuntu server with Raspi 

Follow the tutorial in this link: https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line

Side notes: eduroam don't like ROS2 multicast, using home network/access point/ personal hotspot on mobile phone is a better idea. In some android device, personal hotspot is configured to connect to mobile data **only** and to share wifi connection, there is a WiFI bridge function instead (at least this is the case on my very old Huawei Phone)

Possible new solution suggested by Kelvin: wicd curses

If there is ECDSA key problem with ssh, solve by
```
ssh-keygen -R {ip_address of rpi}
```

### Update the firmware of the raspi
```
sudo rpi-eeprom-update
```

## Installation of the Additional Packages (this apply to both raspi and remote nodes(ie. laptop)) 

Install all of the packages listed below

### ROS2 humble-hawksbill

Please follow this link
notes: intall ros-base with developer tools only, no need GUI for ROS
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

To check ros2 humble is installed properly
```
printenv ROS_DISTRO
```
It should print out 
```
humble
```

## pip
```
sudo apt install python3-pip
```

## numpy
```
pip install numpy
```
This command installs numpy on user space. 
Perhaps this is not a good thing. (We need to check)

## matplotlib
pip install matplotlib

## setup tool warning while colcon build
pip install setuptools==58.2.0

## tf2
```
sudo apt-get install ros-humble-tf2-tools ros-humble-tf-transformations
```

## git
```
sudo apt-get install git
```

## opencv2
```
pip install opencv-python
```
## robot_localization
```
sudo apt-get install ros-humble-robot-localization
```

## rviz2 (**raspi do not need this**)
Rviz2 is a very powerful visualization tool, raspi with ubuntu server can't handle it as we need GUI to run rviz2
```
sudo apt install ros-humble-rviz2
```

## usb-cam

```
sudo apt-get install ros-humble-usb-cam
```

## Install colcon
```
sudo apt install python3-colcon-common-extensions
```

## image_proc
```
sudo apt-get install ros-humble-image-pipeline
```

### Camera calibration

Follow the [Camera Calibration Guide](https://github.com/MonashRobotics/networked_robotics/blob/main/calibration_guide.md)

### pre-generated AprilTags
[Pre-generated AprilTags](https://github.com/AprilRobotics/apriltag-imgs)

### SSH problems
1. If the terminal hang after password prompt try
```
ssh-keygen -R [ip address of target]
```

## gitignore
Adding gitignore for build, install and log is essential to prevent pushing large files to github

## Some important notes
If you ever found that some crucial problem due to dpkg pipe broken and lead to some weird problems (like wlan0 completly gone from ip a) Try these steps before choosing the nuking it(I mean deleting ubuntu and re-installing it :D )



Follow these steps:

Try configuring unconfigured packages:
```
sudo dpkg --configure -a
```
Update the contents of the repositories
```
sudo apt-get update
```
Try to fix missing dependencies:
```
sudo apt-get -f install
```
Update all packages with new versions available:
```
sudo apt-get full-upgrade
```
Reinstall Ubuntu desktop:
```
sudo apt-get install --reinstall ubuntu-desktop
```
Remove unnecessary packages:
```
sudo apt-get autoremove
```
Delete downloaded packages already installed:
```
sudo apt-get clean
```
Reboot the system to see if the issue was resolved:
```
sudo reboot
```

Thanks: https://askubuntu.com/questions/859448/is-there-a-command-to-factory-reset-ubuntu

