# Notes
This package uses 
1. [apriltag](https://github.com/AprilRobotics/apriltag)
2. [apriltag_msgs](https://github.com/christianrauch/apriltag_msgs)
3. [apriltag_ros](https://github.com/christianrauch/apriltag_ros)

Please fetch the latest update if necessary

# Setup apriltag_localization package

## In both PC and raspi end

## clone this repository (ignore this step if previously done)
git clone https://github.com/MonashRobotics/networked_robotics.git

## assign ID to the robot (raspi end)
Go to ros2_localization/src/localization/launch/raspi_nodes.launch.py and edit the global variable ROBOT_ID as string

## Setup the camera and apriltags frames

### important notes
Please refer to http://willshw.me/2018/04/27/apriltag2-ros-frame.html for the correct frame orientation and don't get 
confused by the official documentation

Tag size documentation can be found:
1. https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
2. https://doc.rc-visard.com/latest/en/tagdetect.html

### raspi end 
Go to ros2_localization/src/localization/launch/raspi_nodes.launch.py and edit the camera pose in respect to the centre of the robot

### remote end
Go to ros2_localization/src/localization/launch/remote_nodes.launch.py and edit the apriltag frame respectively


## Setup AprilTag detection
Go to ros2_localization/src/localization/config/aprilTag.yaml and edit the tag size and tag families accordingly


## build the package (both end)
Run 
```
source /opt/ros/humble/setup.bash
```

Add sourcing to startup file
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

```
export ROS_DOMAIN_ID=13
echo "export ROS_DOMAIN_ID=13" >> ~/.bashrc
```

and then go to the root workspace
```
cd <path to /ros2_localization>
```

```
colcon build 
sudo rosdep init 
rosdep update 
rosdep install --from-paths src -y --ignore-src
```

And then,
In ros2_localization\src\localization\launch\raspi_nodes.launch.py, change the camera_info_url parameter for cam_0, cam_1, cam_2, and cam_3 to the corresponding path to yaml file obtained from calibration

Then in root workspace
```
colcon build
```
Notes: If any codes are changed remember to run colcon build

## Setting up for one-to-one communication (Ubuntu PC with Raspi)
### Notes: Make sure the PC and Raspi have the same version of ROS2

## On raspi and PC end

## if ssh not working, then install openssh-server
```
sudo apt-get install openssh-server
sudo systemctl enable ssh --now
sudo ufw allow 22
```

## to allow for multicast 
### Important Note: Do not connect to Eduroam, Eduroam will block multicast

### PC end
```
sudo ufw allow from <raspi-ip>
sudo ufw allow to <raspi-ip>
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```
  
### Raspi End
```
sudo ufw allow to <host-ip> 
sudo ufw allow from <host-ip>
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```
  
### In both host and raspi
```
export ROS_DOMAIN_ID=13
echo "export ROS_DOMAIN_ID=13" >> ~/.bashrc
```

## check for multicast working or not

### at one end
```
ros2 multicast receive
```

### another end: 
```
ros2 multicast send
```
should see a Hello World ! if multicast is working

# Steps to repeat each time to run the package
In the root workspace (/ros2_localization)
```
. install/setup.bash
```
### Important !!! : always remember to . install/setup.bash each time after to cd to the ros2 worskpace

## raspi end
```
ros2 launch <path to raspi_nodes.launch.py>
```

## remote end

### In the first terminal
```
ros2 launch <path to remote_nodes.launch.py>
```

### Second terminal
```
ros2 run rviz2 rviz2
```
and then add tf visualisation to visualise the result

## To control the robot
Install teleop keyboard on raspi and control the robot with terminal of raspi(via SSH)
```
sudo apt-get install ros-humble-teleop-twist-keyboard 
```

To run it, open a new terminal on raspi(to minimize delay)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the key shown in the instruction printed on terminal and tune it to:
linear velocity of around 0.2 to 0.25 metre/s
angular velocity around 2 rad/s ( the robot wheel give a lot of resistance)

Have fun pacticing with it !!!





# Miscellaneous

## To test the reliability of camera calibration and AprilTags
go to /apriltags_localization and then
```
. install/setup.bash
ros2 run apriltags_localization measure
```

## Pre-generated apriltags 
Visit [here](https://github.com/AprilRobotics/apriltag-imgs)

## Tutorials on robot_localization package 

1. [ros-sensor-fusion tutorial](https://github.com/methylDragon/ros-sensor-fusion-tutorial)

2. [Kapernikov: The ROS robot_localization package](https://kapernikov.com/the-ros-robot_localization-package/)

3. [https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/]

4. [Setting up Odometry](https://navigation.ros.org/setup_guides/odom/setup_odom.html)

5. [A similar project to this one](https://raceon.io/localization/)

## Tutorials on tf2

1. [Official tf2 for ROS2 Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

2. [Navigation ROS tf2 tutorial](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html)

## Tutorials for Quartenions and Kinematics

1. http://elvis.rowan.edu/~kay/papers/kinematics.pdf
2. https://eater.net/quaternions

## Unable to load and run RVIZ2

[check this post](https://answers.ros.org/question/402022/rviz2-error-while-loading-shared-libraries/)


