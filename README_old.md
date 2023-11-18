# Networked Robotics Project

We are building a swarm of mobile robots that communicate with each
other to cooperatively perform tasks. Our broad aim is to investigate
multi-agent systems and various sensor fusion mechanisms. We are
particularly interested in visual sensing and multi-camera processing
in a distributed manner. We have deliberately chosen off-the-shelf
hardware to keep the costs down and get the robots going as soon as
possible. 

Each robot consists of:
* A romi chassis kit (including motors, wheels, encoders etc.)
* A romi 32U4 control board
* Four USB cameras which are arranged to obtain 360 degrees view, and
* A Raspberry Pi 4
* A Raspberry Pi battery hat

After putting the robot kit together (see romi
chassis assembly pages), follow the below links in ascending order to
complete a robot's basic setup:
1. [Raspberry Pi installation guide](https://github.com/MonashRobotics/networked_robotics/blob/main/raspi_n_laptop_setup.md)
2. [Camera calibration](https://github.com/MonashRobotics/networked_robotics/blob/main/calibration_guide.md)
3. [Setting up Romi board](https://github.com/MonashRobotics/networked_robotics/blob/main/setup_raspi_pololu.md)
4. [Setting up the localization](https://github.com/MonashRobotics/networked_robotics/blob/main/apriltag_localization_guide.md)


There are some quick handy ROS2 command that might be helpful in [quick_ros_commands.txt](https://github.com/MonashRobotics/networked_robotics/blob/main/quick_ros_commands.txt)