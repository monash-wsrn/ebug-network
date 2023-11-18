1. create a .yaml file to configure the robot ID and other robot specific properties, put in ros2_localization/src/localization/configs
- check by using the command ros2 topic list while the raspi_nodes.launch.py is ran, check robot_id and other properties are well assigned or not

2. Try fiddle around with the usb_cam nodes, documentation is provided at : https://github.com/ros-drivers/usb_cam/tree/ros2
The repository just updated, there are some new features that I am not aware of previously whihc might be helpful

3.  design the arena and modify the remote_nodes.launch.py accordingly to the apriltag location. Update the /config/aprilTag.yaml accordingly to the properties of apriltag used

4. ekf_odom.yaml and ekf.yaml are configured manually to subscribe to certain topic, might be worth to write simple pythin code to read robot_id from the .yaml file prepared in TODO 1.

5.  try to play around with poller.py in /src/localization, we might be able to poll two camera at once after the usb_cam repo updates