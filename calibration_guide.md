# Camera Calibration Guide
## Important Notes
Calibration need to be done on an Ubuntu 22.04 device with GUI

## Related tutorials
https://navigation.ros.org/tutorials/docs/camera_calibration.html

Everything is the same, Except in step 2
```
sudo apt install ros-humble-image-pipeline
```

If any dependencies error do:
```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
sudo apt-get install libboost-all-devcol
```

## Install usb_cam
```
sudo apt get install ros-humble-usb-cam
```
To launch usb_cam:
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0"
```
If the device have build in webcam, replace video0 with video2
### follow the https://navigation.ros.org/tutorials/docs/camera_calibration.html to complete the calibration

```
ros2 run camera_calibration cameracalibrator --size 4x7 --square 0.05 --ros-args -r image:=/image_raw 

```

## Important Notes: Remember to mark the usb end of the camera to indicate which camera corresponds to which .yaml file
The ID of the usb port is as follow
![Alt text](resource/img/usb_port_id.jpg)
