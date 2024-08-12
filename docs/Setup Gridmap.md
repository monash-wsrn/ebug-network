# Ebug Gridmap Interface Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)
4. [Running the Interface](#running-the-interface)
5. [Configuration](#configuration)
6. [Gridmap Visualizer for the Projector](#gridmap-visualizer-for-the-projector)
7. [Troubleshooting](#troubleshooting)
8. [Contact](#contact)

## Introduction
The ebug Gridmap Interface is designed to provide a visual representation of robot movements and interactions on a grid. This document serves as a guide for setting up, running, and maintaining the interface. The setup for the gridmap is not included in the docker container and should be built locally. 

## Prerequisites
- Ubuntu 22.04 or later
- ROS 2 Humble
- Python 3.8 or later
- `grid_map` package by ANYbotics
- Docker
- Projector for visual output (dimensions: 1.92m x 1.08m)

## Installation


### Install Dependencies
```sh
sudo apt-get update
sudo apt-get install python3-pip
pip3 install -r requirements.txt
```

### Build
Ensure that you have cloned the repository.
```sh 
cd ebug-network/ros/src
colcon build --symlink-install 
source install/setup.bash
```

## Running the Interface
Ensure that the gridmap is connected with the network
```sh
sudo bash
cd /ebug-network/ros/src 
source install/setup.bash 
source opt/ros/humble/setup.bash 
export ROS_DOMAIN_ID=13 
```
#### Running the Visualiser 
```sh
ros2 launch ebug_gridmap.launch.py
```


## Configuration 
The following parameters can be adjusted in the `ebug_base/src/gridmap_controller.cpp` file:

#### Gaussian Filter 
- `sigma_x`: Standard deviation of the Gaussian function along the x-axis. (default: `0.5`)
- `sigma_y`: Standard deviation of the Gaussian function along the y-axis. (default: `0.5`)
- `amplitude`: Amplitude of the Gaussian function. (default: `0.0001`)

#### Gridmap Size
- `resolution`: Grid resolution (default: `0.03 meters`)
- `width`: Width of the gridmap (default: `19.20 meters`)
- `height`: Height of the gridmap (default: `10.80 meters`)
- `scale_down`: Scale down factor of the entire gridmap (default: `5.0`, change in `ebug/launch/gridmap.launch.py`)

To modify these parameters, open the `ebug_base/src/gridmap_controller.cpp` file and adjust the values as needed.

#### RVIZ Config Settings 
```sh 
Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Scale: 496
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Angle: 0
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz)
      X: 0
      Y: 0
```
The rviz file can be found here: `ebug/RVIZ/gridmap.rviz`

## Gridmap Visualizer for the Projector

#### Source Files 
The gridmap node is written in the file `ebug_base/src/gridmap_controller.cpp`. This generates the visualization overlay that uses the custom message `msg/RobotPose.msg`.

#### Gaussian Filter 
`sigma_x`, `sigma_y`, and `amplitude` determine the size of the encompassing radius and the gradient strength, respectively. Note that amplitude will also increase the height of the cell, so large amplitudes will change the visualization. A min-max filter is applied to prevent the visualisation from diminishing or exploding. 

#### Arena 
The scale factor is used to change the overall resolution of the grid, with the current grid set at 19.20m x 10.80m at 0.03m resolution. This is then reduced all by a factor of 5 for performance reasons. The overall size of the arena is 1.41m x 2.16m, and adjusted using keystone correction on the projector. 

#### RVIZ Display 
The current RVIZ view uses the orthogonal view at a scale of 496, using the `scale_down` factor of 5. Laptop resolution is 1920x1080 and projector resolution is 1920x1080.

## Troubleshooting

### Common Issues
1. **RVIZ not displaying the gridmap**
    - A common problem is the actual gridmap does not show. This case is usually due to the gridmap is not connected to the network. To resolve this issue: 
    ```sh 
    sudo bash 
    cd ebug-network/ros/src 
    source install/setup.bash 
    source opt/ros/humble/setup.bash 
    export ROS_DOMAIN_ID=13 
    ```
    

2. **The grid is not aligned properly**

    This can be multiple reasons:
    1. RVIZ view config is not aligned properly - If the scale_down factor in the node file has changed, the scale factor in the view panel of RVIZ will need to be adjusted such that it is aligned with the projector and laptop's resolution. 
    2. Laptop and projector resolution does not match - Ensure that both the laptop and the projector resolution is set at 1920x1080. 




### Debugging Tips
- **Check ROS node connections**:
    Use `rqt_graph` to visualize the ROS node graph and ensure that all nodes are connected correctly.

    ```sh
    rqt_graph
    ```

- **Verify topic messages**:
    Use `ros2 topic echo /<topic>` to check the messages being published on a specific topic. For example, to check the `RobotPose` messages:

    ```sh
    ros2 topic echo /global_poses
    ```

- **Inspect logs**:
    Check the logs for any errors or warnings that might indicate issues with the gridmap controller or other nodes.

    ```sh
    ros2 node info /GridmapController
    ```

- **Check parameter values**:
    Ensure that all parameters are set correctly. You can list the parameters of the node using:

    ```sh
    ros2 param list /GridmapController
    ```

    To get the value of a specific parameter:

    ```sh
    ros2 param get /GridmapController <parameter_name>
    ```

### Additional Tips
- **Rebuild the workspace**:
    If you make any changes to the code or configuration files, ensure to rebuild the workspace:

    ```sh
    colcon build --symlink-install
    source install/setup.bash
    ```


By following these troubleshooting steps, you should be able to identify and resolve common issues that might arise while running the ebug gridmap interface.

## Contact
For further assistance, contact the visualisation lead at richiepham0@gmail.com.