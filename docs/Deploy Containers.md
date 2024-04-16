
*Containers are run using host networking, interprocess communication, and process ID domains.*
*This means it will be run as the **root** user. To access it from a host device, please view last section.*

## Build the ebug container
1. Clone the EBug Git Repository
    ```sh
    cd ~
    git clone https://github.com/monash-wsrn/ebug-network.git
    ```
2. Build the EBug container
    ```sh
    cd ~/ebug-network/ros/src
    docker build -t ebug .
    ```


## Deploy the container (Client Mode)
1. Run the ebug container as a client
    ```sh
    # Supply environment variables using the -e flag. The available variable defaults are:
    #    ROBOT_ID (String):         'default'
    #    CAMERA_POLLING (String):   'disable'   // Enable/disable polling additional cameras
    
    # Supply host devices using the --device flag. The mappable devices are:
    #    /dev/i2c-1  (Required)
    #    /dev/video0 (Required)
    #    /dev/video1 (Optional)
    #    /dev/video2 (Optional)
    #    /dev/video3 (Optional)

    docker run --net host --ipc host --pid host -e \
        ROBOT_ID='robot_0' --device /dev/i2c-1 --device /dev/video0 --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ./launch client
    ```


## Deploy the container (Agent Mode)
1. Run the ebug container as an agent
    ```sh
    # Supply environment variables using the -e flag. The available variable defaults are:
    #    ROBOT_ID (String):         'default'
    #    ROBOT_ALGO (String):       'BoidsService'

    docker run --net host --ipc host --pid host -e ROBOT_ID='robot_0' --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ./launch agent
    ```


## Deploy the container (Principal Mode)
1. Run the ebug container as a principal
    ```sh
    # Supply environment variables using the -e flag. The available variable defaults are:
    #    ----

    docker run --net host --ipc host --pid host -e --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ./launch principal
    ```


## Deploy the container (Pseudo Mode)
1. Run the ebug container as a pseudo
    ```sh
    # Supply environment variables using the -e flag. The available variable defaults are:
    #    ROBOT_ID (String):         'default'
    #    ROBOT_ALGO (String):       'BoidsService'
    #    TICK_RATE (Float):         25.0    // Simulated FPS, should match actual cameras
    #    START_POSX (Float):        0.0
    #    START_POSY (Float):        0.0
    #    START_YAW (Float):         0.0

    docker run --net host --ipc host --pid host -e \
        ROBOT_ID='robot_0' -e START_POSX=50 -e START_POSY=50 --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ./launch pseudo
    ```


## Deploy the container (Visualiser Mode)
1. Run the ebug container as a visualiser
    ```sh
    # Supply environment variables using the -e flag. The available variable defaults are:
    #    DISPLAY_SCALE (Integer):   3   // Scale the 200x200 px arena display

    docker run --net host --ipc host --pid host -e DISPLAY_SCALE=3 --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ./launch visualiser
    ```


## Access the container ROS2 network
1. Access the ROS2 network of the container(s) from the host device
    ```sh
    # Enter bash as the root user, which is running the containers
    sudo bash

    export ROS_DOMAIN_ID=13
    source /opt/ros/humble/setup.bash

    # The host device now has ROS2 configured to connect to the containers

    # Exit the privelleged bash after 
    exit
    ```


