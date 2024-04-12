
## Organise a host device for the Principal and Agent(s)
***Ensure all virtual/unused network adapaters are disabled on the host device***
***Windows cannot be used for this, as WSL introduces a layer of network virtualisation***

1. Configure wireless connection <br>
    SSID:       *lightrobot* <br>
    Password:   *lightrobot2023* <br>
2. Install Docker
    Platform-specific instructions [here](https://docs.docker.com/engine/install/)
3. Install Git
    Platform-specific instructions [here](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
4. Add your user to Docker user group
    ```sh
    sudo usermod -aG docker ${USER}
    su - ${USER}
    ```

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


## Deploy the container (Agent Mode)
1. Run the ebug container as an agent
    ```sh
    # Supply environment variables, such as ROBOT_ID, or ROBOT_ALGO, using the -e flag
    docker run --net host --ipc host --pid host -e ROBOT_ID='robot_0' --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ros2 launch ebug_agent ebug_agent.launch.py
    ```

*This will run the container using host networking, interprocess communication, and process ID domains.*
*It will be run as the **root** user.*


## Deploy the container (Principal Mode)
1. Run the ebug container as a principal
    ```sh
    docker run --net host --ipc host --pid host -e --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ros2 launch ebug_principal ebug_principal.launch.py
    ```

*This will run the container using host networking, interprocess communication, and process ID domains.*
*It will be run as the **root** user.*


## Access the container ROS2 network
1. Access the ROS2 network of the container(s) from the host device
    ```sh
    # Enter bash as the root user, which is running the containers
    sudo bash

    export ROS_DOMAIN_ID=13
    source /opt/ros/humble/setup.bash

    # The host device now has ROS2 configured to connect to the containers

    # Exit the root bash after 
    exit
    ```