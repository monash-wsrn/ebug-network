
## Organise a host device for the Visualisation
***Ensure all virtual/unused network adapaters are disabled on the host device***

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
5. Install [XMing for Windows](https://sourceforge.net/projects/xming/) or [XQuartz for macOS](https://www.xquartz.org/)

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


## Deploy the container (Visualisation Mode)
1. Run the ***XLaunch*** to capture the visualisation window from the Docker container. For more information, visit [here](https://medium.com/@rndonovan1/running-pygame-gui-in-a-docker-container-on-windows-cc587d99f473).
2. Run the ebug container as a visualisation
    ```sh
    # Supply environment variables, such as DISPLAY_SCALE to scale up from 200px by 200px
    docker run --net host --ipc host --pid host -e DISPLAY_SCALE=3 --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ros2 launch ebug_visualisation ebug_visualisation.launch.py
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