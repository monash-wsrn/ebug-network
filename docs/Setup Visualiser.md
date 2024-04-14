
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



## Deploy the container (Visualisation Mode)
1. Follow the instructions [here](/docs/Deploy%20Containers.md) to build the container.
2. Run the ***XLaunch*** to capture the visualisation window from the Docker container. For more information, visit [here](https://medium.com/@rndonovan1/running-pygame-gui-in-a-docker-container-on-windows-cc587d99f473).
3. Run the ebug container as a visualisation
    ```sh
    # Supply environment variables, such as DISPLAY_SCALE to scale up from 200px by 200px
    docker run --net host --ipc host --pid host -e DISPLAY_SCALE=3 --rm -it ebug
    
    # In the containers interactive terminal, you can launch the ROS2 package
    ./launch visualiser
    ```

*This will run the container using host networking, interprocess communication, and process ID domains.*
*It will be run as the **root** user.*
