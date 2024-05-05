
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
5. Install [VcXsrv for Windows](https://github.com/marchaesen/vcxsrv/releases/tag/21.1.10) or [XQuartz for macOS](https://www.xquartz.org/)
6. Run the ***XLaunch*** to capture the visualisation window from the Docker container. 
    For more information, visit [here](https://medium.com/@rndonovan1/running-pygame-gui-in-a-docker-container-on-windows-cc587d99f473).


## Running the containers
Follow the instructions [here](/docs/Deploy%20Containers.md) to build and deploy the relevant container(s).
