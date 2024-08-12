
## Organise a host device for the Principal
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


## Running the containers
Follow the instructions [here](/docs/Deploy%20Containers.md) to build and deploy the relevant container(s).