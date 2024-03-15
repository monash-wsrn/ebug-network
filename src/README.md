## Overview
![Component Architecure](component_architecture.png)

## Principal
This ROS2 container implements Boids in a global scope. 
Agents should query the Principal to determine their next course.

The Principal must be run on the central compute server.

```sh
    # Build the principal container, with src as the working directory
    sudo docker build -t ebug_principal . -f Principal.Dockerfile

    # Run the principal container
    sudo docker run --env UID=$(id -u) --env GID=$(id -g) -it ebug_principal

    # In the container run the Principal
    ros2 launch ebug_principal ebug_principal.launch.py
```

## Agent
This ROS2 container implements localisation component(s)
An Agent should be instantiated for each corresponding Client.

The Agent can either be run on each robot' Raspberry Pi, or on the central compute server.

The robot ID should be passed in as an environment variable, `ROBOT_ID`, using docker run or docker-compose.

The robot algorithm can also be passed in as an environment variable, `ROBOT_ALGO`, using docker run or docker-compose.
If left blank, ROBOT_ALGO will default to 'BoidsService'.

```sh
    # Build the agent container, with src as the working directory
    sudo docker build -t ebug_agent . -f Agent.Dockerfile

    # Run the agent container
    sudo docker run --env UID=$(id -u) --env GID=$(id -g) -e ROBOT_ID='robot_0' -e ROBOT_ALGO='BoidsService' -it ebug_agent 

    # In the container run the agent
    ros2 launch ebug_agent ebug_agent.launch.py
```

## Client
This ROS2 container implements camera, polling, and movement component(s)
A Client should be instantiated for each robot in the swarm, corresponding with an Agent instance.

The Principal must be run individually on each robot in the swarm.

The robot ID should be passed in as an environment variable, `ROBOT_ID`, using docker run or docker-compose.

By default, camera polling is disabled, it can be enabled by setting the environment variable `CAMERA_POLLING` to `enabled`.
Doing so will enable the remaining three cameras on the robot, as well as the polling logic to alternate between them.

*Please note: Currently the I2C connection has not been validated.*

```sh
    # Build the client container, with src as the working directory
    docker build -t ebug_client . -f Client.Dockerfile

    # Run the client container, passing through I2C-1
    # ebug_client.util.AStar creates an SMBus on I2C-1
    docker run --env UID=$(id -u) --env GID=$(id -g) -e ROBOT_ID='robot_0' --device /dev/i2c-1 -it ebug_client

    # In the container run the client
    ros2 launch ebug_client ebug_client.launch.py
```


