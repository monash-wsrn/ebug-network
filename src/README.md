## Overview
![Component Architecure](component_architecture.png)

## Principal
This ROS2 container implements Boids in a global scope. 
Agents should query the Principal to determine their next course.

The Principal must be run on the central compute server.

```sh
    # Build the principal container, with src as the working directory
    docker build -t ebug_principal . -f Principal.Dockerfile

    # Run the principal container
    docker run -it ebug_principal
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
    docker build -t ebug_agent . -f Agent.Dockerfile

    # Run the agent container
    docker run -e ROBOT_ID='robot_0' -e ROBOT_ALGO='BoidsService' -it ebug_agent 
```

## Client
This ROS2 container implements camera, polling, and movement component(s)
A Client should be instantiated for each robot in the swarm, corresponding with an Agent instance.

The Principal must be run individually on each robot in the swarm.

The robot ID should be passed in as an environment variable, `ROBOT_ID`, using docker run or docker-compose.

*Please note: Currently the I2C connection has not been validated.*

```sh
    # Build the client container, with src as the working directory
    docker build -t ebug_client . -f Client.Dockerfile

    # Run the client container, passing through I2C-1
    # ebug_client.util.AStar creates an SMBus on I2C-1
    docker run -e ROBOT_ID='robot_0' --device /dev/i2c-1 -it ebug_client 
```


