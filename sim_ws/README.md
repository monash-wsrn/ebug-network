## Getting Started
Open a terminal in this **sim_ws** folder.
```bash
# Build the container from the Dockerfile, this will take a few minutes the first time
docker build -t sim-central .

# Run the built container, this will automatically build the ros workspace and launch the sim_central package
docker run -it sim-central
```

## Structure
The workspace contains two packages; **sim_central** and **sim_interfaces**.

The packages are intended to control the simulation, but are considerate of the physical implementation on robots also. Therefore it should be relatively easily interopable with the actual robots.

**sim_central** is the primary launch point of the simulation package, the *sim_central.launch.py* file defines the robot controllers and central control algorithm.

**sim_interfaces** contains no logic, but defines custom interfaces. For more details about custom interfaces please visit the [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html). 