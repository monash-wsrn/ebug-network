# A robot network for exploring power and potential of cooperative robotics

We are building a network of mobile robots for exploring the
power and potential of cooperative robotics. Our broad aim is to
investigate multiagent systems and various sensor fusion
mechanisms. We are particularly interested in visual sensing and
multi-camera processing in a distributed manner. There are quite a lot
of interesting avenues for research.

The first stage is to create the hardware and software infrastructure
that will allow the robots localize themselves and others around
them. After this, as a demonstration, we will implement [Boids
algorithm](https://en.wikipedia.org/wiki/Boids). The robot network
will then behave like a school of fish or flock of birds.


We have intentionally chosen off-the-shelf hardware to keep the costs
down. Each robot consists of:
* A [Romi](https://www.pololu.com/category/203/romi-chassis-kits) chassis kit (including motors, wheels, encoders etc.)
* A Raspberry Pi 4 single board computer
* A Romi 32U4 control board
* 2 USB cameras for robots to see their surroundings, and

Project details and documentation can be found in its [https://github.com/monash-wsrn/ebug-network/wiki](Wiki pages) (we are updating them quite frequently).

Below are copied from the old README.md, we will transfer the contents
to Wiki soon. 

After putting the robot kit together (see romi
chassis assembly pages), follow the below links in ascending order to
complete a robot's basic setup:
1. [Raspberry Pi installation guide](https://github.com/monash-wsrn/ebug_network/blob/main/raspi_n_laptop_setup.md)
2. [Camera calibration](https://github.com/monash-wsrn/ebug_network/blob/main/calibration_guide.md)
3. [Setting up Romi board](https://github.com/monash-wsrn/ebug_network/blob/main/setup_raspi_pololu.md)
4. [Setting up the localization](https://github.com/monash-wsrn/ebug_network/blob/main/apriltag_localization_guide.md)


