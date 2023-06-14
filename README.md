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


We have deliberately chosen off-the-shelf hardware to keep the costs
down. Each robot consists of:
* A romi chassis kit (including motors, wheels, encoders etc.)
* A Raspberry Pi 4 single board computer
* A Raspberry Pi battery hat
* A romi 32U4 control board
* 2 USB cameras for robots to see their surroundings, and
