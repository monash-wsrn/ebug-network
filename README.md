# A robot network for exploring power and potential of cooperative robotics

We are building a network of mobile robots to explore the power and potential of cooperative robotics. Our broad aim is to investigate multiagent systems and various sensor fusion mechanisms, with a particular focus on visual sensing and multi-camera processing in a distributed manner. There are quite a lot of interesting avenues for research.

The first stage is to create the hardware and software infrastructure that will enable the robots to localize themselves and others around them. Following this, as a demonstration, we will implement the  [Boids algorithm](https://en.wikipedia.org/wiki/Boids). The robot network will then behave like a school of fish or a flock of birds.

We have intentionally chosen off-the-shelf hardware to keep the costs
down and shorten the time to reach what we want to achieve. Each robot consists of:
* A [Romi](https://www.pololu.com/category/203/romi-chassis-kits) chassis kit (including motors, wheels, encoders etc.)
* A Raspberry Pi 4 single board computer
* A Romi 32U4 control board, and
* 4 USB cameras.

Project details and documentation can be found in its [Wiki pages](https://github.com/monash-wsrn/ebug-network/wiki) (we are updating them quite frequently).
