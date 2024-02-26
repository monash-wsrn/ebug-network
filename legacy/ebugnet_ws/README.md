This is the workspace for the entire ebug system (remote brain + ebugs).

We have a single ROS system albeit it will be a distributed system
that will be running over a set of heterogeneous entities. It's likely
that even though we will only execute some aspects of the code on the
central server/ robots, it will still need to know about code on the
robots. For example, maybe we have some custom message definitions
defined in the server, which the robot code needs to include to
receive messages from this. Having all code in a single workspace
allows ros to automatically sort our dependencies etc. and makes
compilation trivial especially when it is for different architectures
(we just check out code, catkin_make and source the workspace, and
everything just works).

Non-ROS material/software will be kept in separate
folders... for example the RGB LED board PCB design, Arduino code
(Romi control board low-level code or chassis 3D print design etc...)

