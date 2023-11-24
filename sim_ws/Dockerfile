FROM osrf/ros:humble-desktop

RUN sudo apt-get upgrade
RUN sudo apt-get update

RUN sudo apt-get install -y ros-humble-ament-cmake-clang-format

RUN mkdir -p "/sim_ws/"
WORKDIR "/sim_ws/"

RUN echo "colcon build" >> /opt/ros/humble/setup.bash
RUN echo "source /sim_ws/install/setup.bash" >> /opt/ros/humble/setup.bash

# TODO remove hardcoded sim launch for prod, useful for development though
RUN echo "ros2 launch sim_central sim_central.launch.py" >> /opt/ros/humble/setup.bash

COPY "./" "/sim_ws/"
RUN rosdep install -i --from-path src --rosdistro humble -y

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]