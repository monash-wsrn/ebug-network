FROM osrf/ros:humble-desktop

RUN sudo apt-get upgrade
RUN sudo apt-get update

# ROS packages
RUN sudo apt-get install -y \
    ros-humble-ament-cmake-clang-format \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-usb-cam \
    ros-humble-image-pipeline \
    ros-humble-apriltag-ros

# Python packages
COPY python_requirements.txt .
RUN pip install -r python_requirements.txt

RUN mkdir -p "/sim_ws/"
WORKDIR "/sim_ws/"

COPY "./" "/sim_ws/"
RUN rosdep install -i --from-path src --rosdistro humble -y

RUN bash -c "source /opt/ros/humble/setup.bash; colcon build"

RUN echo "source /sim_ws/install/setup.bash" >> /opt/ros/humble/setup.bash

# TODO remove hardcoded sim launch for prod, useful for development though
RUN echo "ros2 launch sim_central sim_central.launch.py" >> /opt/ros/humble/setup.bash

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]