ARG ROS_DOMAIN_ID=13

# Create container
FROM osrf/ros:humble-desktop

RUN sudo apt-get upgrade
RUN sudo apt-get update

# ROS packages
RUN sudo apt-get install -y \
    ros-humble-ament-cmake-clang-format \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-image-pipeline \
    ros-humble-apriltag-ros \
    ros-humble-usb-cam

# Python packages
RUN sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-smbus \
    i2c-tools


RUN mkdir -p "/ws/src/"

COPY "./" "/ws/src/"

WORKDIR "/ws/"

ENV ROS_DOMAIN_ID=$ROS_DOMAIN_ID

RUN pip install -r "/ws/src/python_requirements.txt"
RUN rosdep install -i --from-path src --rosdistro humble -y

RUN bash -c "source /opt/ros/humble/setup.bash; colcon build"

RUN echo "source /ws/install/setup.bash" >> /opt/ros/humble/setup.bash

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD /bin/bash

# TODO maybe remove hardcoded launch for prod, useful for development though
#CMD ["bash", "-c", "ros2 launch ${ENV_ROS_PKG} ${ENV_ROS_PKG}.launch.py"]