# Create container
FROM osrf/ros:humble-desktop
ENV ENV_ROS_PKG="ebug_agent"


RUN sudo apt-get upgrade
RUN sudo apt-get update

# ROS packages
RUN sudo apt-get install -y \
    ros-humble-ament-cmake-clang-format \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-image-pipeline \
    ros-humble-apriltag-ros


RUN mkdir -p "/ws/src/"

COPY "./ebug_interfaces" "/ws/src/ebug_interfaces"
COPY "./${ENV_ROS_PKG}" "/ws/src/${ENV_ROS_PKG}"

WORKDIR "/ws/"

RUN pip install -r "/ws/src/${ENV_ROS_PKG}/python_requirements.txt"
RUN rosdep install -i --from-path src --rosdistro humble -y

RUN bash -c "source /opt/ros/humble/setup.bash; colcon build"

RUN echo "source /ws/install/setup.bash" >> /opt/ros/humble/setup.bash

ENTRYPOINT ["/ros_entrypoint.sh"]

# TODO maybe remove hardcoded launch for prod, useful for development though
CMD ["bash", "-c", "ros2 launch ${ENV_ROS_PKG} ${ENV_ROS_PKG}.launch.py"]