# Create container
FROM osrf/ros:humble-desktop
ENV ENV_ROS_PKG="ebug_client"


RUN sudo apt-get upgrade
RUN sudo apt-get update

# Dependencies
RUN sudo apt-get install -y \
    ros-humble-ament-cmake-clang-format \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-robot-localization \
    ros-humble-usb-cam \
    i2c-tools \
    # python3 \
    # python3-flask \
    python3-smbus 
    


RUN mkdir -p "/ws/src/"

COPY "./ebug_interfaces" "/ws/src/ebug_interfaces"
COPY "./${ENV_ROS_PKG}" "/ws/src/${ENV_ROS_PKG}"

WORKDIR "/ws/"

RUN pip install -r "/ws/src/${ENV_ROS_PKG}/python_requirements.txt"
RUN rosdep install -i --from-path src --rosdistro humble -y

RUN bash -c "source /opt/ros/humble/setup.bash; colcon build"

RUN echo "source /ws/install/setup.bash" >> /opt/ros/humble/setup.bash


# RUN echo "export ROS_DOMAIN_ID=13" >> ~/.bashrc

## Setup I2C Connection
# RUN /bin/bash -c '\
#     groupadd i2c; \
#     chown :i2c /dev/i2c-1; \
#     chmod g+rw /dev/i2c-1; \
#     usermod -aG i2c ubuntu; \
#     -s'
# RUN echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules


ENTRYPOINT ["/ros_entrypoint.sh"]

# TODO maybe remove hardcoded launch for prod, useful for development though
CMD ["bash", "-c", "ros2 launch ${ENV_ROS_PKG} ${ENV_ROS_PKG}.launch.py"]
