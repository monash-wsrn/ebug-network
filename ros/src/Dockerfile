ARG DOMAIN_ID=13

# Create container
FROM ros:humble

RUN sudo apt-get upgrade
RUN sudo apt-get update

# Qt5 & I2C low-level depedency
RUN sudo apt-get install -y \
    libqt5widgets5 \
    libi2c-dev
    
# Python packages
RUN sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-smbus

# ROS Base Packages
RUN sudo apt-get install -y \
    ros-humble-rclcpp-components \
    ros-humble-ament-cmake-clang-format
    
# ROS TF / EKF Packages
RUN sudo apt-get install -y \
    ros-humble-tf2-tools \
    ros-humble-robot-localization

# ROS Image Processing Packages
RUN sudo apt-get install -y \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-pipeline

# ROS Additional Packages
RUN sudo apt-get install -y \
    ros-humble-apriltag-ros \
    ros-humble-usb-cam \
    ros-humble-grid-map \
    ros-humble-diff-drive-controller

ENV ROS_DOMAIN_ID=${DOMAIN_ID:-13}
ENV DISPLAY="host.docker.internal:0.0"
ENV SDL_AUDIODRIVER="dsp"


RUN mkdir -p "/ws/src/"
WORKDIR "/ws/"

COPY "./python_requirements.txt" "/ws/src/python_requirements.txt"
RUN pip install -r "/ws/src/python_requirements.txt"


COPY "./" "/ws/src/"

RUN rosdep install -i --from-path src --rosdistro humble -y
RUN bash -c "source /opt/ros/humble/setup.bash; colcon build"

RUN sudo mv /ws/src/launch /ws/launch && sudo chmod 777 /ws/launch
RUN echo "source /ws/install/setup.bash" >> /opt/ros/humble/setup.bash

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD /bin/bash