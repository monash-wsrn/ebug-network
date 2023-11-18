# Base Image
FROM ros:humble-ros-base


# ENTRYPOINT ["python", "app.py"]

# Dependencies
RUN apt-get -y update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-usb-cam \
    ros-humble-image-pipeline \
    ros-humble-apriltag-ros \
    i2c-tools \
    python3 \
    python3-flask \
    python3-smbus 

# Python Libraies
COPY python_requirements.txt .
RUN pip install -r python_requirements.txt

# Add source commands to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=13" >> ~/.bashrc

# Setup I2C Connection
# RUN /bin/bash -c '\
#     groupadd i2c; \
#     chown :i2c /dev/i2c-1; \
#     chmod g+rw /dev/i2c-1; \
#     usermod -aG i2c ubuntu; \
#     -s'
# RUN echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules


# # Make the prompt a little nicer
# RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc  

# Project specific dependencies
ENV WORKSPACE=/networked_robotics
WORKDIR ${WORKSPACE}/ros2_localization
COPY ./calibration ${WORKSPACE}/calibration
COPY ./ros2_localization/src ${WORKSPACE}/ros2_localization/src

# Build the project
RUN /bin/bash -c '\
    cd ${WORKDIR}/src; \
    colcon build'

# ENTRYPOINT /ros_entrypoint.sh
# CMD /bin/bash