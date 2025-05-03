# Base image: ROS 2 Humble on Ubuntu 22.04
FROM ros:humble-ros-core

# Install dev tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    python3-rosdep \
    git \
    wget \
    curl \
    nano \
    vim \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Setup non-root user
ARG USERNAME=mcav
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ROS 2 Environment setup
ENV ROS_DISTRO=humble
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Set up workspace structure
ENV WORKSPACE_DIR=/home/$USERNAME/ros2_ws
RUN mkdir -p $WORKSPACE_DIR/src
WORKDIR $WORKSPACE_DIR/src

# Clone external repositories via build args
ARG SD_VEHICLE_INTERFACE_REPO=https://github.com/Monash-Connected-Autonomous-Vehicle/SD-VehicleInterface.git
ARG AUTOWARE_MSGS_REPO=https://github.com/autowarefoundation/autoware_msgs.git

RUN git clone $SD_VEHICLE_INTERFACE_REPO \
    && git clone $AUTOWARE_MSGS_REPO

# Initialize rosdep (won't install dependencies yet)
RUN rosdep init && rosdep update

# Change ownership of workspace
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

# Switch to non-root user
USER $USERNAME

# Final working dir
WORKDIR $WORKSPACE_DIR

# Start interactive shell
CMD ["/bin/bash"]