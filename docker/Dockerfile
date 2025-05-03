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
WORKDIR $WORKSPACE_DIR

# Install dependencies via rosdep
RUN rosdep init && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Copy full source in
COPY . src/

# Build with colcon
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

# Change ownership to user
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

# Switch to non-root user
USER $USERNAME

# Final working dir
WORKDIR $WORKSPACE_DIR

# Start interactive shell
CMD ["/bin/bash"]