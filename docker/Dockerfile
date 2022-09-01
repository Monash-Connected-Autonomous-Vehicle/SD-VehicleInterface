FROM ros:noetic-ros-core

# Dev tools
# Try to keep ROS-related dependencies out of here and inside package.xml to be installed with rosdep instead
RUN apt-get -y update && apt-get install -y \
    python3-rosdep \
    python3-pip \
    build-essential \
    vim \
    nano \
    git \
    wget \
    tmux \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create a non-root user with sudo privileges
ARG USERNAME=mcav
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Prepare to build the project
ENV HOMEDIR=/home/${USERNAME}
ENV CATKIN_WS=${HOMEDIR}/mcav_ws
ENV SOURCE_DIR=${CATKIN_WS}/src/sd_vehicle_interface
WORKDIR ${CATKIN_WS}
COPY ./vehicle_interface/package.xml ${SOURCE_DIR}/vehicle_interface/package.xml
# Project-specific dependency install
RUN apt-get update && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
# Build project
COPY . ${SOURCE_DIR}
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd ${CATKIN_WS}; catkin_make -DCMAKE_BUILD_TYPE=Release'

# Change owner of the files to non-root user
RUN chown -R ${USERNAME} /home/${USERNAME}
# Add source commands to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source ~/mcav_ws/devel/setup.bash" >> /home/${USERNAME}/.bashrc
# Change prompt to show we are in a docker container
RUN echo "export PS1='\[\e]0;\u@docker: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@docker\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc
# Add tmux configuration file
COPY docker/.tmux.conf /home/${USERNAME}/.tmux.conf

RUN echo "127.0.0.1	`hostname`.localdomain	`hostname`" >> /etc/hosts

WORKDIR ${SOURCE_DIR}
CMD /bin/bash
USER $USERNAME