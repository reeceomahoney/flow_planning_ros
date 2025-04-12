##############
# Base image #
##############
FROM osrf/ros:noetic-desktop-full AS base

ARG CATKIN_WORKSPACE_DIR="/catkin_ws"

LABEL org.opencontainers.image.authors="reeceo@robots.ox.ac.uk"
LABEL description="Flow Planning ROS Docker"
LABEL version="1.0"

WORKDIR ${CATKIN_WORKSPACE_DIR}
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  git \
  python3-catkin-tools \
  python3-osrf-pycommon \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  ros-"${ROS_DISTRO}"-compressed-image-transport \
  ros-"${ROS_DISTRO}"-compressed-depth-image-transport \
  ros-"${ROS_DISTRO}"-image-transport \
  && rm -rf /var/lib/apt/lists/*

# Add additional installation instructions here...

ENV DEBIAN_FRONTEND=dialog


#####################
# Development image #
#####################
FROM base AS dev

ARG CATKIN_WORKSPACE_DIR="/catkin_ws"
ARG USERNAME="developer"
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  ack \
  bmon \
  cloc \
  gdb \
  htop \
  iperf3 \
  iputils-ping \
  mlocate \
  net-tools \
  psmisc \
  tmux \
  xterm \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  python3-vcstool \
  ros-"${ROS_DISTRO}"-rqt-common-plugins \
  ros-"${ROS_DISTRO}"-rqt-robot-plugins \
  ros-"${ROS_DISTRO}"-rviz \
  && rm -rf /var/lib/apt/lists/*

# Install additional developer tools here...
RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  ros-"${ROS_DISTRO}"-libfranka \
  ros-"${ROS_DISTRO}"-franka-ros \
  ros-"${ROS_DISTRO}"-panda-moveit-config \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  ros-"${ROS_DISTRO}"-gazebo-ros-control \ 
  ros-"${ROS_DISTRO}"-rospy-message-converter \
  ros-"${ROS_DISTRO}"-effort-controllers \
  ros-"${ROS_DISTRO}"-joint-state-controller \
  ros-"${ROS_DISTRO}"-moveit \
  ros-"${ROS_DISTRO}"-moveit-commander \
  ros-"${ROS_DISTRO}"-moveit-visual-tools \
  ros-"${ROS_DISTRO}"-combined-robot-hw \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
  python3 \
  python3-pip \
  build-essential \
  swig \
  libcppunit-dev \
  && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
  numpy==1.23 \ 
  numpy-quaternion==2020.5.11.13.33.35 \
  future \
  numba \
  torch

RUN apt-get update \
  && apt-get install --no-install-recommends -y sudo \
  && rm -rf /var/lib/apt/lists/* \
  && addgroup --gid ${GID} ${USERNAME} \
  && adduser --disabled-password --gecos '' --uid ${GID} --gid ${GID} ${USERNAME} \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chown -R ${UID}:${GID} /home/${USERNAME} \
  && chown -R ${UID}:${GID} ${CATKIN_WORKSPACE_DIR}

ENV DEBIAN_FRONTEND=dialog

RUN echo "alias rsource='source ${CATKIN_WORKSPACE_DIR}/devel/setup.bash'" >> /home/${USERNAME}/.bash_aliases \
  && echo "alias rbuild='(cd ${CATKIN_WORKSPACE_DIR} && catkin build)'" >> /home/${USERNAME}/.bash_aliases \
  && echo "alias rclean='(cd ${CATKIN_WORKSPACE_DIR} && catkin clean -y)'" >> /home/${USERNAME}/.bash_aliases \
  && echo "rsource || source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
