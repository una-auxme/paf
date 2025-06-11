ARG BASE_FLAVOUR=cuda

FROM rocm/dev-ubuntu-24.04:6.4 AS agent-base-rocm

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y software-properties-common

# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository -y ppa:kisak/turtle && apt-get update && \
  apt-get upgrade -y && apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda
# FROM nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu20.04 AS agent-base-cuda
FROM ubuntu:24.04 AS agent-base-cuda
ARG NVIDIA_DRIVER_VERSION=550
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.17.5/docker-specialized.html#nvidia-require-constraints
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_REQUIRE_DRIVER="cuda>=12.0 driver>=${NVIDIA_DRIVER_VERSION}"

# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y libvulkan1 vulkan-tools libnvidia-gl-${NVIDIA_DRIVER_VERSION} \
  nvidia-cuda-toolkit

############################################
##### The main agent image starts here #####
############################################
FROM agent-base-${BASE_FLAVOUR} AS agent

ENV DEBIAN_FRONTEND=noninteractive

#######################
# Install ROS 2 Jazzy # https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#######################
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && apt-get install -y software-properties-common

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository universe

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt install -y curl && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions

# Required for rosdep since python 3.11
ENV PIP_BREAK_SYSTEM_PACKAGES=1
# Setup rosdep
RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && \
  bash -c "\
  source /opt/ros/${ROS_DISTRO}/setup.bash && \
  rosdep init"

#################################
# Create the internal workspace #
#################################
# This workspace at INTERNAL_WORKSPACE_DIR contains
# - PAF_ROS_WS: a ROS2 workspace for the PAF project packages
# - CARLA_ROS_WS: a ROS2 workspace for the carla ros bridge packages
# INTERNAL_WORKSPACE_DIR is editable by the user with PAF_UID and PAF_GID

ENV PYTHON_VERSION=3.12
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && apt-get install -y \
  python${PYTHON_VERSION} python${PYTHON_VERSION}-venv python3-pip python-is-python3 \
  sudo

# Create the paf user
ARG PAF_USERNAME=paf
ARG PAF_UID=1000
ARG PAF_GID=1000

# Allow duplicated uids with -o
RUN groupadd --gid ${PAF_GID} ${PAF_USERNAME} -o \
  && useradd --uid ${PAF_UID} --gid $PAF_GID -m -s /bin/bash ${PAF_USERNAME} -o \
  && echo ${PAF_USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${PAF_USERNAME} \
  && chmod 0440 /etc/sudoers.d/${PAF_USERNAME}

# Build internal workspace
ENV INTERNAL_WORKSPACE_DIR=/internal_workspace
RUN mkdir -p ${INTERNAL_WORKSPACE_DIR} && \
  chown ${PAF_UID}:${PAF_GID} ${INTERNAL_WORKSPACE_DIR}
USER ${PAF_USERNAME}
ENV PIP_CACHE_DIR=${INTERNAL_WORKSPACE_DIR}/pip-cache
ADD build/docker/agent-ros2/setup-scripts ${INTERNAL_WORKSPACE_DIR}/setup-scripts/

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/.env.bash

############################
# Install carla ros bridge #
############################
# The ros bridge is installed into a separate workspace CARLA_ROS_WS that always has to be sourced BEFORE PAF_ROS_WS

ARG LEADERBOARD_GIT_BRANCH=leaderboard-2.1

USER ${PAF_USERNAME}

ENV CARLA_ROS_WS=${INTERNAL_WORKSPACE_DIR}/carla_ros2_ws
RUN mkdir -p ${CARLA_ROS_WS}/src
WORKDIR ${CARLA_ROS_WS}
RUN git clone --recurse-submodules -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/carla-simulator/ros-bridge ./src/ros-bridge
# Fix compilation dependency: https://github.com/carla-simulator/ros-bridge/issues/737
RUN sed -i '/include_directories(include ${PCL_INCLUDE_DIRS})/a include_directories(${tf2_eigen_INCLUDE_DIRS})' ./src/ros-bridge/pcl_recorder/CMakeLists.txt
# In jazzy, some deprecated header files got removed. This includes tf2_eigen.h (tf2_eigen.hpp should be used instead)
# https://docs.ros.org/en/jazzy/Releases/Jazzy-Jalisco-Complete-Changelog.html#tf2-eigen
# Restore the tf2-eigen.h: https://github.com/ros2/geometry2/pull/645
USER root
RUN wget -O /opt/ros/${ROS_DISTRO}/include/tf2_eigen/tf2_eigen/tf2_eigen.h https://raw.githubusercontent.com/ros2/geometry2/1621942bc2ad1270ee0bfc1f6b7a44a5849a7b5e/tf2_eigen/include/tf2_eigen/tf2_eigen.h

# Dependencies for the bridge as listed in ros-bridge/install_dependencies.sh and ros-bridge/CMakelists.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-ackermann-msgs \
  ros-${ROS_DISTRO}-derived-object-msgs \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-vision-opencv \
  ros-${ROS_DISTRO}-rqt-image-view \
  ros-${ROS_DISTRO}-rqt-gui-py \
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-tf2-eigen \
  ros-${ROS_DISTRO}-tf2-geometry-msgs

USER ${PAF_USERNAME}
# Other dependencies via rosdep
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/.env.bash && \
  ${INTERNAL_WORKSPACE_DIR}/setup-scripts/rosdep_install.sh ./src \
  "

RUN bash -c "source ${INTERNAL_WORKSPACE_DIR}/.env.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
  echo source "${CARLA_ROS_WS}/install/local_setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/.env.bash

##########################################
# Finish building the internal workspace #
##########################################

USER ${PAF_USERNAME}

WORKDIR ${INTERNAL_WORKSPACE_DIR}

# TODO: create PAF_ROS_WS

ADD build/docker/agent-ros2/user-scripts ${INTERNAL_WORKSPACE_DIR}/user-scripts/

RUN echo source ${INTERNAL_WORKSPACE_DIR}/.env.bash >> ~/.bashrc

USER root
ADD build/docker/agent-ros2/entrypoint_ros2.sh /entrypoint.sh
USER ${PAF_USERNAME}
ENTRYPOINT [ "/entrypoint.sh" ]

###########################################
##### The agent-dev image starts here #####
###########################################
FROM agent AS agent-dev

# Dev tools
USER root

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  neovim htop nano iputils-ping \
  man-db manpages-posix manpages-dev manpages-posix-dev

USER ${PAF_USERNAME}
