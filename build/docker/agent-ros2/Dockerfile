ARG BASE_FLAVOUR=cuda

FROM rocm/dev-ubuntu-24.04:6.4 AS agent-base-rocm

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y software-properties-common

# GPU vulkan driver update
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository -y ppa:kisak/turtle && apt-get update && apt-get upgrade -y

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda
# FROM nvcr.io/nvidia/cuda:12.8.1-cudnn-runtime-ubuntu20.04 AS agent-base-cuda
FROM ubuntu:24.04 AS agent-base-cuda

############################################
##### The main agent image starts here #####
############################################
FROM agent-base-${BASE_FLAVOUR} AS agent

ENV DEBIAN_FRONTEND=noninteractive

#######################
# Install ROS 2 Jazzy https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
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

# Install paf project dependencies
# RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
#   --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   apt-get update && apt-get install -y \
#   ros-${ROS_DISTRO}-ackermann-msgs \
#   ros-${ROS_DISTRO}-derived-object-msgs \
#   ros-${ROS_DISTRO}-pcl-conversions \
#   ros-${ROS_DISTRO}-rviz2 \
#   ros-${ROS_DISTRO}-rqt \
#   ros-${ROS_DISTRO}-pcl-ros \
#   ros-${ROS_DISTRO}-rosbridge-suite \
#   ros-${ROS_DISTRO}-rosbridge-server \
#   ros-${ROS_DISTRO}-py-trees-ros \
#   ros-${ROS_DISTRO}-rqt-reconfigure \
#   ros-${ROS_DISTRO}-ament-cmake-pytest \
#   ros-${ROS_DISTRO}-robot-localization \
#   ros-${ROS_DISTRO}-tf2-ros \
#   ros-${ROS_DISTRO}-tf2-eigen \
#   ros-${ROS_DISTRO}-tf2-geometry-msgs


# Not available in jazzy:
#  ros-jazzy-carla-msgs \
#  ros-jazzy-ros-numpy \
#  ros-jazzy-robot-pose-ekf \
#  ros-jazzy-rqt-py-trees \
#  ros-jazzy-ros-pytest \, replaced with ros-jazzy-ament-cmake-pytest

#################################
# Create the internal workspace #
#################################
# This workspace at INTERNAL_WORKSPACE_DIR contains
# - INTERNAL_ROS_WS: a ROS2 workspace for the PAF project packages (created later)
# - INTERNAL_VENV: a venv for the python dependencies
# INTERNAL_WORKSPACE_DIR is editable by non-root users (chmod at the end of this image)

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
ADD build/docker/agent/setup-scripts ${INTERNAL_WORKSPACE_DIR}/setup-scripts/

RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/.env.bash

#################################################
# Install carla leaderboard and scenario runner #
#################################################
ARG LEADERBOARD_GIT_BRANCH=leaderboard-2.1

# # Scenario runner
# ENV SCENARIO_RUNNER_ROOT=/opt/scenario_runner
# RUN git clone -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/carla-simulator/scenario_runner.git $SCENARIO_RUNNER_ROOT

# # Carla leaderboard
# ENV LEADERBOARD_ROOT=/opt/leaderboard
# RUN git clone -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/una-auxme/leaderboard $LEADERBOARD_ROOT

# WORKDIR ${INTERNAL_WORKSPACE_DIR}/setup-scripts
# ENV LEADERBOARD_VENV=${INTERNAL_WORKSPACE_DIR}/leaderboard_venv
# RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
#   bash ./create_venv.sh ${LEADERBOARD_VENV}

# RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
#   bash install_requirements.sh ${LEADERBOARD_VENV} \
#   $LEADERBOARD_ROOT/requirements.txt ${SCENARIO_RUNNER_ROOT}/requirements.txt

# # Setup carla + leaderboard python dependencies
# ENV CARLA_ROOT=/opt/carla
# COPY --from=carla-api /opt/carla/PythonAPI /opt/carla/PythonAPI
# RUN echo "${CARLA_ROOT}/PythonAPI/carla" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
#   echo "${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
#   echo "${SCENARIO_RUNNER_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth && \
#   echo "${LEADERBOARD_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth

############################
# Install carla ros bridge #
############################
# The ros bridge is installed into a separate workspace ROOT_ROS_WS that always has to be sourced BEFORE INTERNAL_ROS_WS

USER ${PAF_USERNAME}

ENV ROOT_ROS_WS=${INTERNAL_WORKSPACE_DIR}/root_ros2_ws
RUN mkdir -p ${ROOT_ROS_WS}/src
WORKDIR $ROOT_ROS_WS
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

RUN bash -c "source ${INTERNAL_WORKSPACE_DIR}/.env.bash && colcon build" && \
  echo source "${ROOT_ROS_WS}/install/local_setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/.env.bash

# WIP: Install carla API
# install dependencies for libgit2 and Carla PythonAPI
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#   --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   apt-get update && apt-get install -y wget unzip

# ENV CARLA_ROOT=/opt/carla
# # Download Carla PythonAPI (alternative to getting it from the Carla-Image, which is commented out above)
# # If the PythonAPI/Carla version changes, either update the link, or refer to the comment at the top of this file.
# # RUN wget https://github.com/una-auxme/paf/releases/download/v0.0.1/PythonAPI_Leaderboard-2.0.zip --quiet -O PythonAPI.zip \
# #   && unzip PythonAPI.zip \
# #   && rm PythonAPI.zip \
# #   && mkdir -p ${CARLA_ROOT} \
# #   && mv PythonAPI ${CARLA_ROOT}/PythonAPI

# COPY --from=carla-api /opt/carla/PythonAPI /opt/carla/PythonAPI


##########################################
# Finish building the internal workspace #
##########################################

# WORKDIR ${INTERNAL_WORKSPACE_DIR}/setup-scripts
# RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
#   bash install_requirements.sh ${INTERNAL_VENV} \
#   requirements_carla.txt

# # Setup carla + leaderboard python dependencies
# RUN echo "${CARLA_ROOT}/PythonAPI/carla" >>"${INTERNAL_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
#   echo "${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg" >>"${INTERNAL_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
#   echo "${SCENARIO_RUNNER_ROOT}" >>"${INTERNAL_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth && \
#   echo "${LEADERBOARD_ROOT}" >>"${INTERNAL_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth
# "Fix" the carla python API
# RUN wget http://security.ubuntu.com/ubuntu/pool/main/t/tiff/libtiff5_4.3.0-6ubuntu0.10_amd64.deb -O libtiff5.deb && \
#   apt-get install -y ./libtiff5.deb && \
#   rm libtiff5.deb
USER ${PAF_USERNAME}

WORKDIR ${INTERNAL_WORKSPACE_DIR}
ADD build/docker/agent/user-scripts ${INTERNAL_WORKSPACE_DIR}/user-scripts/

RUN echo source ${INTERNAL_WORKSPACE_DIR}/.env.bash >> ~/.bashrc

USER root
ADD build/docker/agent/entrypoint_ros2.sh /entrypoint.sh
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
