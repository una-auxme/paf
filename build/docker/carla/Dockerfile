ARG BASE_FLAVOUR=cuda
ARG BASE_IMAGE=ubuntu:24.04

FROM ${BASE_IMAGE} AS carla-base-rocm

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
# FROM nvcr.io/nvidia/cuda:12.6.3-runtime-ubuntu24.04 AS carla-base-cuda
# The cuda runtime image breaks vulkan support
FROM ${BASE_IMAGE} AS carla-base-cuda
# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y libvulkan1 vulkan-tools libnvidia-gl-570

############################################
##### The main agent image starts here #####
############################################
FROM carla-base-${BASE_FLAVOUR} AS carla

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  xz-utils

ENV CARLA_ROOT=/opt/carla
RUN --mount=type=bind,target=/opt/carla.tar.xz,source=build/docker/carla/CARLA_Leaderboard_2.0.tar.xz \
  mkdir -p ${CARLA_ROOT} && \
  tar --no-same-owner -xf /opt/carla.tar.xz -C ${CARLA_ROOT} --strip-components=1
WORKDIR ${CARLA_ROOT}
RUN chmod +x CarlaUE4.sh && chmod +x ./CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt install -y \
  xdg-user-dirs xdg-utils

FROM ${BASE_IMAGE} as carla-api

COPY --from=carla /opt/carla/PythonAPI /opt/carla/PythonAPI

FROM carla-api as carla-ros-bridge

#####################################
# Install python build dependencies #
#####################################

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  build-essential libssl-dev zlib1g-dev libbz2-dev \
  libreadline-dev libsqlite3-dev wget curl llvm \
  libncurses5-dev libncursesw5-dev xz-utils tk-dev \
  libffi-dev liblzma-dev python3-openssl git

###########################
# Create internal workdir #
###########################

ENV INTERNAL_WORKSPACE_DIR=/internal_workspace
RUN mkdir -p ${INTERNAL_WORKSPACE_DIR}
ENV PIP_CACHE_DIR=${INTERNAL_WORKSPACE_DIR}/pip-cache
ADD build/docker/agent/setup-scripts ${INTERNAL_WORKSPACE_DIR}/setup-scripts/

##################################
# Compile python 3.8 from source #
##################################

WORKDIR ${INTERNAL_WORKSPACE_DIR}
RUN mkdir python38 && \
  wget https://www.python.org/ftp/python/3.8.20/Python-3.8.20.tgz -O python38.tgz && \
  tar -xf python38.tgz -C python38 --strip-components=1 && \
  rm python38.tgz

WORKDIR ${INTERNAL_WORKSPACE_DIR}/python38
RUN ./configure --enable-optimizations --prefix=/usr && \
  make -j$(nproc) && \
  make altinstall
ENV PYTHON_VERSION=3.8
# ENV LEADERBOARD_PYTHON=${INTERNAL_WORKSPACE_DIR}/python38/python

#################################################
# Install carla leaderboard and scenario runner #
#################################################
ARG LEADERBOARD_GIT_BRANCH=leaderboard-2.1

# Scenario runner
ENV SCENARIO_RUNNER_ROOT=/opt/scenario_runner
RUN git clone -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/carla-simulator/scenario_runner.git $SCENARIO_RUNNER_ROOT

# Carla leaderboard
ENV LEADERBOARD_ROOT=/opt/leaderboard
RUN git clone -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/una-auxme/leaderboard $LEADERBOARD_ROOT

ENV INTERNAL_WORKSPACE_DIR=/internal_workspace
RUN mkdir -p ${INTERNAL_WORKSPACE_DIR}
ENV PIP_CACHE_DIR=${INTERNAL_WORKSPACE_DIR}/pip-cache
ADD build/docker/agent/setup-scripts ${INTERNAL_WORKSPACE_DIR}/setup-scripts/
WORKDIR ${INTERNAL_WORKSPACE_DIR}/setup-scripts
ENV LEADERBOARD_VENV=${INTERNAL_WORKSPACE_DIR}/leaderboard_venv
RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  python3.8 -m venv "${LEADERBOARD_VENV}" && \
  echo source "${LEADERBOARD_VENV}/bin/activate" >> ~/.bashrc

RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  bash install_requirements.sh ${LEADERBOARD_VENV} \
  ${LEADERBOARD_ROOT}/requirements.txt ${SCENARIO_RUNNER_ROOT}/requirements.txt

# Setup carla + leaderboard python dependencies
ENV CARLA_ROOT=/opt/carla
COPY --from=carla-api /opt/carla/PythonAPI /opt/carla/PythonAPI
RUN echo "${CARLA_ROOT}/PythonAPI/carla" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
  echo "${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
  echo "${SCENARIO_RUNNER_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth && \
  echo "${LEADERBOARD_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth

# "Fix" libtiff.so.5 missing for the carla python API
RUN wget http://security.ubuntu.com/ubuntu/pool/main/t/tiff/libtiff5_4.3.0-6ubuntu0.10_amd64.deb -O libtiff5.deb && \
  apt-get install -y ./libtiff5.deb && \
  rm libtiff5.deb

##################################
# Install ROS build dependencies #
##################################
# Based on https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get -y install locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS repository
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update &&  apt-get install -y software-properties-common && \
  add-apt-repository universe && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update &&  apt-get install -y \
  ros-dev-tools

###########################
# Build ROS 2 from source #
###########################

ENV ROS_ROOT=${INTERNAL_WORKSPACE_DIR}/ros_${ROS_DISTRO}
RUN mkdir -p ${ROS_ROOT}/src
WORKDIR ${ROS_ROOT}
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos src

# Always use bash because we need to build in the venv with python 3.8
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && \
  bash -c " \
  source ${LEADERBOARD_VENV}/bin/activate && \
  rosdep init && rosdep update && \
  rosdep install --from-paths src --ignore-src -y \
  --skip-keys 'fastcdr rti-connext-dds-6.0.1 urdfdom_headers' \
  "

# Install python dependencies again but this time including the ROS ones
RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  cd ${INTERNAL_WORKSPACE_DIR}/setup-scripts && \
  pip freeze | awk -F "==" '{ print $1 }' > ./requirements_ros_build_freeze.txt && \
  bash install_requirements.sh ${LEADERBOARD_VENV} \
  ${LEADERBOARD_ROOT}/requirements.txt ${SCENARIO_RUNNER_ROOT}/requirements.txt ./requirements_ros_build.txt ./requirements_ros_build_freeze.txt


RUN bash -c " \
  source ${LEADERBOARD_VENV}/bin/activate && \
  colcon build --symlink-install \
  "

