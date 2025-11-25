# Currently supported BASE_FLAVOUR:
# - gpu: base for amd + intel (vulkan and opengl drivers only)
# - cuda: nvidia
ARG BASE_FLAVOUR=gpu
ARG BASE_IMAGE=ubuntu:24.04

FROM ${BASE_IMAGE} AS carla-base-gpu

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y software-properties-common wget curl

# GPU vulkan driver install (amd + intel)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository -y ppa:kisak/turtle && apt-get update && \
  apt-get upgrade -y && apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers

FROM carla-base-gpu AS carla-base-cuda
ARG NVIDIA_DRIVER_VERSION=550
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.17.5/docker-specialized.html#nvidia-require-constraints
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_REQUIRE_DRIVER="driver>=${NVIDIA_DRIVER_VERSION}"

# Cuda toolkit install
# Using the cuda-toolkit instead of the ubuntu-packaged nvidia-cuda-toolkit avoids pulling in the libnvidia-compute-535 driver package.
# (It is a dependency of nvidia-cuda-toolkit, but not of cuda-toolkit)
# The libnvidia-compute-535 driver can conflict with the host's driver
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
  dpkg -i cuda-keyring_1.1-1_all.deb && \
  rm cuda-keyring_1.1-1_all.deb
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y cuda-toolkit libxext6

############################################
##### The main carla image starts here #####
############################################
FROM carla-base-${BASE_FLAVOUR} AS carla

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  xz-utils pigz

ENV CARLA_ROOT=/opt/carla
RUN --mount=type=bind,target=/opt/carla.tar.gz,source=build/docker/carla/carla.tar.gz \
  mkdir -p ${CARLA_ROOT} && \
  tar --use-compress-program=pigz --no-same-owner -xf /opt/carla.tar.gz -C ${CARLA_ROOT}
WORKDIR ${CARLA_ROOT}
RUN --mount=type=bind,target=${CARLA_ROOT}/Import/carla-additional-maps.tar.gz,source=build/docker/carla/carla-additional-maps.tar.gz \
  tar --use-compress-program=pigz --no-same-owner --skip-old-files -xf ./Import/carla-additional-maps.tar.gz -C ${CARLA_ROOT}
# Tar with --keep-newer-files in ImportAssets.sh leads to errors: "Unexpected inconsistency when making directory"
# chmod +x ImportAssets.sh && ./ImportAssets.sh
RUN chmod +x CarlaUE4.sh && chmod +x ./CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  xdg-user-dirs xdg-utils

COPY --chmod=755 build/docker/carla/carla_entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

FROM ${BASE_IMAGE} AS carla-api

COPY --from=carla /opt/carla/PythonAPI /opt/carla/PythonAPI

##################################################
##### The carla-ros-bridge image starts here #####
##################################################
FROM carla-api AS carla-ros-bridge

#####################################
# Install python build dependencies #
#####################################

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  build-essential libssl-dev zlib1g-dev libbz2-dev \
  libreadline-dev libsqlite3-dev wget curl llvm \
  libncurses5-dev libncursesw5-dev xz-utils tk-dev \
  libffi-dev liblzma-dev python3-openssl git\
  python3.12 python3.12-venv

###########################
# Create internal workdir #
###########################

ENV INTERNAL_WORKSPACE_DIR=/internal_workspace
RUN mkdir -p ${INTERNAL_WORKSPACE_DIR}
ENV PIP_CACHE_DIR=${INTERNAL_WORKSPACE_DIR}/pip-cache

##################################
# Compile python 3.8 from source #
##################################

WORKDIR ${INTERNAL_WORKSPACE_DIR}
ENV PYTHON_VERSION=3.12
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
COPY build/docker/carla/setup-scripts ${INTERNAL_WORKSPACE_DIR}/setup-scripts/
WORKDIR ${INTERNAL_WORKSPACE_DIR}/setup-scripts
ENV LEADERBOARD_VENV=${INTERNAL_WORKSPACE_DIR}/leaderboard_venv
RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  python${PYTHON_VERSION} -m venv "${LEADERBOARD_VENV}" && \
  echo source "${LEADERBOARD_VENV}/bin/activate" >> ${INTERNAL_WORKSPACE_DIR}/env.bash

RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  bash install_requirements.sh ${LEADERBOARD_VENV} \
  ./requirements_leaderboard.txt ./requirements_ros_build.txt ./requirements_ros_bridge.txt

# Setup carla + leaderboard python dependencies
ENV CARLA_ROOT=/opt/carla
COPY --from=carla-api /opt/carla/PythonAPI /opt/carla/PythonAPI
RUN bash -c "\
  source "${LEADERBOARD_VENV}/bin/activate" && \
  pip install "${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl" \
  "
RUN \
  echo "${SCENARIO_RUNNER_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth && \
  echo "${LEADERBOARD_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth

# libtiff.so.5 is required for for the carla python API, but it is not shipped anymore in ubuntu 24.04
# -> build from source and install
WORKDIR ${INTERNAL_WORKSPACE_DIR}
RUN mkdir libtiff && \
  wget https://download.osgeo.org/libtiff/tiff-4.3.0.tar.gz -O tiff.tar.gz && \
  tar -xf tiff.tar.gz -C libtiff --strip-components=1 && \
  rm tiff.tar.gz
RUN cd libtiff && ./configure && make && make install

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

###########################
# Build ROS 2 from source #
###########################

ENV ROS_ROOT=${INTERNAL_WORKSPACE_DIR}/ros_${ROS_DISTRO}
RUN mkdir -p ${ROS_ROOT}/src
WORKDIR ${ROS_ROOT}
RUN bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  vcs import --input https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos src \
  "

# Always use bash because we need to build in the venv with python 3.8
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && \
  bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  rosdep init && rosdep update && \
  rosdep install --from-paths src --ignore-src -y \
  --skip-keys 'fastcdr rti-connext-dds-6.0.1 urdfdom_headers' \
  "

# Install python dependencies again but this time including the ROS ones
# The pip freeze... uses the installed system packages as requirement for the venv (without versions)
WORKDIR ${INTERNAL_WORKSPACE_DIR}/setup-scripts
RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  pip freeze | awk -F "==" '{ print $1 }' > ./requirements_ros_build_freeze.txt && \
  bash install_requirements.sh ${LEADERBOARD_VENV} \
  ./requirements_leaderboard.txt ./requirements_ros_build.txt ./requirements_ros_build_freeze.txt
WORKDIR ${ROS_ROOT}

# Remove some conflicting libraries because ros ships its own version
# https://github.com/ros2/orocos_kdl_vendor/issues/29
RUN apt-get remove -y pybind11-dev liborocos-kdl-dev liborocos-kdl1.5 libshiboken2-dev

# # In jazzy, some deprecated header files got removed. This includes tf2_eigen.h (tf2_eigen.hpp should be used instead)
# # https://docs.ros.org/en/jazzy/Releases/Jazzy-Jalisco-Complete-Changelog.html#tf2-eigen
# # Restore the tf2-eigen.h: https://github.com/ros2/geometry2/pull/645
RUN wget -O /${ROS_ROOT}/src/ros2/geometry2/tf2_eigen/include/tf2_eigen/tf2_eigen.h https://raw.githubusercontent.com/ros2/geometry2/1621942bc2ad1270ee0bfc1f6b7a44a5849a7b5e/tf2_eigen/include/tf2_eigen/tf2_eigen.h

RUN bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  colcon build --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-ignore \
  qt_gui_cpp rqt_gui_cpp \
  " && \
  rm -r ${ROS_ROOT}/build ${ROS_ROOT}/log

RUN echo source "${ROS_ROOT}/install/setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/env.bash

############################
# Install carla ros bridge #
############################
# The ros bridge is installed into a separate workspace CARLA_ROS_WS

ENV CARLA_ROS_WS=${INTERNAL_WORKSPACE_DIR}/carla_ros2_ws
RUN mkdir -p ${CARLA_ROS_WS}/src
WORKDIR $CARLA_ROS_WS
RUN git clone --recurse-submodules -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/carla-simulator/ros-bridge ./src/ros-bridge
# Fix compilation dependency: https://github.com/carla-simulator/ros-bridge/issues/737
RUN sed -i '/include_directories(include ${PCL_INCLUDE_DIRS})/a include_directories(${tf2_eigen_INCLUDE_DIRS})' ./src/ros-bridge/pcl_recorder/CMakeLists.txt

RUN git clone --recurse-submodules -b 1.0.0 --single-branch https://github.com/ros-perception/pcl_msgs ./src/pcl_msgs
RUN git clone --recurse-submodules -b 2.6.3 --single-branch https://github.com/ros-perception/perception_pcl ./src/perception_pcl
RUN git clone --recurse-submodules -b 4.0.0 --single-branch https://github.com/astuff/astuff_sensor_msgs.git ./src/astuff_sensor_msgs

# Always use bash because we need to build in the venv with python 3.8
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && \
  bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  rosdep update && \
  rosdep install -r --from-paths src --ignore-src -y \
  "

RUN bash -c "\
  source ${LEADERBOARD_VENV}/bin/activate && \
  source ${ROS_ROOT}/install/setup.bash && \
  colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  " && \
  rm -r ${CARLA_ROS_WS}/build ${CARLA_ROS_WS}/log
RUN echo source "${CARLA_ROS_WS}/install/local_setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/env.bash

####################
# Setup ros2 agent #
####################

# Add paf_leaderboard_ros_bridge package because the leaderboards wants to launch something
WORKDIR ${INTERNAL_WORKSPACE_DIR}
RUN mkdir -p ros2_ws/src
COPY build/docker/carla/paf_leaderboard_ros_bridge ros2_ws/src/
RUN bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  cd ros2_ws && \
  colcon build" && \
  echo source "${INTERNAL_WORKSPACE_DIR}/ros2_ws/install/local_setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/env.bash

RUN echo "${CARLA_ROOT}/PythonAPI/carla" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth

# The agent launches the paf_leaderboard_ros_bridge package
COPY build/docker/carla/agent_ros2_bridge.py ./
CMD [ "bash", "-c", "sleep infinity"  ]

RUN echo "source ${INTERNAL_WORKSPACE_DIR}/env.bash" >> ~/.bashrc
COPY build/docker/carla/wait_for_carla.py /
COPY build/docker/carla/bridge_entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
