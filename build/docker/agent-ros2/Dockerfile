# Currently supported BASE_FLAVOUR:
# - generic: no gpu support
# - gpu: base for amd + intel (vulkan and opengl drivers only)
# - rocm: amd with compute
# - cuda: nvidia with compute
ARG BASE_FLAVOUR=generic
ARG BASE_IMAGE=ubuntu:24.04

FROM ${BASE_IMAGE} AS agent-base-generic

FROM agent-base-generic AS agent-base-gpu

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y software-properties-common

# GPU vulkan driver install (amd + intel)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository -y ppa:kisak/turtle && apt-get update && \
  apt-get upgrade -y && apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers

FROM agent-base-gpu AS agent-base-rocm
# No changes needed here, only pytorch installation requires changes

FROM agent-base-gpu AS agent-base-cuda
ARG NVIDIA_DRIVER_VERSION=550
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.17.5/docker-specialized.html#nvidia-require-constraints
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_REQUIRE_DRIVER="cuda>=12.0 driver>=${NVIDIA_DRIVER_VERSION}"

# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y libnvidia-gl-${NVIDIA_DRIVER_VERSION} \
  nvidia-cuda-toolkit

############################################
##### The main agent image starts here #####
############################################
FROM agent-base-${BASE_FLAVOUR} AS agent
# Redefine these args, because they are not inherited
ARG BASE_FLAVOUR
ENV BASE_FLAVOUR=${BASE_FLAVOUR}

ENV DOCKER_RESOURCE_BASE=build/docker/agent-ros2

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
  apt-get update && apt-get install curl -y && \
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
  dpkg -i /tmp/ros2-apt-source.deb

RUN  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && apt-get install -y \
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
  python${PYTHON_VERSION} python3-pip python-is-python3 \
  sudo

# Create the paf user
ARG PAF_USERNAME=paf
ARG PAF_UID=1000
ARG PAF_GID=1000

# Delete exiting user or group with paf ids if they exist
RUN if getent passwd "${PAF_UID}"; then userdel -r $(id -u -n "${PAF_UID}"); fi && \
  if getent group "${PAF_GID}"; then groupdel $(id -g -n "${PAF_GID}"); fi
RUN groupadd --gid "${PAF_GID}" "${PAF_USERNAME}" \
  && useradd --uid "${PAF_UID}" --gid "$PAF_GID" -m -s /bin/bash "${PAF_USERNAME}" \
  && passwd -d "${PAF_USERNAME}" \
  && echo "${PAF_USERNAME}" ALL=\(root\) NOPASSWD:ALL > "/etc/sudoers.d/${PAF_USERNAME}" \
  && chmod 0440 "/etc/sudoers.d/${PAF_USERNAME}"

# Build internal workspace
ENV INTERNAL_WORKSPACE_DIR=/internal_workspace
RUN mkdir -p ${INTERNAL_WORKSPACE_DIR} && \
  chown ${PAF_UID}:${PAF_GID} ${INTERNAL_WORKSPACE_DIR}
USER ${PAF_USERNAME}
RUN echo source "/opt/ros/${ROS_DISTRO}/setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/env.bash

############################
# Install carla ros bridge #
############################
# The ros bridge is installed into a separate workspace CARLA_ROS_WS that always has to be sourced BEFORE PAF_ROS_WS

ARG LEADERBOARD_GIT_BRANCH=leaderboard-2.1

USER ${PAF_USERNAME}

ENV CARLA_ROS_WS=${INTERNAL_WORKSPACE_DIR}/carla_ros2_ws
RUN mkdir -p ${CARLA_ROS_WS}/src
WORKDIR ${CARLA_ROS_WS}
# ros2_numpy is not a dependency of the ros-bridge but currently does not have a rosdep key -> Also installed in the CARLA_ROS_WS
RUN git clone --recurse-submodules -b ${ROS_DISTRO} --single-branch https://github.com/Box-Robotics/ros2_numpy.git ./src/ros2_numpy
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
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  sudo apt-get update && \
  rosdep update --rosdistro ${ROS_DISTRO} && \
  rosdep install -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} -y \
  "

RUN bash -c "source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
  echo source "${CARLA_ROS_WS}/install/local_setup.bash" >> ${INTERNAL_WORKSPACE_DIR}/env.bash

##########################################
# Internal workspace: basic python setup #
##########################################

USER root
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y python${PYTHON_VERSION}-venv

USER ${PAF_USERNAME}
WORKDIR ${INTERNAL_WORKSPACE_DIR}
ENV PIP_CACHE_DIR=${INTERNAL_WORKSPACE_DIR}/pip-cache
ENV PYTHONUSERBASE=${INTERNAL_WORKSPACE_DIR}/python-user-base
RUN mkdir -p ./scripts/
COPY --chmod=755 ${DOCKER_RESOURCE_BASE}/scripts/install-python-requirements.sh ./scripts/

# Enable the python fault handler for printing stack traces on segfaults/c-assertions
ENV PYTHONFAULTHANDLER=1

#################################################
# Install carla leaderboard and scenario runner #
#################################################

USER ${PAF_USERNAME}
# Scenario runner
ENV SCENARIO_RUNNER_ROOT=${INTERNAL_WORKSPACE_DIR}/scenario_runner
RUN git clone -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/carla-simulator/scenario_runner.git $SCENARIO_RUNNER_ROOT

# Carla leaderboard
ENV LEADERBOARD_ROOT=${INTERNAL_WORKSPACE_DIR}/leaderboard
RUN git clone -b ${LEADERBOARD_GIT_BRANCH} --single-branch https://github.com/Zelberor/leaderboard.git $LEADERBOARD_ROOT

WORKDIR ${INTERNAL_WORKSPACE_DIR}/scripts
COPY --chmod=755 ${DOCKER_RESOURCE_BASE}/scripts/venv_install_requirements.sh \
  ${DOCKER_RESOURCE_BASE}/scripts/requirements_leaderboard.txt \
  ./
ENV LEADERBOARD_VENV=${INTERNAL_WORKSPACE_DIR}/leaderboard_venv
RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  python${PYTHON_VERSION} -m venv "${LEADERBOARD_VENV}" --system-site-packages

RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked \
  bash venv_install_requirements.sh ${LEADERBOARD_VENV} ./requirements_leaderboard.txt

# Add carla + leaderboard libraries into leaderboard_venv
ENV CARLA_ROOT=/opt/carla
COPY --from=carla-leaderboard-api:2.1 /opt/carla/PythonAPI ${CARLA_ROOT}/PythonAPI
RUN bash -c "\
  source "${LEADERBOARD_VENV}/bin/activate" && \
  pip install "${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl" \
  "
RUN echo "${CARLA_ROOT}/PythonAPI/carla" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth && \
  echo "${SCENARIO_RUNNER_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth && \
  echo "${LEADERBOARD_ROOT}" >>"${LEADERBOARD_VENV}"/lib/python${PYTHON_VERSION}/site-packages/leaderboard.pth
# Add carla + leaderboard libraries into PYTHONUSERBASE
RUN pip install "${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.16-cp312-cp312-manylinux_2_31_x86_64.whl"
RUN echo "${CARLA_ROOT}/PythonAPI/carla" >>"${PYTHONUSERBASE}"/lib/python${PYTHON_VERSION}/site-packages/carla.pth

##########################################
# Finish building the internal workspace #
##########################################

# Create the main agent workspace
ENV PAF_ROS_WS=${INTERNAL_WORKSPACE_DIR}/paf_ros2_ws
RUN mkdir ${PAF_ROS_WS}

##############################################
##### The agent-deploy image starts here #####
##############################################
# The deploy image is just an experiment for now / reference for discussion
FROM agent AS agent-deploy
# TODO
RUN echo source ${INTERNAL_WORKSPACE_DIR}/env.bash >> ~/.bashrc

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

# Vs Code. This makes it possible to start VS Code fully inside the container.
# Not required to only attach the host's VS Code to the container.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
  install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
  echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null && \
  rm -f packages.microsoft.gpg && \
  apt-get update && apt-get install -y code
# Use different data dir to avoid dbus conflicts with the host installation
ENV VSCODE_APPDATA=/home/${PAF_USERNAME}/.config/code_docker

USER ${PAF_USERNAME}
WORKDIR ${INTERNAL_WORKSPACE_DIR}
COPY --chown=${PAF_UID}:${PAF_GID} ${DOCKER_RESOURCE_BASE}/scripts/dev.bashrc ./
COPY --chmod=755 ${DOCKER_RESOURCE_BASE}/scripts/entrypoint-dev.sh \
  ${DOCKER_RESOURCE_BASE}/scripts/devfunctions.bash \
  ${DOCKER_RESOURCE_BASE}/scripts/reset_env.bash \
  ${DOCKER_RESOURCE_BASE}/scripts/leaderboard.sh \
  ./scripts/

ENTRYPOINT [ "/internal_workspace/scripts/entrypoint-dev.sh" ]

WORKDIR ${PAF_ROS_WS}
# Install package.xml dependencies (but allow it to fail, because this is a dev container)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  --mount=type=bind,target=${PAF_ROS_WS}/src,source=code \
  bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/env.bash && \
  sudo apt-get update && \
  rosdep update --rosdistro ${ROS_DISTRO} && \
  ((rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y |& tee ${INTERNAL_WORKSPACE_DIR}/rosdep_install.log; exit \${PIPESTATUS[0]}) \
  || echo \"echo -e \\\"\\\\e[31mERROR: rosdep install failed, check ${INTERNAL_WORKSPACE_DIR}/rosdep_install.log in the container\\\\e[0m\\\"\" >> ${INTERNAL_WORKSPACE_DIR}/dev.bashrc) \
  "

# Install python requirements.txt dependencies (but allow it to fail, because this is a dev container)
RUN --mount=type=cache,target=${PIP_CACHE_DIR},sharing=locked,uid="${PAF_UID}",gid="${PAF_GID}" \
  --mount=type=bind,target=${PAF_ROS_WS}/src,source=code \
  bash -c " \
  source ${INTERNAL_WORKSPACE_DIR}/dev.bashrc && \
  ((${INTERNAL_WORKSPACE_DIR}/scripts/install-python-requirements.sh |& tee ${INTERNAL_WORKSPACE_DIR}/pip_install.log; exit \${PIPESTATUS[0]}) \
  || echo \"echo -e \\\"\\\\e[31mERROR: pip install failed, check ${INTERNAL_WORKSPACE_DIR}/pip_install.log in the container\\\\e[0m\\\"\" >> ${INTERNAL_WORKSPACE_DIR}/dev.bashrc) \
  "

RUN echo source ${INTERNAL_WORKSPACE_DIR}/dev.bashrc >> ~/.bashrc

VOLUME [ "/workspace" ]
# Link the workspace into the PAF_ROS_WS
RUN ln -s -T /workspace/code ${PAF_ROS_WS}/src

USER root
# Make a backup of the home directories, because they get mounted as a volume
RUN mkdir -p /opt/home-bk && cp -ar /home/. -t /opt/home-bk/
USER ${PAF_USERNAME}

# Just keep this container open because it is used as a dev environment
CMD [ "/bin/sleep", "infinity" ]
