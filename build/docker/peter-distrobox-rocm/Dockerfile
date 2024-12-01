FROM rocm/dev-ubuntu-20.04:6.1-complete

ENV DEBIAN_FRONTEND=noninteractive

RUN yes | unminimize
RUN apt-get update && apt-get install -y build-essential lsb-release software-properties-common \
  python3.8 python3.8-venv python-is-python3 \
  curl wget git \
  man-db manpages-posix manpages-dev manpages-posix-dev

# GPU vulkan driver update
RUN add-apt-repository -y ppa:kisak/turtle && apt-get update && apt-get upgrade -y

# ROS noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# Carla
RUN apt-get install -y libomp-dev
ENV CARLA_ROOT=/opt/carla
# Do not use https://tiny.carla.org/carla-0-9-14-linux
# Use leaderboard release instead: https://leaderboard.carla.org/get_started/
RUN wget --no-verbose -O /opt/carla.tar.xz https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz && \
  mkdir -p ${CARLA_ROOT} && \
  tar -xf /opt/carla.tar.xz -C ${CARLA_ROOT} --strip-components=1 && \
  rm /opt/carla.tar.xz
# Alternate: 0.9.15
# RUN wget --no-verbose -O /opt/carla.tar.gz https://tiny.carla.org/carla-0-9-15-linux && \
#  mkdir -p ${CARLA_ROOT} && \
#  tar -xf /opt/carla.tar.gz -C ${CARLA_ROOT} && \
#  rm /opt/carla.tar.gz
# Alternate: Add by file
# ADD CARLA_0.9.15.tar.gz ${CARLA_ROOT}/
# # Carla additional maps
# COPY AdditionalMaps_0.9.15.tar.gz ${CARLA_ROOT}/Import/
# RUN cd ${CARLA_ROOT} && chmod +x ./ImportAssets.sh && ./ImportAssets.sh
ENV CARLA_PORT=2000

# ROS bridge
# RUN apt-get install -y ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs \
#   ros-noetic-carla-msgs ros-noetic-pcl-conversions \
#   ros-noetic-rviz ros-noetic-rqt ros-noetic-pcl-ros ros-noetic-rosbridge-suite ros-noetic-rosbridge-server \
#   ros-noetic-robot-pose-ekf ros-noetic-ros-numpy \
#   ros-noetic-py-trees-ros ros-noetic-rqt-py-trees ros-noetic-rqt-reconfigure \
#   python3-rosdep ros-noetic-ros-pytest
RUN apt-get install -y python3-rosdep
ENV ROOT_CATKIN_WS=/catkin
WORKDIR $ROOT_CATKIN_WS
RUN git clone --recurse-submodules -b leaderboard-2.0 --recurse-submodules --single-branch https://github.com/carla-simulator/ros-bridge ./src/ros-bridge
RUN cd ./src/ros-bridge/carla_ackermann_control && \
  sed -i '24 i \  <build_depend condition="$ROS_VERSION == 1">carla_msgs</build_depend>' package.xml
RUN rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Scenario runner
ENV SCENARIO_RUNNER_ROOT=/opt/scenario_runner
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git $SCENARIO_RUNNER_ROOT

# Carla leaderboard
ENV LEADERBOARD_ROOT=/opt/leaderboard
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/una-auxme/leaderboard $LEADERBOARD_ROOT

# # Vscodium
# RUN wget -qO - https://gitlab.com/paulcarroty/vscodium-deb-rpm-repo/raw/master/pub.gpg \
#   | gpg --dearmor \
#   | dd of=/usr/share/keyrings/vscodium-archive-keyring.gpg
# RUN echo 'deb [ signed-by=/usr/share/keyrings/vscodium-archive-keyring.gpg ] https://download.vscodium.com/debs vscodium main' \
#   | tee /etc/apt/sources.list.d/vscodium.list
# RUN apt-get update && apt-get install -y codium

# Vs Code
RUN wget --no-verbose -O /opt/vscode.deb https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64 && \
  apt-get install -y /opt/vscode.deb && \
  rm /opt/vscode.deb

# Dev tools
RUN apt-get install -y neovim htop

# Build python cache (Creates a dummy venv to force building the cache)
ENV PIP_CACHE_DIR=/pip-cache
ADD ./user-package/create_venv.sh ./user-package/requirements*.txt /root/build/
RUN bash /root/build/create_venv.sh /root/venv && rm -rf /root/venv

# Workdir for volume mount
ENV WORKSPACE_DIR=/workspace
RUN mkdir ${WORKSPACE_DIR}

# Setup init scripts
ENV INTERNAL_WORKSPACE_DIR=/internal_workspace
ADD ./init_hook.sh /
RUN chmod +x /init_hook.sh
RUN chmod +x ${CARLA_ROOT}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping



