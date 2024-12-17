# Based on https://github.com/ll7/paf21-1/blob/master/components/carla_ros_bridge/Dockerfile

# This is commented out because of the large image download necessary to build this image otherwise.
# If the PythonAPI/carla version changes, one can comment out the Download Carla PythonAPI line below.
# Then, uncomment this line and the COPY --from=carla line to build the image.
# FROM ghcr.io/nylser/carla:leaderboard as carla
# Use this image to enable instance segmentation cameras
# FROM carlasim/carla:0.9.14 as carla

# supply the base image with an environment supporting ROS UI via x11
FROM osrf/ros:noetic-desktop-full-focal
# COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG DEBIAN_FRONTEND=noninteractive

# Install all dependencies, including wget and unzip, in one step
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    fprintd libfprint-2-2 libpam-fprintd \
    build-essential libssl-dev libbz2-dev libreadline-dev


# CUDA installieren
RUN apt-get update && apt-get install -y --no-install-recommends wget && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin --quiet && \
    mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb --quiet && \
    dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb && \
    cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y cuda && \
    rm -rf /var/lib/apt/lists/* && \
    rm cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb


# Umgebungsvariablen setzen
ENV PATH=/usr/local/cuda-11.8/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
ENV CUDA_HOME=/usr/local/cuda-11.8

# Download Carla PythonAPI (alternative to getting it from the Carla-Image, which is commented out above)
# If the PythonAPI/Carla version changes, either update the link, or refer to the comment at the top of this file.
RUN wget https://github.com/una-auxme/paf/releases/download/v0.0.1/PythonAPI_Leaderboard-2.0.zip --quiet -O PythonAPI.zip \
    && unzip PythonAPI.zip \
    && rm PythonAPI.zip \
    && mkdir -p /opt/carla \
    && mv PythonAPI /opt/carla/PythonAPI

# Workaround/fix for using dpkg for cuda installation
# Only required for the lab PCs
RUN rm -f /var/lib/dpkg/info/fprintd.postinst \
    && rm -f /var/lib/dpkg/info/libfprint*.postinst \
    && rm -f /var/lib/dpkg/info/libpam-fprintd*.postinst \
    && dpkg --configure -a


# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:/opt/carla/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla
# :/usr/lib/python3.8/site-packages



# install mlocate, pip, wget, git and some ROS dependencies for building the CARLA ROS bridge
RUN apt-get update && apt-get install -y \
    mlocate python3-pip wget git python-is-python3 \
    ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs \
    ros-noetic-carla-msgs ros-noetic-pcl-conversions \
    ros-noetic-rviz ros-noetic-rqt ros-noetic-pcl-ros ros-noetic-rosbridge-suite ros-noetic-rosbridge-server \
    ros-noetic-robot-pose-ekf ros-noetic-ros-numpy \
    ros-noetic-py-trees-ros ros-noetic-rqt-py-trees ros-noetic-rqt-reconfigure

SHELL ["/bin/bash", "-c"]

# ------------------------------------------------------------------ 
# ------------------------ CLRerNET INSTALL ------------------------

# Installiere PyTorch und MMCV
ENV TORCH_CUDA_ARCH_LIST="7.5;8.0;8.6"
RUN pip install torch==1.12.1+cu116 -f https://download.pytorch.org/whl/torch_stable.html 
RUN pip install torchvision==0.13.1
RUN pip install -U openmim==0.3.3
RUN mim install mmcv-full==1.7.0

# Restliche Abhängigkeiten für ROS und Carla
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libxext6 libx11-6 libglvnd0 libgl1 libglx0 libegl1 freeglut3-dev apt-utils \
    sudo git zip curl libopencv-dev libffi-dev liblapack-dev libsqlite3-dev \
    build-essential libssl-dev libbz2-dev libreadline-dev ssh-client wget vim python3-opencv unzip && \
    rm -rf /var/lib/apt/lists/*

# mmdet
ARG MMDET_VERSION
WORKDIR /home/docker/
RUN pip install Cython
RUN git clone https://github.com/open-mmlab/mmdetection.git mmdetection -b v2.28.0 --depth 1
WORKDIR /home/docker/mmdetection
ENV FORCE_CUDA="1"
RUN pip install --no-cache-dir -e .    
# install ninja for nms build
RUN apt-get update && apt-get install -y ninja-build


# CLRerNet Repository einfügen
ARG CLRERNET_REPO_URL=https://github.com/hirotomusiker/CLRerNet.git
ARG CLRERNET_BRANCH=main 

# Klone das CLRerNet-Repository und wechsle zum entsprechenden Verzeichnis
RUN git clone --branch main https://github.com/hirotomusiker/CLRerNet.git /opt/CLRerNet_model && \
    echo "Clone erfolgreich!" && \
    ls -l /opt/CLRerNet_model

# Lade die Modellgewichte
WORKDIR /opt/CLRerNet_model
RUN wget https://github.com/hirotomusiker/CLRerNet/releases/download/v0.1.0/clrernet_culane_dla34_ema.pth -O clrernet_culane_dla34_ema.pth

# Installiere die Abhängigkeiten von CLRerNet
RUN pip install -r requirements.txt || echo "Keine requirements.txt gefunden, überspringe Abhängigkeiten"

# Modell zu PYTHONPATH hinzufügem
ENV PYTHONPATH=/opt/CLRerNet_model:$PYTHONPATH

# Debugging: Überprüfen, ob das Modell erfolgreich importiert werden kann
#RUN python -c "from libs.models.dense_heads.seg_decoder import SegDecoder" || echo "Fehler beim Importieren von SegDecoder"
#RUN python -c "from libs.utils.lane_utils import Lane" || echo "Fehler beim Importieren von Lane"

# Setuptools wird für die Installation von NMS benötigt
WORKDIR /opt/CLRerNet_model/libs/models/layers/nms
RUN pip install -U pip setuptools

# Installiere das nms-Modul
ENV LD_LIBRARY_PATH=/usr/local/lib/python3.8/dist-packages/torch/lib:$LD_LIBRARY_PATH
RUN python setup.py install 

RUN pip install .
RUN ls /usr/lib/python3.8/site-packages | grep nms

ENV PYTHONPATH=/usr/lib/python3.8/site-packages:$PYTHONPATH
RUN find /usr/lib/python3.8/site-packages -name "*nms*"
#RUN ldconfig && python -c "import nms"
RUN python -c "import nms"


# Deinstalliere Setuptools wieder
RUN pip uninstall -y pip setuptools


# ------------------------ END CLRerNET INSTALL ------------------------
# ---------------------------------------------------------------------- 

# Create and apply the non-root user
# -f parameter to work in macos (https://unix.stackexchange.com/a/465014), since gid is already in use there
RUN groupadd --gid $USER_GID $USERNAME -f \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# create a catkin workspace
WORKDIR /catkin_ws

# make sure the catkin wo/code
# install the CARLA ROS bridge
ARG ROS_BRIDGE_GITHUB_URL=https://github.com/carla-simulator/ros-bridge
RUN git clone -b leaderboard-2.0 --recurse-submodules --single-branch $ROS_BRIDGE_GITHUB_URL src/ros-bridge
# apply patch to fix the build of carla_ros_bridge
RUN cd src/ros-bridge/carla_ackermann_control && \
    sed -i '24 i \  <build_depend condition="$ROS_VERSION == 1">carla_msgs</build_depend>' package.xml
RUN source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install --from-paths src --ignore-src -r && \
    catkin_make install

ENV CARLA_ROS_BRIDGE_ROOT=/catkin_ws/src/ros-bridge



# avoid python warnings about missing bin directory in PATH
# (as we're not running as root, pip installs into ~/.local/bin)
ENV PATH=$PATH:/home/$USERNAME/.local/bin

# install pip packages
RUN python -m pip install pip --upgrade \
    && python -m pip install simple_pid pygame transformations roslibpy lxml black

# install the scenario runner from GitHub leaderboard-2.0 branch
ENV CARLA_ROOT=/opt/carla
ENV SCENARIO_RUNNER_ROOT=/opt/scenario_runner
RUN sudo mkdir $SCENARIO_RUNNER_ROOT && sudo chown $USERNAME:$USERNAME $SCENARIO_RUNNER_ROOT
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git $SCENARIO_RUNNER_ROOT
RUN echo 'pexpect' >> $SCENARIO_RUNNER_ROOT/requirements.txt && \
    python -m pip install -r $SCENARIO_RUNNER_ROOT/requirements.txt

# install the leaderboard from GitHub leaderboard-2.0 branch
ENV LEADERBOARD_ROOT=/opt/leaderboard
RUN sudo mkdir $LEADERBOARD_ROOT && sudo chown $USERNAME:$USERNAME $LEADERBOARD_ROOT
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/una-auxme/leaderboard $LEADERBOARD_ROOT
RUN python -m pip install -r $LEADERBOARD_ROOT/requirements.txt
RUN sudo mkdir /opt/leaderboard-py3 && sudo chown $USERNAME:$USERNAME /opt/leaderboard-py3 && \
    ln -s $LEADERBOARD_ROOT/leaderboard /opt/leaderboard-py3/leaderboard && \
    ln -s $SCENARIO_RUNNER_ROOT/srunner /opt/leaderboard-py3/srunner

# environment variables for the leaderboard_evaluator
ENV ROUTES=$LEADERBOARD_ROOT/data/routes_devtest.xml
ENV REPETITIONS=1
ENV DEBUG_CHALLENGE=1
#ENV TEAM_AGENT=$LEADERBOARD_ROOT/leaderboard/autoagents/human_agent.py
ENV TEAM_AGENT=/workspace/code/agent/src/agent/agent.py
ENV CHECKPOINT_ENDPOINT=/workspace/code/simulation_results.json
ENV CHALLENGE_TRACK=MAP

ENV CARLA_SIM_HOST=localhost
ENV CARLA_SIM_WAIT_SECS=15
ENV SCENARIO_RUNNER_PATH=/opt/scenario_runner

# setup python path
RUN echo /catkin_ws/install/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /catkin_ws/devel/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/ros/noetic/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/ros/noetic/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/leaderboard-py3 >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/carla/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/carla/PythonAPI/carla >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth

RUN sudo mkdir /workspace && sudo chown $USERNAME:$USERNAME /workspace

RUN echo "export CUDA_HOME=/usr/local/cuda-11" >> ~/.bashrc \
    && echo "export PATH=$PATH:$CUDA_HOME/bin" >> ~/.bashrc

COPY --chown=$USERNAME:$USERNAME ./code/requirements.txt /workspace/
ENV TORCH_CUDA_ARCH_LIST="5.2 6.0 6.1 7.0 7.5 8.0 8.6 8.7 8.9+PTX"
ENV IABN_FORCE_CUDA=1

RUN source ~/.bashrc && pip install -r /workspace/requirements.txt

#RUN git clone --branch main https://github.com/hirotomusiker/CLRerNet.git /opt/CLRerNet_model && \
#    chown -R $USERNAME:$USERNAME ./code/perception/src/CLRerNet_model && \
#    echo "Clone erfolgreich!"

#RUN sudo wget https://github.com/hirotomusiker/CLRerNet/releases/download/v0.1.0/clrernet_culane_dla34_ema.pth -O ./code/perception/src/CLRerNet_model/clrernet_culane_dla34_ema.pth

# Add agent code
COPY --chown=$USERNAME:$USERNAME ./code /workspace/code/
#COPY /code/perception/src/CLRerNet_model /workspace/code/perception/src/CLRerNet_model/






# For debugger
ENV PAF_CATKIN_CODE_ROOT=/catkin_ws/src/code
# Link code into catkin workspace
RUN ln -s -T /workspace/code ${PAF_CATKIN_CODE_ROOT}

# re-make the catkin workspace
RUN source /opt/ros/noetic/setup.bash && catkin_make

ADD ./build/docker/agent/entrypoint.sh /entrypoint.sh

# set the default working directory to the code
WORKDIR /workspace/code

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "sleep 10 && \
    python3 /opt/leaderboard/leaderboard/leaderboard_evaluator.py \
    --debug=${DEBUG_CHALLENGE} \
    --repetitions=${REPETITIONS} \
    --checkpoint=${CHECKPOINT_ENDPOINT} \
    --track=${CHALLENGE_TRACK} \
    --agent=${TEAM_AGENT} \
    --routes=${ROUTES} \
    --host=${CARLA_SIM_HOST}"]
