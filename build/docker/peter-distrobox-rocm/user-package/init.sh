#!/bin/bash
set -e
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

INIT_SETUP_DONE_FILE=${USER_PACKAGE_DIR}/.init_setup_done
if [ -e ${INIT_SETUP_DONE_FILE} ]; then
  exit 0
fi

# Make sure HOME is under a Distrobox subfolder so we do not accidently delete the users bashrc
[[ "$HOME" =~ .*Distrobox.* ]]
if [ -f ~/.bashrc ]; then
  rm ~/.bashrc
fi

# Setup python venv
USER_VENV_DIR=${INTERNAL_WORKSPACE_DIR}/venv
${USER_PACKAGE_DIR}/create_venv.sh ${USER_VENV_DIR}
source ${USER_VENV_DIR}/bin/activate

# Setup additional dependencies
echo ${CARLA_ROOT}/PythonAPI/carla >> ${USER_VENV_DIR}/lib/python3.8/site-packages/carla.pth
echo ${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg >> ${USER_VENV_DIR}/lib/python3.8/site-packages/carla.pth
echo ${SCENARIO_RUNNER_ROOT} >> ${USER_VENV_DIR}/lib/python3.8/site-packages/leaderboard.pth
echo ${LEADERBOARD_ROOT} >> ${USER_VENV_DIR}/lib/python3.8/site-packages/leaderboard.pth

# ROS setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

cd ${ROOT_CATKIN_WS}
catkin_make
echo "source ${ROOT_CATKIN_WS}/devel/setup.bash" >> ~/.bashrc
source ${ROOT_CATKIN_WS}/devel/setup.bash

# Workspace setup
ln -s -T ${WORKSPACE_DIR} ${INTERNAL_WORKSPACE_DIR}/workspace
USER_CATKIN_WS=${INTERNAL_WORKSPACE_DIR}/catkin_ws
mkdir -p ${USER_CATKIN_WS}
ln -s -T ${WORKSPACE_DIR}/code ${USER_CATKIN_WS}/src
cd ${USER_CATKIN_WS}
catkin_make
echo "source ${USER_CATKIN_WS}/devel/setup.bash" >> ~/.bashrc
echo "export PAF_CATKIN_CODE_ROOT=${USER_CATKIN_WS}/src" >> ~/.bashrc
source ${USER_CATKIN_WS}/devel/setup.bash

# Enable services
if [ -n "${XDG_CONFIG_HOME}" ]; then
  if [ -d "${XDG_CONFIG_HOME}/services" ]; then
    rm -r "${XDG_CONFIG_HOME}/services"
  elif [ -h "${XDG_CONFIG_HOME}/services" ]; then
    rm "${XDG_CONFIG_HOME}/services"
  fi

  ln -s -T ${USER_PACKAGE_DIR}/services "${XDG_CONFIG_HOME}/services"
fi

touch ${INIT_SETUP_DONE_FILE}

echo "alias leaderboard=${USER_PACKAGE_DIR}/run_leaderboard.sh" >> ~/.bashrc
