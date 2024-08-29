#!/bin/bash

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Source the catkin workspace setup
source /catkin_ws/devel/setup.bash

# Set up any additional environment variables if needed
export CARLA_ROOT=/opt/carla
export SCENARIO_RUNNER_ROOT=/opt/scenario_runner
export LEADERBOARD_ROOT=/opt/leaderboard

# Print a welcome message
echo "Welcome to the CARLA development environment!"
echo "ROS Noetic and CARLA dependencies are set up."
echo "Your workspace is at /workspace/code"

# Execute the command passed to the script, or start a bash session if no command was given
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi