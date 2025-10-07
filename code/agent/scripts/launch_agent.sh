#!/bin/bash
set -e

# Remove some arguments ros2 launch automatically adds
args=("$@")
for ((i=0; i<"${#args[@]}"; ++i)); do
    case ${args[i]} in
        --ros-args) unset "args[i]";;
        -r) unset "args[i]";;
    esac
done

# Reset full ROS/Python environment
source "${INTERNAL_WORKSPACE_DIR}/scripts/reset_env.bash"
reset_env

# Source main workspace environment
source "${INTERNAL_WORKSPACE_DIR}/env.bash"

# launch
exec ros2 launch "${args[@]}"
