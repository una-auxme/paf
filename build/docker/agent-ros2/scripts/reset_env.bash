#!/bin/bash

# This function removes both the loaded (sourced) ROS workspace and
# any python venv from the shell's environment variables
reset_env() {
  # Unset ROS and Python env variables
  unset AMENT_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset COLCON_PREFIX_PATH
  unset PYTHONPATH
  unset VIRTUAL_ENV_PROMPT
  unset VIRTUAL_ENV
  unset LD_LIBRARY_PATH
  if [[ ! -f "${INTERNAL_WORKSPACE_DIR}/default_PATH" ]]; then
    echo "Error: default_PATH file not found at ${INTERNAL_WORKSPACE_DIR}/default_PATH" >&2
    return 1
  fi
  PATH=$(cat "${INTERNAL_WORKSPACE_DIR}/default_PATH")
  export PATH
}
export -f reset_env
