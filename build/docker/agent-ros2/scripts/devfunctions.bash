#!/bin/bash
# This script contains development helper functions

# This message prints on every interactive bash shell that opens
cat <<EOF
Development utility functions:
- devsource: sources the ${INTERNAL_WORKSPACE_DIR}/env.bash (includes the ${PAF_ROS_WS}/install/local_setup.bash)
- devbuild: colcon builds the ${PAF_ROS_WS} (colcon build --symlink-install)
EOF

# This function sources the ROS /workspace
devsource() {
  source "${INTERNAL_WORKSPACE_DIR}/env.bash"
  echo "${INTERNAL_WORKSPACE_DIR}/env.bash sourced."
}
export -f devsource

# This function builds the ROS /workspace and sources it again afterwards
devbuild() {
  CURRENT_DIR=${PWD}

  cd "${PAF_ROS_WS}" || return $?

  # Build the ros workspace
  colcon build --symlink-install || return $?

  cat <<EOF

ROS workspace built successfully.
Do not forget to source the ros workspace again if you changed anything in the package.xml or setup.py files.
This shell window will do it automatically. In other open shells run: devsource
EOF

  devsource || return $?

  cd "${CURRENT_DIR}" || return $?
}
export -f devbuild
