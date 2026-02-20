#!/bin/bash
set -euo pipefail

MODE="${1:-sync}"

if [[ "${MODE}" != "sync" && "${MODE}" != "check" ]]; then
  echo "Usage: dependency-sync.sh [sync|check]"
  exit 2
fi

if [[ -z "${PAF_ROS_WS:-}" || -z "${INTERNAL_WORKSPACE_DIR:-}" || -z "${ROS_DISTRO:-}" ]]; then
  echo "PAF_ROS_WS, INTERNAL_WORKSPACE_DIR and ROS_DISTRO must be set."
  exit 1
fi

source "${INTERNAL_WORKSPACE_DIR}/env.bash"
cd "${PAF_ROS_WS}"

echo "Refreshing apt and rosdep indices..."
sudo apt-get update
rosdep update --rosdistro "${ROS_DISTRO}"

if [[ "${MODE}" == "check" ]]; then
  echo "Running rosdep check for workspace package.xml dependencies..."
  rosdep check --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}"

  echo "Running pip health check..."
  python3 -m pip check

  cat <<EOF
Dependency check completed.
- rosdep dependencies are satisfied for current package.xml manifests.
- pip reports no broken package requirements for this environment.
EOF
  exit 0
fi

echo "Installing ROS package.xml dependencies with rosdep..."
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y -r

echo "Installing Python requirements (requirements*.txt + infrastructure)..."
"${INTERNAL_WORKSPACE_DIR}/scripts/install-python-requirements.sh"

echo "Running pip health check..."
python3 -m pip check

cat <<EOF
Dependency synchronization completed.
If dependencies changed for your package, rebuild the workspace with:
  devbuild
EOF
