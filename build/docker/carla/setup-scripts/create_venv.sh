#!/bin/bash
set -e

VENV_DIR=$1

# We need system packages for rosdep dependencies :(
python3 -m venv --system-site-packages "${VENV_DIR}"

source "${VENV_DIR}/bin/activate"

echo Python executable: "$(which python)"
echo Pip executable: "$(which pip)"

pip install -U wheel pip
