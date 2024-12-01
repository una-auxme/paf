#!/bin/bash
set -e
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

VENV_DIR="$1"
ADDED_REQUIREMENTS_CMD="${@:2}"

ADDED_REQUIREMENTS=()
for requirement in ${ADDED_REQUIREMENTS_CMD}; do
  ADDED_REQUIREMENTS+=(-r ${requirement})
done

# We need system packages for rosdep dependencies :(
python3 -m venv --system-site-packages ${VENV_DIR}
echo "source ${VENV_DIR}/bin/activate" >> ~/.bashrc
source ${VENV_DIR}/bin/activate

echo Python executable: $(which python)
echo Pip executable: $(which pip)
echo Added requirements: ${ADDED_REQUIREMENTS[*]}

pip_tmpdir=~/.pip-tmpdir-distrobox
mkdir -p ${pip_tmpdir}

pip install -U wheel pip
TMPDIR=${pip_tmpdir} pip install -U -r ${SCRIPT_DIR}/requirements_leaderboard.txt -r ${SCRIPT_DIR}/requirements_torch.txt -r ${SCRIPT_DIR}/requirements_ros.txt -r ${SCRIPT_DIR}/requirements_code.txt ${ADDED_REQUIREMENTS[*]}
rm -rf ${pip_tmpdir}

