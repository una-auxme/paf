#!/bin/bash
set -e

VENV_DIR=$1

python3 -m venv "${VENV_DIR}"

source "${VENV_DIR}/bin/activate"

echo Python executable: "$(which python)"
echo Pip executable: "$(which pip)"

pip install -U wheel pip
