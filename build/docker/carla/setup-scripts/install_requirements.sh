#!/bin/bash
set -e

VENV_DIR=$1
ADDED_REQUIREMENTS_CMD=("${@:2}")

ADDED_REQUIREMENTS=()
for requirement in "${ADDED_REQUIREMENTS_CMD[@]}"; do
  ADDED_REQUIREMENTS+=(-r "${requirement}")
done

source "${VENV_DIR}/bin/activate"

echo Python executable: "$(which python)"
echo Pip executable: "$(which pip)"
echo Added requirements: "${ADDED_REQUIREMENTS[@]}"

pip_tmpdir=~/.pip-tmpdir-docker
mkdir -p ${pip_tmpdir}

# TMPDIR=${pip_tmpdir} pip install -U pip setuptools
TMPDIR=${pip_tmpdir} pip install -U "${ADDED_REQUIREMENTS[@]}"
rm -rf ${pip_tmpdir}
