#!/bin/bash
set -e

if [ -z "${PAF_ROS_WS}" ]; then
  echo PAF_ROS_WS is not set
  exit 1
fi

cd "${PAF_ROS_WS}/src"

REQUIREMENTS=()

# Based on https://stackoverflow.com/questions/23356779/how-can-i-store-the-find-command-results-as-an-array-in-bash/54561526#54561526
readarray -d '' REQUIREMENT_FILES < <(find . -name requirements.txt -print0)
for i in "${REQUIREMENT_FILES[@]}"; do
  REQUIREMENTS+=(-r "$i")
  echo Found python requirements: "$i"
done

if [ -n "$BASE_FLAVOUR" ]; then
  # Only rocm and cuda flavours use special requirement files.
  if [ "$BASE_FLAVOUR" == "generic" ]; then
    BASE_FLAVOUR="cpu"
  elif [ "$BASE_FLAVOUR" == "gpu" ]; then
    BASE_FLAVOUR="cpu"
  fi

  readarray -d '' REQUIREMENT_FILES < <(find . -name requirements."$BASE_FLAVOUR".txt -print0)
  for i in "${REQUIREMENT_FILES[@]}"; do
    REQUIREMENTS+=(-r "$i")
    echo Found "$BASE_FLAVOUR" python requirements: "$i"
  done
fi

# Install infrastructure requirements
readarray -d '' REQUIREMENT_FILES < <(find . -name requirements_infrastructure.txt -print0)
for i in "${REQUIREMENT_FILES[@]}"; do
  REQUIREMENTS+=(-r "$i")
  echo Found infrastructure python requirements: "$i"
done

if [ ${#REQUIREMENTS[@]} -gt 0 ]; then
  pip_tmpdir=~/.pip-tmpdir-docker
  mkdir -p ${pip_tmpdir}

  TMPDIR=${pip_tmpdir} pip install --user "${REQUIREMENTS[@]}"
  rm -rf ${pip_tmpdir}
else
  echo No python requirements found.
fi
