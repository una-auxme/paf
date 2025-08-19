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

if [ ${#REQUIREMENTS[@]} -gt 0 ]; then
  pip install "${REQUIREMENTS[@]}"
else
  echo No python requirements found.
fi
