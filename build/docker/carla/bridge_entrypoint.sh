#!/bin/bash
set -e
# Required otherwise the leaderboard does not reliably print to the console when started from a shell script
export PYTHONUNBUFFERED=true

source /.env.bash

python3 /wait_for_carla.py

command=()
for arg in "$@"; 
do
    command+=("$(echo "$arg" | envsubst)")
done

exec "${command[@]}"
