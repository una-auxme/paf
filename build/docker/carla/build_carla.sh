#!/bin/bash
set -e
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
cd "${SCRIPT_DIR}"

if [ ! -f ./CARLA_Leaderboard_2.0.tar.xz ]; then
  wget --progress=bar https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz -O ./CARLA_Leaderboard_2.0.tar.xz
fi

docker build --pull -t carla-leaderboard-gpu:2.1 --target carla -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-cuda:2.1 --target carla --build-arg BASE_FLAVOUR=cuda -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-api:2.1 --target carla-api -f ./Dockerfile ../../../
docker build --pull -t carla-leaderboard-ros-bridge:2.1 --target carla-ros-bridge -f ./Dockerfile ../../../
