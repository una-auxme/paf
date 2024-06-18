# Docker Swarm Notes

```bash
GPU-0f76ba56

sudo docker service create --replicas 1 \
  --cap-add all \
  --network host \
  --name carla-server \
  --generic-resource "NVIDIA-GPU=0" \
  -e DISPLAY=$DISPLAY \
  carlasim/carla:latest \
  /bin/bash ./CarlaUE4.sh


sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:latest /bin/bash ./CarlaUE4.sh

  docker service create --replicas 1 \
  --name tensor-qs \
  --generic-resource "gpu=1" \
  tomlankhorst/tensorflow-quickstart


  -e XDG_RUNTIME_DIR=/tmp/runtime-carla \


sudo docker service create \
  --name carla-server \
  --mount type=bind,source=/var/run/docker.sock,target=/var/run/docker.sock,ro \
  docker \
  docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY ghcr.io/una-auxme/paf23:leaderboard-2.0 /bin/bash ./CarlaUE4.sh

```
