ARG BASE_FLAVOUR=cuda

FROM ubuntu:24.04 AS carla-base-rocm

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y software-properties-common

# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository -y ppa:kisak/turtle && apt-get update && \
  apt-get upgrade -y && apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda
FROM nvcr.io/nvidia/cuda:12.6.3-runtime-ubuntu24.04 AS carla-base-cuda
# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y libvulkan1 vulkan-tools libnvidia-gl-570

# Weird bug: https://github.com/NVIDIA/nvidia-container-toolkit/issues/1041
RUN cp /usr/share/vulkan/icd.d/nvidia_icd.json /etc/vulkan/icd.d/ && \
  cp /usr/share/vulkan/implicit_layer.d/nvidia_layers.json /etc/vulkan/implicit_layer.d/

############################################
##### The main agent image starts here #####
############################################
FROM carla-base-${BASE_FLAVOUR} AS carla

ENV DEBIAN_FRONTEND=noninteractive

ENV CARLA_ROOT=/opt/carla
RUN --mount=type=bind,target=/opt/carla.tar.gz,source=build/docker/carla/carla.tar.gz,rw \
  chown root:root /opt/carla.tar.gz && \
  mkdir -p ${CARLA_ROOT} && \
  tar -xf /opt/carla.tar.gz -C ${CARLA_ROOT}
WORKDIR ${CARLA_ROOT}
RUN chmod +x CarlaUE4.sh && chmod +x ./CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping
# RUN --mount=type=bind,target=${CARLA_ROOT}/Import/additional-maps.tar.gz,source=build/docker/carla/additional-maps.tar.gz,rw \
#   chown root:root ${CARLA_ROOT}/Import/additional-maps.tar.gz && \
#   chmod +x ./ImportAssets.sh && ./ImportAssets.sh

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt install -y \
  xdg-user-dirs xdg-utils
