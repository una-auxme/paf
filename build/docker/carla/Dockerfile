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
# FROM nvcr.io/nvidia/cuda:12.6.3-runtime-ubuntu24.04 AS carla-base-cuda
# The cuda runtime image breaks vulkan support
FROM ubuntu:24.04 AS carla-base-cuda
# GPU vulkan driver install
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y libvulkan1 vulkan-tools libnvidia-gl-570

############################################
##### The main agent image starts here #####
############################################
FROM carla-base-${BASE_FLAVOUR} AS carla-base

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  xz-utils

ENV CARLA_ROOT=/opt/carla
RUN --mount=type=bind,target=/opt/carla.tar.xz,source=build/docker/carla/CARLA_Leaderboard_2.0.tar.xz \
  mkdir -p ${CARLA_ROOT} && \
  tar --no-same-owner -xf /opt/carla.tar.xz -C ${CARLA_ROOT} --strip-components=1
WORKDIR ${CARLA_ROOT}
RUN chmod +x CarlaUE4.sh && chmod +x ./CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping


FROM alpine as carla-api

COPY --from=carla-base /opt/carla/PythonAPI /opt/carla/PythonAPI

FROM carla-base as carla

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt install -y \
  xdg-user-dirs xdg-utils
