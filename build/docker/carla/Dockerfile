# Currently supported BASE_FLAVOUR:
# - gpu: base for amd + intel (vulkan and opengl drivers only)
# - cuda: nvidia
ARG BASE_FLAVOUR=gpu
ARG BASE_IMAGE=ubuntu:24.04

FROM ${BASE_IMAGE} AS carla-base-gpu

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y software-properties-common wget curl

# GPU vulkan driver install (amd + intel)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  add-apt-repository -y ppa:kisak/turtle && apt-get update && \
  apt-get upgrade -y && apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers

FROM carla-base-gpu AS carla-base-cuda
ARG NVIDIA_DRIVER_VERSION=550
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.17.5/docker-specialized.html#nvidia-require-constraints
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_REQUIRE_DRIVER="driver>=${NVIDIA_DRIVER_VERSION}"

# Cuda toolkit install
# Using the cuda-toolkit instead of the ubuntu-packaged nvidia-cuda-toolkit avoids pulling in the libnvidia-compute-535 driver package.
# (It is a dependency of nvidia-cuda-toolkit, but not of cuda-toolkit)
# The libnvidia-compute-535 driver can conflict with the host's driver
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  UBUNTU_VERSION=$(. /etc/os-release && echo $VERSION_ID | tr -d '.') && \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${UBUNTU_VERSION}/x86_64/cuda-keyring_1.1-1_all.deb && \
  dpkg -i cuda-keyring_1.1-1_all.deb && \
  rm cuda-keyring_1.1-1_all.deb
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get upgrade -y && \
  apt-get install -y cuda-toolkit libxext6

############################################
##### The main carla image starts here #####
############################################
FROM carla-base-${BASE_FLAVOUR} AS carla

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  xz-utils pigz

ENV CARLA_ROOT=/opt/carla
RUN --mount=type=bind,target=/opt/carla.tar.gz,source=build/docker/carla/carla.tar.gz \
  mkdir -p ${CARLA_ROOT} && \
  tar --use-compress-program=pigz --no-same-owner -xf /opt/carla.tar.gz -C ${CARLA_ROOT}
WORKDIR ${CARLA_ROOT}
RUN --mount=type=bind,target=${CARLA_ROOT}/Import/carla-additional-maps.tar.gz,source=build/docker/carla/carla-additional-maps.tar.gz \
  tar --use-compress-program=pigz --no-same-owner --skip-old-files -xf ./Import/carla-additional-maps.tar.gz -C ${CARLA_ROOT}
# Tar with --keep-newer-files in ImportAssets.sh leads to errors: "Unexpected inconsistency when making directory"
# chmod +x ImportAssets.sh && ./ImportAssets.sh
RUN chmod +x CarlaUE4.sh && chmod +x ./CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  xdg-user-dirs xdg-utils

COPY --chmod=755 build/docker/carla/carla_entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

FROM ${BASE_IMAGE} AS carla-api

COPY --from=carla /opt/carla/PythonAPI /opt/carla/PythonAPI
