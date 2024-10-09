# 🛠️ Installation

To run the project you have to install [docker](https://docs.docker.com/engine/install/) with NVIDIA GPU support, [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

For development, we recommend Visual Studio Code with the plugins that are recommended inside the `.vscode` folder.

## Installation

If not yet installed first install docker as described in section [Docker with NVIDIA GPU support](#docker-with-nvidia-gpu-support).

## Docker with NVIDIA GPU support

For this installation, it's easiest to follow the guide in the [NVIDIA docs](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

For simplicity, we list the necessary steps here:

### Docker

Install Docker using the convenience script.

```shell
curl https://get.docker.com | sh \
 && sudo systemctl --now enable docker
```

### Allow non-root user to execute Docker commands

We recommend this step, to not have to execute every command as root.

```shell
# add docker group
sudo groupadd docker

# add your current user to the docker group
sudo usermod -aG docker $USER
```

After this, _restart_ your system to propagate the group changes.

### NVIDIA Container toolkit

Setup the package repository and the GPG key:

```shell
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Install the `nvidia-docker2` package (and dependencies) after updating the package listing:

```shell
sudo apt-get update
sudo apt-get install -y nvidia-docker2
```

Restart the Docker daemon to complete the installation after setting the default runtime:

```shell
sudo systemctl restart docker
```

## 🚨 Common Problems

### Vulkan device not available

Cannot find a compatible Vulkan Device.
Try updating your video driver to a more recent version and make sure your video card supports Vulkan.

![Vulkan device not available](../assets/vulkan_device_not_available.png)

Verify the issue with the following command:

```shell
$ docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
Failed to initialize NVML: Unknown Error
```

> [!TIP] Solution found in <https://stackoverflow.com/a/78137688>

```shell
sudo vim /etc/nvidia-container-runtime/config.toml
```

, then changed `no-cgroups = false`, save

Restart docker daemon:

```shell
sudo systemctl restart docker
```

, then you can test by running

```shell
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

Based on:

1. <https://bobcares.com/blog/docker-failed-to-initialize-nvml-unknown-error/>
2. <https://bbs.archlinux.org/viewtopic.php?id=266915>
