# Praktikum Autonomes Fahren 2023 - PAF23

## Prerequisites

To be able to execute and develop the project, you need a Linux system equipped with:

- NVIDIA GPU (everything >= RTX 3070 should be fine)
- A minimum of 16G of RAM - A minimum of 100G of free disk space

As the project is still in early development, these requirements are subject to change.

## Installation

To run the project you have to install [b5](https://github.com/team23/b5)
and [docker](https://docs.docker.com/engine/install/) with NVIDIA GPU support,
[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

Afterwards, you can set up and execute the project with the following two commands:

```shell
# Setup project
b5 install

# Run project
b5 run
```

More detailed instruction about setup and execution can be found [here](./doc/01_general/Readme.md).

More available b5 commands are documented [here](./doc/01_general/03_commands.md).

## Development

If you contribute to this project please read the guidelines first. They can be found [here](./doc/02_development/Readme.md).

## Research

The research of existing projects we did can be found [here](./doc/03_research/Readme.md).
