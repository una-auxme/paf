# Praktikum Autonomes Fahren 2023 - PAF23

This repository contains the source code for the "Praktikum Autonomes Fahren" at the Chair of Mechatronics from the University of Augsburg in the winter semester of 2023/2024.
The goal of the project is to develop a self-driving car that can navigate through a simulated environment.
The project is based on the [CARLA simulator](https://carla.org/) and uses the [ROS](https://www.ros.org/) framework for communication between the different components.
In the future, the project aims to contribute to the [CARLA Autonomous Driving Challenge](https://leaderboard.carla.org/challenge/).
The current maintainers of the project are @JulianTrommer and @ll7.

## Video of the latest agent

The current state as of 2024-03-25 of the project can be seen in the following video:

[![Video of the latest agent](https://img.youtube.com/vi/2sR87lO9-Aw/0.jpg)](https://www.youtube.com/watch?v=2sR87lO9-Aw)

## Prerequisites

To be able to execute and develop the project, you need a Linux system equipped with:

- NVIDIA GPU (everything >= RTX 3070 should be fine)
- A minimum of 16G of RAM - A minimum of 100G of free disk space

As the project is still in early development, these requirements are subject to change.

## Installation

To run the project you have to install [b5](https://github.com/team23/b5)
and [docker](https://docs.docker.com/engine/install/) with NVIDIA GPU support,
[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).
`b5` is used to simplify some of the docker commands and to provide a more user-friendly interface.
`docker` and `nvidia-docker` are used to run the project in a containerized environment with GPU support.

Afterwards, you can set up and execute the project with the following two commands:

```bash
# Setup project
b5 install

# Run project
b5 run
```

More detailed instructions about setup and execution can be found [here](./doc/01_general/Readme.md).

More available b5 commands are documented [here](./doc/01_general/03_commands.md).

## Development

If you contribute to this project please read the guidelines first. They can be found [here](./doc/02_development/Readme.md).

## Research

The research on existing projects we did can be found [here](./doc/03_research/Readme.md).
