# Praktikum Autonomes Fahren - PAF

This repository contains the source code for the "Praktikum Autonomes Fahren" at the Chair of Mechatronics from the University of Augsburg.
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

To run the project you have to install [docker](https://docs.docker.com/engine/install/) with NVIDIA GPU support,
[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).
`docker` and `nvidia-docker` are used to run the project in a containerized environment with GPU support.

More detailed instructions about setup and execution can be found [here](./doc/general/Readme.md).

## Development

If you contribute to this project please read the guidelines first. They can be found [here](./doc/development/Readme.md).

## Research

The research on existing projects we did can be found [here](./doc/research/Readme.md).
