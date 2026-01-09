# Execution

This document provides an overview of how to execute the project,
detailing the purpose and usage of the various configuration files located in the [build](../../build/) directory.
The project utilizes Docker and Docker Compose to manage services and dependencies,
facilitating both normal and distributed execution modes.

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Quick Start](#quick-start)
- [Directory Structure](#directory-structure)
- [Docker Compose Files](#docker-compose-files)
  - [`docker-compose.carla.base.yaml`](#docker-composecarlabaseyaml)
  - [`docker-compose.dev.base.yaml`](#docker-composedevbaseyaml)
  - [`docker-compose.deploy.base.yaml`](#docker-composedeploybaseyaml)
  - [`docker-compose.linter.yaml`](#docker-composelinteryaml)
  - [`docker-compose.docs.yaml`](#docker-composedocsyaml)
- [Execution Modes](#execution-modes)
  - [Normal Execution](#normal-execution)
  - [Distributed Execution](#distributed-execution)
- [Usage](#usage)
- [Notes](#notes)
- [Conclusion](#conclusion)

## Quick Start

After cloning the paf repository, make sure to execute the `./user_setup.sh` in the project root once.
This sets up important docker compose environment variables.

In order to start the default leaderboard execution simply navigate to the [build](../../build/) folder and select the `Compose up` option in the right-click menu of the `docker-compose.dev.<your-gpu-type>.yml` file. As `<your-gpu-type>` `cuda` should be used for the lab PCs.

## Directory Structure

The `build` directory contains the necessary configuration and setup files for building and running the project services. Below is an overview of the key files:

- **Docker Compose Files**
  - `docker-compose.carla.base.yaml`
  - `docker-compose.carla.<your-gpu-type>.yaml`
  - `docker-compose.dev.base.yaml`
  - `docker-compose.dev.<your-gpu-type>.yaml`
  - `docker-compose.linter.yaml`
  - `docker-compose.docs.yaml`

## Docker Compose Files

The base compose files (`docker-compose.*.base.yaml`) define the configurations for individual services used in the project. These files are extended in platform specific configuration files to adapt the development environment to different hosts.
For the lab, only the `docker-compose.*.cuda.yaml` configurations should be executed, but the base configuration files contain pretty much all relevant parameters for development.

### `docker-compose.carla.base.yaml`

Defines the configuration for the `carla-simulator` service, which runs the CARLA simulator. Key configurations include:

- **Image**: Uses the CARLA simulator image tailored for the project. The image can be built manually with [`build_carla.sh`](../../build/docker/carla/build_carla.sh), but is pulled from the container registry by default.
- **Command**: Starts the simulator with specific settings such as resolution, quality level, and disabling sound.
- **Environment Variables**: Sets up desktop→docker pass-through
- **Volumes**:
  - Desktop: X11 UNIX socket + `${XDG_RUNTIME_DIR}`
  - Custom CARLA settings
- **Networks**: Connects to the `carla` network.

`docker-compose.carla.<your-gpu-type>.yaml` configurations inherit their settings from `docker-compose.carla.base.yaml` and properly set up the GPU drivers for a specific vendor.

### `docker-compose.dev.base.yaml`

**Purpose**: Provides a container for development. Starts a VS Code instance inside the container. Leaderboard and agent have to be started manually inside the container with `leaderboard.dev` and `agent.dev` respectively.

- **Image**: Custom [ubuntu24.04+ros2 jazzy Dockerfile](../../build/docker/agent-ros2/Dockerfile) set up for starting the leaderboard and the agent. This configuration uses the **agent-dev** target of the Dockerfile.
- **Includes**:
  - `docker-compose.carla.base.yaml`
- **Environment Variables**:
  - Sets up desktop→docker pass-through
  - CARLA_SIM_HOST points to the carla-simulator
- **Volumes**:
  - Desktop: X11 UNIX socket + `${XDG_RUNTIME_DIR}`
  - Paf project root to `/workspace`
  - `<prj.root>/volumes/home` to `/home`: provides a persistent home directory in the container. -> Configuration files for RViz/Rqt can persist.
- **Networks**: Connects to the `carla` network.
- **Notes**:
  - All important ROS2 files in the container (Like ROS workspaces, leaderboard, logs, etc...) can be found under `/internal_workspace`

### `docker-compose.deploy.base.yaml`

**Purpose**: Provides a container that autonomously executes the leaderboard+agent with a given route. It exits when the leaderboard is finished. Intended for evaluations, CI/CD. NOT intended for live development.

- **Image**: Shares the same custom [ubuntu24.04+ros2 jazzy Dockerfile](../../build/docker/agent-ros2/Dockerfile) with `docker-compose.dev.base.yaml`. But this configuration uses the agent-deploy target of the Dockerfile.
- **Notable differences to `docker-compose.dev.base.yaml`**:
  - The image build fails fully when pip or rosdep dependency installation fails
  - The image automatically starts `leaderboard.deploy` on startup
  - `leaderboard.deploy` also includes/starts the agent
  - The project files are copied into the image in the build stage and also built in the build stage
  - The project directory is not mounted into the container
- **Includes**:
  - `docker-compose.carla.base.yaml`
- **Environment Variables**:
  - Sets up desktop→docker pass-through
  - CARLA_SIM_HOST points to the carla-simulator
- **Volumes**:
  - Desktop: X11 UNIX socket + `${XDG_RUNTIME_DIR}`
- **Networks**: Connects to the `carla` network.

### `docker-compose.linter.yaml`

**Purpose**: Defines services for code linting and static analysis. Includes:

- **ruff-check**: Python linting with the Ruff version pinned in `build/pins/ruff.env`.
- **ruff-format**: Python formatting with Ruff.
- **mdlint**: Markdown file linting.
- **Volumes**: Mounts the project directory for linting files within the container.

### `docker-compose.docs.yaml`

**Purpose**: Rebuilds part of the documentation with pydoc-markdown. Currently only used for `mapping`:

- **Volumes**: The project is mounted to `/workspace`

## Execution Modes

The project supports two primary execution modes, suitable for different development and deployment scenarios.

### Normal Execution

In normal execution mode, all services (agent, CARLA simulator) run on a single machine. This mode is suitable for:

- Development and testing with smaller models.
- Scenarios where the machine's resources (especially VRAM) are sufficient to handle both the agent's computation and the simulator.

### Distributed Execution

> !! Distributed Execution is currently NOT set up for ROS2 and has to be ported!!

Distributed execution separates the agent and the CARLA simulator onto different machines. This is necessary when:

- Running large vision models that require extensive VRAM.
- The single machine's resources are insufficient to handle both the agent and simulator.

**Note**: In distributed execution, the CARLA simulator must be running on a second desktop PC, and the `CARLA_SIM_HOST` environment variable should be set accordingly. Further information can be found in [here](../development/distributed_simulation.md).  

## Usage

To run the project using the provided Docker Compose files simply navigate to the files in the VS Code Explorer and select `Compose Up` after right-clicking the file.

## Notes

- Ensure that you have proper GPU support configured if running models that require GPU acceleration.
- The `docker-compose.dev.base.yaml` and other base service files are crucial for defining the common configurations and should not be modified unless necessary.
- When running in distributed mode, update the `CARLA_SIM_HOST` environment variable in the appropriate service configurations to point to the simulator's IP address.
- The linter services defined in `docker-compose.linter.yaml` can be used to maintain code quality and should be run regularly during development.

## Conclusion

This documentation should help you understand the build structure and how to
work with the different Docker configurations provided in the project.
Whether you are developing locally, running complex simulations,
or integrating with CI/CD pipelines, these configurations are designed to facilitate a smooth workflow.
