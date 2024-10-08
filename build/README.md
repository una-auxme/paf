# Build Directory Documentation

This document provides an overview of the build structure of the project,
detailing the purpose and usage of the various configuration files located in the `build` directory.
The project utilizes Docker and Docker Compose to manage services and dependencies,
facilitating both normal and distributed execution modes.

## Table of Contents

- [Build Directory Documentation](#build-directory-documentation)
  - [Table of Contents](#table-of-contents)
  - [Directory Structure](#directory-structure)
  - [Base Service Files](#base-service-files)
    - [`agent_service.yaml`](#agent_serviceyaml)
    - [`carla-simulator_service.yaml`](#carla-simulator_serviceyaml)
    - [`linter_services.yaml`](#linter_servicesyaml)
    - [`roscore_service.yaml`](#roscore_serviceyaml)
  - [Docker Compose Files](#docker-compose-files)
    - [`docker-compose.yaml`](#docker-composeyaml)
    - [`docker-compose_dev.yaml`](#docker-compose_devyaml)
    - [`docker-compose_cicd.yaml`](#docker-compose_cicdyaml)
  - [Execution Modes](#execution-modes)
    - [Normal Execution](#normal-execution)
    - [Distributed Execution](#distributed-execution)
  - [Usage](#usage)
  - [Notes](#notes)
  - [Conclusion](#conclusion)

## Directory Structure

The `build` directory contains the necessary configuration and setup files for building and running the project services. Below is an overview of the key files:

- **Base Service Files**
  - `agent_service.yaml`
  - `carla-simulator_service.yaml`
  - `linter_services.yaml`
  - `roscore_service.yaml`
- **Docker Compose Files**
  - `docker-compose.yaml`
  - `docker-compose_dev.yaml`
  - `docker-compose_cicd.yaml`

## Base Service Files

The base service files define the configurations for individual services used in the project. These files are included or extended in the Docker Compose files to create different execution setups.

### `agent_service.yaml`

Defines the configuration for the `agent` service, which represents the autonomous driving agent. Key configurations include:

- **Build Settings**: Specifies the Dockerfile and build arguments for creating the agent image.
- **Environment Variables**: Sets up necessary environment variables like `DISPLAY`, `ROS_MASTER_URI`, and `CARLA_SIM_HOST`.
- **Volumes**: Mounts directories like `/workspace` to share code and data between the host and the container.
- **Networks**: Connects the agent to the `carla` and `ros` networks.

### `carla-simulator_service.yaml`

Defines the configuration for the `carla-simulator` service, which runs the CARLA simulator. Key configurations include:

- **Image**: Uses the CARLA simulator image tailored for the project.
- **Command**: Starts the simulator with specific settings such as resolution, quality level, and disabling sound.
- **Environment Variables**: Sets variables like `DISPLAY` and NVIDIA capabilities.
- **Volumes**: Shares the X11 UNIX socket and custom CARLA settings.
- **Networks**: Connects to the `carla` network.

### `linter_services.yaml`

Defines services for code linting and static analysis. Includes:

- **flake8**: For Python linting.
- **mdlint**: For Markdown file linting.
- **Volumes**: Mounts the project directory for linting files within the container.

### `roscore_service.yaml`

Defines the `roscore` service for running the ROS master node. Key configurations include:

- **Image**: Uses the official ROS Noetic image.
- **Command**: Starts `roscore`.
- **Environment Variables**: Sets up ROS networking variables.
- **Networks**: Connects to the `ros` network.

## Docker Compose Files

The Docker Compose files orchestrate multiple services defined in the base service files, allowing for different execution scenarios.

### `docker-compose.yaml`

- **Includes**:
  - `linter_services.yaml`
  - `roscore_service.yaml`
  - `carla-simulator_service.yaml`
- **Services**:
  - Extends the `agent` service from `agent_service.yaml`.
- **Purpose**: Runs the agent with special scenarios included. Solving these scenarios is the primary goal of the project.

### `docker-compose_dev.yaml`

- **Includes**:
  - `linter_services.yaml`
  - `roscore_service.yaml`
  - `carla-simulator_service.yaml`
- **Services**:
  - Extends the `agent` service from `agent_service.yaml`.
- **Environment Overrides**:
  - Sets `ROUTE` to a simple route file (`routes_simple.xml`) which contains no special scenarios.
- **Command Override**:
  - Runs the agent with simplified settings suitable for development and testing.
- **Purpose**: Provides a minimal setup for development without special scenarios.

### `docker-compose_cicd.yaml`

- **Includes**:
  - `roscore_service.yaml`
  - `carla-simulator_service.yaml`
- **Services**:
  - Defines an `agent` service using a prebuilt image from the project's container registry.
- **Dependencies**:
  - Depends on `carla-simulator` and `roscore` to ensure they start before the agent.
- **Purpose**: Runs the leaderboard evaluator as part of Continuous Integration/Continuous Deployment (CI/CD) pipelines.

## Execution Modes

The project supports two primary execution modes, suitable for different development and deployment scenarios.

### Normal Execution

In normal execution mode, all services (agent, CARLA simulator, ROS core) run on a single machine. This mode is suitable for:

- Development and testing with smaller models.
- Scenarios where the machine's resources (especially VRAM) are sufficient to handle both the agent's computation and the simulator.

### Distributed Execution

Distributed execution separates the agent and the CARLA simulator onto different machines. This is necessary when:

- Running large vision models that require extensive VRAM.
- The single machine's resources are insufficient to handle both the agent and simulator.

**Note**: In distributed execution, the CARLA simulator must be running on a second desktop PC, and the `CARLA_SIM_HOST` environment variable should be set accordingly.

## Usage

To run the project using the provided Docker Compose files:

- **Standard Execution with Special Scenarios**:

  ```bash
  docker-compose -f build/docker-compose.yaml up
  ```

- **Development Execution without Special Scenarios**:

  ```bash
  docker-compose -f build/docker-compose_dev.yaml up
  ```

- **CI/CD Execution**:

  The `docker-compose_cicd.yaml` is intended to be used within CI/CD pipelines and may be invoked as part of automated scripts.

## Notes

- Ensure that you have NVIDIA GPU support configured if running models that require GPU acceleration.
- The `agent_service.yaml` and other base service files are crucial for defining the common configurations and should not be modified unless necessary.
- When running in distributed mode, update the `CARLA_SIM_HOST` environment variable in the appropriate service configurations to point to the simulator's IP address.
- The linter services defined in `linter_services.yaml` can be used to maintain code quality and should be run regularly during development.

## Conclusion

This documentation should help you understand the build structure and how to
work with the different Docker configurations provided in the project.
Whether you are developing locally, running complex simulations,
or integrating with CI/CD pipelines, these configurations are designed to facilitate a smooth workflow.
