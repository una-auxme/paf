# Title of wiki page

**Summary:** This page contains possible improvements to the **docker/build system** of the project

- [Use a venv for python dependencies](#use-a-venv-for-python-dependencies)
- [Use a singular Dockerfile with multiple build stages instead of the include](#use-a-singular-dockerfile-with-multiple-build-stages-instead-of-the-include)
- [User permission handling in the Dockerfile](#user-permission-handling-in-the-dockerfile)
  - [Options](#options)
    - [Remove user-specific instructions from the Dockerfile](#remove-user-specific-instructions-from-the-dockerfile)
    - [Use a non-root user in the Dockerfile](#use-a-non-root-user-in-the-dockerfile)
    - [Use rootless docker and leave the root user in the Dockerfile](#use-rootless-docker-and-leave-the-root-user-in-the-dockerfile)
    - [Use the devcontainer-cli](#use-the-devcontainer-cli)
    - [Use podman instead of docker with userns-remap=keep-id](#use-podman-instead-of-docker-with-userns-remapkeep-id)
- [Retain user configurations across container recreations](#retain-user-configurations-across-container-recreations)
- [Make it possible to keep Vs Code attached to the leaderboard container](#make-it-possible-to-keep-vs-code-attached-to-the-leaderboard-container)
- [Make it possible to restart the leaderboard independently of Carla](#make-it-possible-to-restart-the-leaderboard-independently-of-carla)
- [Properly wait for container startup (Carla)](#properly-wait-for-container-startup-carla)
- [Enable caching for pip and apt in the Dockerfile](#enable-caching-for-pip-and-apt-in-the-dockerfile)
- [Add support for AMD GPUs](#add-support-for-amd-gpus)

More ideas and experiments can be found in [[Feature]: Development container refactor #545](https://github.com/una-auxme/paf/issues/545)

## Use a venv for python dependencies

A venv can be used to decouple the python dependencies from the fixed ubuntu versions.

In ROS1, to get a working setup, the venv still has to include the system packages, limiting its usefulness.
The ROS2 behavior is currently unknown to me.

Main benefit even with the system packages: Allows updating pip and other system packages.

Problems: None really. The venv always has to be sourced, but the Dockerfile does it automatically for the developer.

Effort estimate: Very low. Initializing the venv is simple and sourcing can be done together with the sourcing of the ROS environment.

Recommendation: Use a venv.  
Different package managers can be used for managing the venv.  
**More considerations about the python dependency management in ROS can be found [here](../ros2_porting/python_porting.md#version-management-options)**

## Use a singular Dockerfile with multiple build stages instead of the include

Reduced build time and does not require custom syntax (just buildkit, now enabled by default in docker)

## User permission handling in the Dockerfile

### Options

#### Remove user-specific instructions from the Dockerfile

Put the user switch into the entrypoint instead

#### Use a non-root user in the Dockerfile

This is the current setup

#### Use rootless docker and leave the root user in the Dockerfile

#### Use the devcontainer-cli

#### Use podman instead of docker with userns-remap=keep-id

## Retain user configurations across container recreations

Mainly keep the rqt config

## Make it possible to keep Vs Code attached to the leaderboard container

## Make it possible to restart the leaderboard independently of Carla

Will reduce simulation startup time

## Properly wait for container startup (Carla)

Will reduce simulation startup time by not waiting an arbitrary amount of time

## Enable caching for pip and apt in the Dockerfile

## Add support for AMD GPUs

Already proven to work, will just use a different base docker image and compose config
