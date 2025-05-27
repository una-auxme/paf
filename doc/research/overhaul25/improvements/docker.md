# Docker improvements

**Summary:** This page contains possible improvements to the **docker/build system** of the project

- [Use a venv for python dependencies](#use-a-venv-for-python-dependencies)
- [Move explicitly installed ROS dependencies into the package.xml and install them with rosdep](#move-explicitly-installed-ros-dependencies-into-the-packagexml-and-install-them-with-rosdep)
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

Priorities: LOW-NiceToHave, MED-VeryNiceToHave, HIGH-MustHave
Efforts: LOW-CanBeDoneOnTheSide, MED-DayOfWork, HIGH-Week(s)ofWork

More ideas and experiments can be found in [[Feature]: Development container refactor #545](https://github.com/una-auxme/paf/issues/545)

## Use a venv for python dependencies

A venv can be used to decouple the python dependencies from the fixed ubuntu versions.

In ROS1, to get a working setup, the venv still has to include the system packages, limiting its usefulness.  
The ROS2 behavior is currently unknown... (but the [documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment) does not require to enable system packages)

Main benefit even with the system packages: Allows updating pip and other system packages.

Problems: None really. The venv always has to be sourced, but the docker entrypoint does it automatically for the developer.

Effort estimate: LOW. Initializing the venv is simple and sourcing can be done together with the sourcing of the ROS environment.

Recommendation: Use a venv.  
Different package managers can be used for managing the venv.  
**More considerations about the python dependency management in ROS can be found [here](../ros2_porting/python_porting.md#dependency-management-options)**

## Move explicitly installed ROS dependencies into the package.xml and install them with rosdep

Pros:

- Reduced Dockerfile complexity
- Better documents which package requires which dependencies

Cons:

- Reliability of rosdep is unknown.

Priority: Low. The current setup works; easy to change later

Required effort: Low

Recommendation: Might be implemented when the package.xml files are migrated.

## Use a singular Dockerfile with multiple build stages instead of the include

Pros:

- Reduced build time when building both leaderboard and dev images
- Does not require custom syntax (just buildkit, now enabled by default in docker)
- When a target build stage is set, only the required stages are built

Cons:

- Dockerfile becomes quite long and consists out of multiple sections making it harder to read

Priority: HIGH, since it's an integral part of the build process

Required effort: LOW

Status: Implemented as part of the new ROS2 docker image.

## User permission handling in the Dockerfile

Priority: HIGH
Effort: MED

TODO: Optimal solution still needs to be figured out + documented

### Options

#### Remove user-specific instructions from the Dockerfile

Put the user switch into the entrypoint instead

#### Use a non-root user in the Dockerfile

This is the current setup

#### Use rootless docker and leave the root user in the Dockerfile

#### Use the devcontainer-cli

#### Use podman instead of docker with userns-remap=keep-id

## Retain user configurations across container recreations

TODO Mainly keep the rqt config

## Make it possible to keep Vs Code attached to the leaderboard container

Priority: MED
Effort: MED

TODO

## Make it possible to restart the leaderboard independently of Carla

Will reduce simulation startup time

Priority: Very LOW
Effort: MED

TODO

## Properly wait for container startup (Carla)

Will reduce simulation startup time by not waiting an arbitrary amount of time

Priority: LOW
Effort: LOW

TODO

## Enable caching for pip and apt in the Dockerfile

Reduces docker builds times dramatically, because download times become non-existent after the first build.

Priority: LOW
Effort: LOW

- The `--mount=type=cache` flag needs to be added to each RUN instruction that uses apt or pip.

Status: Implemented as part of the new ROS2 docker image.

## Add support for AMD GPUs

Pros:

- **Reduces GPU vendor lock-in**
- AMD users can work from home :-)

Cons:

- Unnecessary for the lab PCs
- Additional complexity when changing the Dockerfile and GPU dependencies down the line

Priority: LOW for the project, but HIGH for personal use :-)

Required effort: LOW

- The base image in the Dockerfile needs to be switchable between `nvidia/cuda` and `amd/rocm`
- The torch flavor needs to be adjustable

Status: Implemented as part of the new ROS2 docker image.
