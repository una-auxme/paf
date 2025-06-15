# General improvements

**Summary:** The page contains possible improvements to (the infrastructure of) the project

- [**Docker improvements**](#docker-improvements)
- [Integration of unit-tests into the CI](#integration-of-unit-tests-into-the-ci)
- [Add performance benchmarks](#add-performance-benchmarks)
  - [Approaches (benchmarks)](#approaches-benchmarks)
    - [Measure project startup time](#measure-project-startup-time)
    - [Measure docker build times](#measure-docker-build-times)
    - [Measure callback performance manually inside the nodes](#measure-callback-performance-manually-inside-the-nodes)
    - [Measure topic publish rates](#measure-topic-publish-rates)
    - [ros2\_benchmark for benchmarking parts of the node graph](#ros2_benchmark-for-benchmarking-parts-of-the-node-graph)
    - [ros2\_tracing for tracing the whole project](#ros2_tracing-for-tracing-the-whole-project)
  - [Status (benchmarks)](#status-benchmarks)
- [Create simple script for restarting all PAF nodes without restarting the leaderboard](#create-simple-script-for-restarting-all-paf-nodes-without-restarting-the-leaderboard)
- [Implement Correct Synchronization Mechanism for CARLA Simulation Steps](#implement-correct-synchronization-mechanism-for-carla-simulation-steps)
- [Add more cython annotations in the mapping\_common package](#add-more-cython-annotations-in-the-mapping_common-package)
- [Fix Carla crashes](#fix-carla-crashes)
  - [Solutions (Carla)](#solutions-carla)
  - [Status (Carla)](#status-carla)
- [Use official leaderboard release](#use-official-leaderboard-release)

Priorities: LOW=NiceToHave, MED=VeryNiceToHave, HIGH=MustHave  
Efforts: LOW=CanBeDoneOnTheSide, MED=DayOfWork, HIGH=Week(s)ofWork

## **Docker improvements**

**Docker specific improvements [here](./docker.md)**

## Integration of unit-tests into the CI

Issue: [Improve the actions and their test coverage #385](https://github.com/una-auxme/paf/issues/385)

Priority: MED
Effort: MED-HIGH

TODO

## Add performance benchmarks

### Approaches (benchmarks)

#### Measure project startup time

Check which improvements/changes to the infrastructure affect startup performance.
Startup performance is very important for the development iteration speed and increases developer efficiency.

A simple approach might be to measure the time between container startup (Start of bash command) and the response of certain nodes of the system.

For the response, a similar approach to the debug_logger with `multiprocessing.connection` might be used, because it is independent of ROS and Carla.

Benefits:

- A lot of proposed changes to the docker setup will mainly affect startup performance.
- Easy to implement now and then port to ROS2 -> comparable
- Can easily write a file with values for ci/cd as part of the existing drive action

Problems:

- Only able to measure startup
- Measurements are probably subject to high fluctuations

Effort: MED
Priority: MED? Should be done before the main ROS2 porting effort.

Status: Planned

#### Measure docker build times

TODO

Status: Planned

#### Measure callback performance manually inside the nodes

Maybe a custom class for ROS2?

Status: Not planned

#### Measure topic publish rates

ROS1: <https://wiki.ros.org/Topics#Topic_statistics>
ROS2: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Topic-Statistics.html>

Pros:

- Easy to activate for ROS1: Global parameter.
- Available in ROS2

Cons:

- ROS2:
  - Seems to be kinda unfinished: <https://github.com/ros2/ros2/issues/917>
  - Has to be manually enabled for each subscriber: <https://github.com/ros-tooling/roadmap/issues/1>
- Only works for topics (No services, actions)
- Data only shows if a topic is too slow or fast enough for the simulator speed. It does not provide useful timing data if a topic reaches the simulator update rate.

Status: Prefer [manually measuring the nodes](#measure-callback-performance-manually-inside-the-nodes) instead

#### [ros2_benchmark](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark) for benchmarking parts of the node graph

Status: Not planned

#### [ros2_tracing](https://github.com/ros2/ros2_tracing) for tracing the whole project

Monitor tracepoints across the whole project, ROS and kernel. This makes it easy to measure callback execution times without performance overhead.

Talk: <https://www.youtube.com/watch?v=PEBJU7bFf-o>

Pro:

- Very easy to integrate
- Can maybe even provide statistics in ci

Con:

- Unknown how well this integrates with python nodes. Should still be able to measure how long callbacks take.
- Not directly available for ROS1, but the talk mentions that they had an implementation for ROS1
- Does not measure project startup times
- Probably overkill for our needs

Recommendation: Move to future research?

Status: Not planned

### Status (benchmarks)

General problems:

- Benchmarks are highly dependent on the hardware they run on and the hardware is not standardized across the project. But at least the ci runner is always the same.
- Benchmarks are highly dependent on background loads that run alongside the project.

Recommendation: Implement before main porting effort:

- [Measure project startup time](#measure-project-startup-time)
- [Measure docker build times](#measure-docker-build-times)
- ~~[Measure callback performance manually inside the nodes](#measure-callback-performance-manually-inside-the-nodes)~~

## Create simple script for restarting all PAF nodes without restarting the leaderboard

Would allow rapid development and does not run into the Carla memory leak

Restart should exclude some nodes like the GlobalPlanner

## Implement Correct Synchronization Mechanism for CARLA Simulation Steps

Issue: [[Feature]: Implement Correct Synchronization Mechanism for CARLA Simulation Steps #701](https://github.com/una-auxme/paf/issues/701)

There might be a way to properly control the data-flow inside the projekt -> possible removal of the loop_sleep_time

Benefits:

- Reduced dead-time in simulation (less waiting time between steps)
- The simulation speed dynamically adjusts to the hosts CPU speed.
  - Currently, a host faster than loop_sleep_time will have dead-time
  - A host slower than loop_sleep_time will not leave enough time for computations inside the nodes. Datasets then start lagging (e.g. the behavior tree will use an outdated map for its decisions)

Priority: Low. As long as the loop_sleep_time is set high enough for the given host-CPU, **the only real benefit is a faster simulation speed**

Status: Discuss steps once the ROS2 port is done

## Add more cython annotations in the mapping_common package

Benefits: performance improvements of unknown quantity. The current performance improvements from cython are unknown -> benefit of further annotations is also unknown.

An investigation including before/after benchmarks might make sense since the intermediate layer seems to be a major performance bottleneck for the system.  
➡ Soft Dependency on [Add performance benchmarks](#add-performance-benchmarks)

Priority: Low, since the project works without it and improvements mainly lead to faster evaluation.

Effort estimate: Low. Modified function and class definitions have to be copied into the .pxd files.

## Fix Carla crashes

The U4 Linux version of the Carla leaderboard always crashes after some time (vulkan memory leak?).

This is a problem when:

- running longer evaluations.
- Leaving carla running independently of the leaderboard

There is a fix, but no new leaderboard-specific version was released:

- <https://github.com/carla-simulator/carla/issues/7156>
- <https://github.com/carla-simulator/carla/issues/8703>

### Solutions (Carla)

- There are nightly builds available: <https://github.com/carla-simulator/carla/tree/ue4-dev?tab=readme-ov-file#download-carla>
  - It is unknown if they are up-to-date and stable enough for usage
- It might be possible to compile the ue4-dev branch from source... but this requires a lot of effort

### Status (Carla)

Benefits of fixing: Mainly for the long-running evaluations. This means that this issue does NOT majorly affect the project.

Priority: Very low, not planned. Because the benefits are low and fixing it might involve a lot of work and testing.

Recommendation: Wait/Hope for a new release. An issue including this information will be created.

## Use official leaderboard release

TODO
