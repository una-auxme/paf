# Overhaul plan

**Summary:** This page outlines possible steps for the overhaul/rework of the project during SoSe-25. (Projektmodul Peter Viechter, Sommersemester 2025)

The overhaul research files *were* "living" documents that were updated alongside the overhaul. Last update: November 2025

- [ROS2 port](#ros2-port)
  - [Organization](#organization)
- [General improvements](#general-improvements)

## ROS2 port

**Status: ROS2 port merged with <https://github.com/una-auxme/paf/pull/832>**

The main goal of this overhaul is to port the project to ROS2, since the last ROS1 release (Noetic) is EOL in May 2025.

The **required steps, assessment of potential problems, etc...** can be found **[➡ here](./ros2_porting/README.md)**.

Information related to the porting of python based packages and nodes can be found [➡ here](./ros2_porting/python_porting.md).

### Organization

~~For the porting effort, a protected branch named `ros2-dev` has been created.
It serves as the main ros2 branch until the porting is completed
and the ros2 based implementation achieves similar performance/quality to the (old, ros1) main branch.
After that this branch *will be merged into* / *become the new* main.~~

Issues are opened based on the steps and problems outlined [➡ here](./ros2_porting/README.md#steps).  
Based on these issues, PRs are opened and merged onto the `ros2-dev` branch.
All these issues and PRs are marked with the ros2 label.

~~During the porting effort, the content of `ros2-dev` will only partially work.  
➡ Changes to the CI are required and the CI coverage needs to be gradually increased during the porting effort.
Planned CI adjustments [➡ here](./ros2_porting/README.md#ci-plan).~~

## General improvements

After the porting to ROS2 is (hopefully) successful, [➡ here](./improvements/README.md) are some general improvements that can be made to the project.  
This list includes assessments of what changes might make sense, their priority and how they could be implemented.

There is also a list of proposed improvements to the docker based build system [➡ here](./improvements/docker.md).  
Some of them will be implemented directly as part of the ROS2 porting effort.
