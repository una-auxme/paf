# Research

**Summary**: The research folder contains the research of each individual group at the start of the project.

- [1. Research Results of Previous Years](#1-research-results-of-previous-years)
- [2. Key Resources for Your Research](#2-key-resources-for-your-research)
  - [2.1. CARLA-Specific Resources](#21-carla-specific-resources)
  - [2.2. General Autonomous Driving Resources](#22-general-autonomous-driving-resources)

## 1. Research Results of Previous Years

The research is structured in folders for each year:

- [PAF22](./paf22/)
- [PAF23](./paf23/)
- [PAF24](./paf24/README.md)

## 2. Key Resources for Your Research

In addition to the research of the previous years, the following resources can be helpful.

### 2.1. CARLA-Specific Resources

- **Leaderboard Website**
  - [Getting Started](https://leaderboard.carla.org/get_started/)
  - [Leaderboard](https://leaderboard.carla.org/leaderboard/)
    - Review other submissions, as many teams publish papers or share code with their leaderboard entries.
  - [Global Route Understanding](https://leaderboard.carla.org/get_started/#22-understanding-the-leaderboard-components)
    - Understand how the global route is generated.
  - [Sensor Override Guide](https://leaderboard.carla.org/get_started/#33-override-the-sensors-method)
    - Learn about available sensors, their functions, advantages, and limitations.
- **Leaderboard Repository**
  - [Repository Link](https://github.com/carla-simulator/leaderboard/tree/leaderboard-2.0)
    - Ensure you use the `leaderboard-2.0` branch.
    - Currently, we are working on a **Fork**:
      - [Forked Repository](https://github.com/una-auxme/leaderboard/tree/leaderboard-2.0)
      - View our changes: [Commit Details](https://github.com/carla-simulator/leaderboard/commit/55a60afc4d60ec1e6766c8640106d377f788fbc5)
  - **Sensor Noise Configuration** can be accessed [here](https://github.com/una-auxme/leaderboard/blob/leaderboard-2.0/leaderboard/autoagents/agent_wrapper.py).
- **CARLA Documentation**
  - [Core Sensors](https://carla.readthedocs.io/en/latest/core_sensors/)
    - Learn about the general sensors in CARLA; note that not all sensors are available for the leaderboard.
- **Additional CARLA Resources**
  - [ROS Bridge](https://github.com/carla-simulator/ros-bridge)
    - Interface linking the leaderboard, CARLA Python API, and ROS nodes.
  - [Scenario Runner](https://github.com/carla-simulator/scenario_runner)
    - Defines general scenarios. Find test scenarios for the leaderboard [here](https://github.com/una-auxme/leaderboard/blob/leaderboard-2.0/data/routes_devtest.xml).
  - [Awesome CARLA Repository](https://github.com/Amin-Tgz/awesome-CARLA)

### 2.2. General Autonomous Driving Resources

- [Awesome Autonomous Vehicles](https://github.com/manfreddiaz/awesome-autonomous-vehicles)
- [Awesome 3D Object Detection for Autonomous Driving](https://github.com/PointsCoder/Awesome-3D-Object-Detection-for-Autonomous-Driving)
- [Microsoft Autonomous Driving Cookbook](https://github.com/microsoft/AutonomousDrivingCookbook)
- [End-to-End Autonomous Driving](https://github.com/OpenDriveLab/End-to-end-Autonomous-Driving)
- [UT-ADL/autoware_mini](https://github.com/UT-ADL/autoware_mini)
