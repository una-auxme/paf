# Traffic Rules and Principles in CARLA

## Table of Contents

- [Introduction](#introduction)
- [Key Traffic Principles and Complex Interactions](#key-traffic-principles-and-complex-interactions)
- [CARLA Maps and Their Traffic Scenarios](#carla-maps-and-their-traffic-scenarios)
  - [Overview of CARLA Maps](#overview-of-carla-maps)
  - [Rural and Highway Scenarios](#rural-and-highway-scenarios)
  - [Custom Maps with ASAM OpenDRIVE®](#custom-maps-with-asam-opendrive)
- [Units of Measurement](#units-of-measurement)
- [References](#references)

## Introduction

CARLA (Car Learning to Act) is an open-source simulator specifically designed for testing and developing autonomous driving systems. It incorporates essential traffic rules to create a flexible environment. This flexibility allows users to modify traffic regulations and design custom maps, enabling the evaluation of driving behaviors and interactions without strictly adhering to the traffic laws of any specific country.

## Key Traffic Principles and Complex Interactions

- **Traffic Signals**: Vehicles must stop at red lights and proceed on green, facilitating the testing of traffic light detection and timing adjustments.

- **Stop Signs**: Vehicles are required to halt at stop signs, yielding to pedestrians and other vehicles, which is essential for ensuring safety in traffic scenarios.

- **Lane Management**: Cars are expected to stay within designated lanes, aiding the development of lane-keeping and lane-following algorithms.

- **Speed Limits**: Varying speed limits across different map zones allow for the testing of speed regulation and adherence to traffic laws.

- **Pedestrian Rights**: Vehicles must yield to pedestrians at crosswalks, which is crucial for pedestrian detection and ensuring safe braking.

- **Intersection Management**: Vehicles must effectively navigate intersections, managing stop lights and stop signs while yielding as necessary to maintain traffic flow.

- **Roundabout Navigation**: Vehicles are required to yield to cars already within the roundabout, supporting testing of circular navigation and merging strategies.

- **Crosswalk Behavior**: Vehicles must detect and stop for pedestrians at crosswalks, highlighting the importance of pedestrian safety.

- **Bicycle Lane Navigation**: Vehicles should be able to detect and navigate around cyclists, ensuring safe interactions in mixed traffic environments.

- **Parking Maneuvers**: Vehicles are tested on their ability to maneuver in tight parking spaces, simulating real-life parking constraints.

## CARLA Maps and Their Traffic Scenarios

CARLA features 12 maps, each designed to test various aspects of autonomous driving, ranging from basic navigation to complex urban and rural settings.

### Overview of CARLA Maps

- **Town 1**: A small town with T-junctions and bridges, focusing on intersection management and lane discipline.

- **Town 2**: Similar to Town 1, it includes commercial and residential areas, testing navigation and environmental awareness.

- **Town 3**: A larger urban area with roundabouts and underpasses, assessing roundabout navigation and adaptive speed control.

- **Town 4**: A small town with a figure-eight ring road, testing multi-lane navigation and pedestrian interactions.

- **Town 5**: An urban environment featuring multilane roads and a highway, focusing on highway navigation and commercial driving strategies.

- **Town 6**: A low-density town with unique junctions, testing multi-lane navigation and slip road utilization.

- **Town 7**: A rural community with unmarked roads, challenging navigation and parking in residential areas.

- **Town 8 & Town 9**: Secret maps used for the Leaderboard challenge.

- **Town 10**: An inner-city area with diverse junctions and pedestrian activity, testing negotiation skills in complex traffic.

- **Town 11**: A minimally decorated map for testing navigation over expansive areas with fewer visual cues.

- **Town 12**: A large map inspired by Amarillo, Texas, with urban, residential, and rural areas, testing navigation and visual perception across diverse environments.

### Rural and Highway Scenarios

Maps like **Town 7** and **Town 12** emphasize rural and highway scenarios, simulating open and high-speed environments. They test long-distance driving, speed regulation, and safe overtaking, requiring adaptability to unmarked roads and agricultural structures in rural settings.

## Road Network Creation with ASAM OpenDRIVE®

Road networks in CARLA are created using **ASAM OpenDRIVE®**, which standardizes road descriptions:

- **Hierarchical Structure**: Roads are organized in nodes for better application integration.

- **Reference Line**: Each road has a reference line to define its shape.

- **Interconnectivity**: Roads can connect for realistic traffic flow.

## Units of Measurement

To ensure consistency across tests, CARLA employs standardized units:

- **Speed**: Measured in meters per second (m/s).

- **Distance**: Measured in meters.

## References

- CARLA Documentation: [CARLA](https://carla.readthedocs.io/en/latest/)

- ASAM OpenDRIVE® Standard: [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
