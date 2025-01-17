# Traffic Rules and Principles in CARLA

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Key Traffic Principles and Complex Interactions](#key-traffic-principles-and-complex-interactions)
- [Using the Traffic Manager in CARLA](#using-the-traffic-manager-in-carla)
- [CARLA Maps and Their Traffic Scenarios](#carla-maps-and-their-traffic-scenarios)
  - [Overview of CARLA Maps](#overview-of-carla-maps)
  - [Rural and Highway Scenarios](#rural-and-highway-scenarios)
- [Road Network Creation with ASAM OpenDRIVE®](#road-network-creation-with-asam-opendrive)
- [Units of Measurement](#units-of-measurement)
- [References](#references)

## Introduction

CARLA (Car Learning to Act) is an open-source simulator for testing and developing autonomous driving systems. It integrates traffic rules to create a flexible environment, letting users adjust regulations and design custom maps to evaluate driving behaviors without following specific national laws.

## Key Traffic Principles and Complex Interactions

- **Traffic Signals**: Vehicles must stop at red lights and go on green.

- **Stop Signs**: Vehicles must stop at stop signs and yield to pedestrians
  and other cars.

- **Lane Management**: Cars should remain in their designated lanes.

- **Speed Limits**: Different speed limits in various areas of the map allow
  testing of speed control and adherence to traffic laws.

- **Pedestrian Rights**: Vehicles must yield to pedestrians at crosswalks,
  which is vital for pedestrian detection and ensuring safe stops.

- **Intersection Management**: Vehicles must navigate intersections
  effectively, managing stop lights and stop signs while yielding when
  necessary to keep traffic moving.
  *Note: According to the [CARLA documentation](https://carla.readthedocs.io/en/latest/adv_traffic_manager/) the Traffic Manager uses its own priority system at junctions. This is a known problem in the simulation.*

- **Roundabout Navigation**: Vehicles should yield to cars already in the
  roundabout. This tests their ability to navigate circular roads and merge
  correctly.

- **Crosswalk Behavior**: Vehicles need to detect and stop for pedestrians
  at crosswalks, highlighting the importance of pedestrian safety.

- **Bicycle Lane Navigation**: Vehicles should be able to detect and safely
  navigate around cyclists in traffic.

- **Parking Maneuvers**: Vehicles are tested on their ability to park in
  tight spaces, simulating real-life parking challenges.

## Using the Traffic Manager in CARLA

The Traffic Manager (TM) in CARLA controls vehicle behavior during simulations. Below are key considerations and customization options:

### General Behavior Patterns

- **Non-goal-oriented Navigation**: Vehicles follow dynamically generated trajectories, choosing paths randomly at junctions. Their navigation is endless.
- **Speed Adjustment**: Vehicles maintain a target speed of 70% of the speed limit unless explicitly modified.
- **Junction Priority**: Junction handling does not strictly follow traffic regulations. Instead, the TM applies its own priority system, leading to potential issues, such as vehicles inside a roundabout yielding to incoming traffic. This limitation is being addressed.

### Customizing TM Behavior via Python API

The TM behavior can be tailored using the Python API. Key functionalities include:

#### General

- Create a TM instance connected to a specific port.
- Retrieve the port where the TM is connected.

#### Safety Conditions

- Set a minimum stopping distance for vehicles (globally or individually).
- Adjust the desired speed as a percentage of the speed limit.
- Reset traffic lights programmatically.

#### Collision Management

- Enable or disable collisions between vehicles and specific actors.
- Configure vehicles to ignore:
  - Other vehicles
  - Pedestrians
  - Traffic lights

#### Lane Management

- Force lane changes, overriding collision considerations.
- Enable or disable lane changes for specific vehicles.

#### Hybrid Physics Mode

- Enable or disable hybrid physics mode for improved performance.
- Adjust the radius within which physics is applied.

For more details, refer to the [TM section of the Python API documentation](https://carla.readthedocs.io/en/latest/adv_traffic_manager/#vehicle-behavior-considerations).

## CARLA Maps and Their Traffic Scenarios

CARLA has 12 maps, each designed to test different aspects of self-driving,
from basic navigation to complex urban and rural situations.
Users can also create custom maps, allowing for additional testing scenarios
tailored to specific needs.

### Overview of CARLA Maps

- **Town 1**: A small town with T-junctions and bridges, focusing on managing
  intersections and staying in lanes.

- **Town 2**: Similar to Town 1, this town includes shops and homes, testing
  navigation and awareness of the surroundings.

- **Town 3**: A larger city area with roundabouts and underpasses, assessing
  navigation in roundabouts and speed adjustments.

- **Town 4**: A small town with a figure-eight road, testing navigation in
  multiple lanes and interactions with pedestrians.

- **Town 5**: An urban area with multilane roads and a highway, focusing on
  highway driving and commercial strategies.

- **Town 6**: A less populated town with unique junctions, testing navigation
  in multiple lanes and the use of slip roads.

- **Town 7**: A rural community with no marked roads, which tests navigation
  and parking in residential areas.

- **Town 8 & Town 9**: Hidden maps used for the Leaderboard challenge.

- **Town 10**: An inner-city area with various junctions and pedestrian
  activity, testing skills in complex traffic situations.

- **Town 11**: A basic map for testing navigation in open areas with fewer
  visual guides.

- **Town 12**: A large map based on Amarillo, Texas, featuring urban,
  residential, and rural areas to test navigation and visual skills across
  diverse environments.

### Rural and Highway Scenarios

Maps like **Town 7** and **Town 12** focus on rural and highway driving,
simulating open and high-speed areas. They test long-distance driving,
speed management, and safe overtaking, requiring drivers to adapt to
unmarked roads and agricultural settings.

## Road Network Creation with ASAM OpenDRIVE®

Road networks in CARLA are built using **ASAM OpenDRIVE®**, which standardizes
road descriptions:

- **Hierarchical Structure**: Roads are organized into nodes for better
  integration.

- **Reference Line**: Each road has a reference line that defines its shape.

- **Interconnectivity**: Roads can connect to each other for realistic
  traffic flow.

## Units of Measurement

To keep tests consistent, CARLA uses standardized units:

- **Speed**: Measured in meters per second (m/s).

- **Distance**: Measured in meters.

## References

- CARLA Documentation: [CARLA](https://carla.readthedocs.io/en/latest/)
- Traffic Manager Behavior Considerations: [CARLA TM Documentation](https://carla.readthedocs.io/en/latest/adv_traffic_manager/#vehicle-behavior-considerations)
- ASAM OpenDRIVE® Standard: [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
