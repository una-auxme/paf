# Traffic Rules in CARLA

## Introduction
CARLA (Car Learning to Act) is an open-source simulator for developing and testing autonomous driving systems. There is no strict set of traffic laws implemented, but key traffic behaviors are emphasized to create a realistic urban driving environment.

## Key Traffic Principles in CARLA
- **Traffic Signals**: Vehicles must obey traffic lights—stop at red and go at green.
- **Stop Signs**: Cars must stop at stop signs and yield to pedestrians and other vehicles.
- **Lane Management**: Vehicles should stay in their lanes.
- **Speed Limits**: There are speed limits in different areas of the simulation.
- **Pedestrian Rights**: Vehicles must yield to pedestrians at crosswalks.

These principles help in developing algorithms for autonomous vehicles by simulating real-world traffic rules.

## Focus on Urban Environments
CARLA focuses on urban scenarios with interactions among vehicles, pedestrians, and cyclists. Examples include:
- **Intersections**: Handling traffic lights and stop signs.
- **Roundabouts**: Yielding to cars already in the circle.
- **Crosswalks**: Stopping for pedestrians.
- **Bike Lanes**: Watching out for cyclists.
- **Parking**: Maneuvering in tight spaces.

These scenarios are essential for training self-driving systems in complex environments.

## Road Network Creation with ASAM OpenDRIVE®
Road networks in CARLA are created using **ASAM OpenDRIVE®**, which standardizes road descriptions:
- **Hierarchical Structure**: Roads are organized in nodes for better application integration.
- **Reference Line**: Each road has a reference line to define its shape.
- **Interconnectivity**: Roads can connect for realistic traffic flow.

## Units of Measurement
In CARLA, speed is measured in meters per second (m/s), and distances are in meters, ensuring consistency in simulations.

## Flexibility and Customization
CARLA allows users to modify traffic rules and create custom maps. This flexibility enables researchers to design specific scenarios, enhancing the testing of self-driving technologies.

## References
- CARLA Documentation: [CARLA](https://carla.readthedocs.io/en/latest/)
- ASAM OpenDRIVE® Standard: [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
