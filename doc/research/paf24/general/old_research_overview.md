# Research Summary

**Summary:** The research of the previous groups is condensed into this file to make it an entry point for this years project.

- [**Research and Resources**](#research-and-resources)
- [**Acting and Control Modules**](#acting-and-control-modules)
- [**Planning and Trajectory Generation**](#planning-and-trajectory-generation)
- [**State Machine for Decision-Making**](#state-machine-for-decision-making)
- [**OpenDrive Integration and Navigation Data**](#opendrive-integration-and-navigation-data)

## **[Research and Resources](./README.md)**

- This section provides an extensive foundation for the autonomous vehicle project by consolidating previous research from **[PAF22](./paf22/)** and **[PAF23](./paf23/)**.  
- **PAF22**: Established core methods for autonomous vehicle control and perception, including traffic light detection and emergency braking features. It also set up the base components of the CARLA simulator integration, essential sensor configurations, and data processing pipelines.
- **PAF23**: Enhanced lane-change algorithms and expanded intersection-handling strategies. This project introduced a more robust approach to decision-making at intersections, factoring in pedestrian presence, oncoming traffic, and improved signal detection. Additionally, **PAF23** refined vehicle
- behavior for more fluid lane-change maneuvers, optimizing control responses to avoid obstacles and maintain lane positioning during highway merging and overtaking scenarios.
- The resources also include CARLA-specific tools such as the CARLA Leaderboard and ROS Bridge integration, which link CARLA’s simulation environment to the Robot Operating System (ROS). Detailed references to CARLA’s sensor suite are provided, covering RGB cameras, LIDAR, radar, GNSS, and IMU
sensors essential for perception and control.

## **[Acting and Control Modules](./paf22/acting/implementation_acting.md)**

- The **acting module** focuses on the vehicle’s control actions, including throttle, steering, and braking.
- **Core Controllers**: Contains controllers like the **PID controller** for longitudinal (speed) control and **Pure Pursuit** and **Stanley controllers** for lateral (steering) control. These controllers work in unison to achieve precise vehicle handling, especially on turns and at varying speeds.
- **PAF22 Contributions**: Implemented the PID-based longitudinal controller and integrated the Pure Pursuit controller for basic trajectory following, setting a solid foundation for steering and throttle control. PAF22 also laid out the preliminary **emergency braking** logic, designed to override
other controls in hazardous situations.
- **PAF23 Contributions**: Introduced refinements in lateral control, including the adaptive Stanley controller, which adjusts steering sensitivity based on vehicle speed to maintain a smooth trajectory. **PAF23** also optimized the emergency braking logic to respond more quickly to obstacles, with
improvements in lane-changing safety.
- **Sensor Integration**: The acting module subscribes to **navigation and sensor data** topics to remain updated on vehicle position and velocity, integrating sensor feedback for real-time control adjustments.

## **[Planning and Trajectory Generation](./paf22/planning/basics.md)**

- The **planning module** is critical for determining safe, efficient routes by combining **global and local path planning** techniques.
- **Global Planning**: Uses the **CommonRoad route planner** from TUM, creating a high-level path based on predefined waypoints. **PAF22** initially set up this planner, while **PAF23** added finer adjustments for lane selection and obstacle navigation.
- **Local Planning**: Tailored for dynamic obstacles, this component focuses on immediate adjustments to the vehicle’s path, particularly useful in urban environments with unpredictable elements.
Local planning includes **trajectory tracking** using Pure Pursuit and Stanley controllers to maintain a steady path.
- **PAF23 Enhancements**: Improved **collision avoidance** algorithms and added real-time updates to the trajectory based on sensor data. The local planner now adapts quickly to lane-change requests or route deviations due to traffic, creating a seamless flow between global and local path planning.

## **[State Machine for Decision-Making](./paf22/planning/state_machine_design.md)**

- This modular state machine handles various driving behaviors, including **lane changes**, **intersections**, and **traffic light responses**.
- **Core State Machines**: The **driving state machine** manages normal vehicle navigation, controlling target speed and ensuring lane compliance. The **lane-change state machine** makes safe decisions based on lane availability and traffic,
while the **intersection state machine** manages vehicle approach, stop, and turn behaviors at intersections.

- **PAF22 Contributions**: Developed the base decision-making states, enabling lane following, simple lane-change maneuvers, and basic intersection handling.
- **PAF23 Contributions**: Significantly enhanced the state machine by adding specialized states for complex maneuvers, like responding to oncoming traffic at intersections, merging onto highways, and making priority-based decisions at roundabouts.
The **intersection state machine** now incorporates detailed behaviors for handling left turns, straight passes, and right turns, considering pedestrian zones and cross-traffic. **PAF23** also introduced an adaptive lane-change state,
which calculates safety based on vehicle speed, distance to adjacent vehicles, and road type.

## **[OpenDrive Integration and Navigation Data](./paf22/planning/OpenDrive.md)**

- **OpenDrive** files provide a structured road network description, detailing lanes, road segments, intersections, and traffic signals. The navigation data is published in CARLA as ROS topics containing GPS/world coordinates and route instructions.
- **PAF22 Setup**: Established OpenDrive as the core format for map data, integrating it with the CARLA simulator. Initial work involved parsing road and lane data to create accurate trajectories.
- **PAF23 Enhancements**: Improved the parsing of OpenDrive files, focusing on high-level map data relevant to the vehicle's route, such as signal placements and lane restrictions. This project also optimized the navigation data integration with ROS,
ensuring the vehicle receives consistent updates on its location relative to the route, intersections, and nearby obstacles.
- **Navigation Data Structure**: The system uses navigation data points, including **GPS coordinates, world coordinates, and high-level route instructions** (e.g., turn left, change lanes) to guide the vehicle. Each point is matched with a **road option command**,
instructing the vehicle on how to proceed at specific waypoints.
