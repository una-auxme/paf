# Leaderboard summary

**Summary:** This document depicts general informations regarding the leaderboard as a quick overview in a more condensed form.

 - [General Information](#general-information)
 - [Traffic Scenarios](#traffic-scenarios)
   - [Generic](#generic)
   - [Control Loss](#control-loss)
   - [Traffic negotiation](#traffic-negotiation)
   - [Highway](#highway)
   - [Obstacle avoidance](#obstacle-avoidance)
   - [Braking and lane changing](#braking-and-lane-changing)
   - [Parking](#parking)
 - [Available Sensors](#available-sensors)


## General Information

The CARLA Leaderboard includes a variety of scenarios to test autonomous driving models in realistic urban environments. This document provides a quick overview of all possible scenarios and available sensors.
The leaderboard offers a driving score metric based on infractions happening during scenarios. 

[#366](https://github.com/una-auxme/paf/issues/366) provides a more indepth look at how this score is calculated.

Each time an infraction takes place several details are recorded and appended to a list specific to the infraction type in question. Besides scenario and score handling, the leaderboard also provides a plethora of sensors to get data from.

The leaderboard provides predefined routes with a starting and destination point. The route description can contain GPS style coordinates, map coordinates or route instructions.

Two participation modalities are offered, "Sensors" and "Map". "Sensors" only offers the sensors listed down below while "Map" offers HD map data in addition to all the sensors.

## Traffic Scenarios

  - ### **Generic:**
    - Traffic lights
    - Signs (stop, speed limit, yield)
  - ### **Control Loss:**
    - The ego-vehicle loses control due to bad conditions on the road and it must recover, coming back to its original lane.
      ![image](https://leaderboard.carla.org/assets/images/TR01.png)
  - ### **Traffic negotiation:**
    - The ego-vehicle is performing an unprotected left turn at an intersection, yielding to oncoming traffic.
      ![image](https://leaderboard.carla.org/assets/images/TR08.png)
    - The ego-vehicle is performing a right turn at an intersection, yielding to crossing traffic.
      ![image](https://leaderboard.carla.org/assets/images/TR09.png)
    - The ego-vehicle needs to negotiate with other vehicles to cross an unsignalized intersection.
      ![image](https://leaderboard.carla.org/assets/images/TR10.png)
    - The ego-vehicle is going straight at an intersection but a crossing vehicle runs a red light, forcing the ego-vehicle to avoid the collision.
      ![image](https://leaderboard.carla.org/assets/images/TR07.png)
    - The ego-vehicle needs to perform a turn at an intersection yielding to bicycles crossing from either the left or right.
      ![image](https://leaderboard.carla.org/assets/images/TR13.png)
  - ### **Highway:**
    - The ego-vehicle merges into moving highway traffic from a highway on-ramp.
      ![image](https://leaderboard.carla.org/assets/images/TR18.png)
    - The ego-vehicle encounters a vehicle merging into its lane from a highway on-ramp.
      ![image](https://leaderboard.carla.org/assets/images/TR19.png)
    - The ego-vehicle encounters a vehicle cutting into its lane from a lane of static traffic.
      ![image](https://leaderboard.carla.org/assets/images/TR20.png)
    - The ego-vehicle must cross a lane of moving traffic to exit the highway at an off-ramp.
      ![image](https://leaderboard.carla.org/assets/images/TR21.png)
    - The ego-vehicle is approached by an emergency vehicle coming from behind.
      ![image](https://leaderboard.carla.org/assets/images/TR23.png)
  - ### **Obstacle avoidance:**
    - The ego-vehicle encounters an obstacle blocking the lane and must perform a lane change into traffic moving in the same or opposite direction to avoid it. The obstacle may be a construction site, an accident or a parked vehicle.
      ![image](https://leaderboard.carla.org/assets/images/TR14.png)
    - The ego-vehicle encounters a parked vehicle opening a door into its lane and must maneuver to avoid it.
      ![image](https://leaderboard.carla.org/assets/images/TR15.png)
    - The ego-vehicle encounters a slow moving hazard (e.g. bicycles) blocking part of the lane. The ego-vehicle must brake or maneuver next to a lane of traffic moving in the same or opposite direction to avoid it.
      ![image](https://leaderboard.carla.org/assets/images/TR16.png)
    - The ego-vehicle encounters an oncoming vehicles invading its lane on a bend due to an obstacle.
      ![image](https://leaderboard.carla.org/assets/images/TR22.png)
  - ### **Braking and lane changing:**
    - The leading vehicle decelerates suddenly due to an obstacle and the ego-vehicle must perform an emergency brake or an avoidance maneuver.
      ![image](https://leaderboard.carla.org/assets/images/TR02.png)
    - The ego-vehicle encounters an obstacle / unexpected entity on the road and must perform an emergency brake or an avoidance maneuver.
      ![image](https://leaderboard.carla.org/assets/images/TR03.png)
    - The ego-vehicle encounters an pedestrian emerging from behind a parked vehicle and advancing into the lane.
      ![image](https://leaderboard.carla.org/assets/images/TR17.png)
    - While performing a maneuver, the ego-vehicle encounters an obstacle in the road, either a pedestrian or a bicycle, and must perform an emergency brake or an avoidance maneuver.
      ![image](https://leaderboard.carla.org/assets/images/TR04.png)
    - While performing a maneuver, the ego-vehicle encounters a stopped vehicle in the road and must perform an emergency brake or an avoidance maneuver.
      ![image](https://leaderboard.carla.org/assets/images/TR19a.png)
    - The ego-vehicle must slow down or brake to allow a parked vehicle exiting a parallel parking bay to cut in front.
      ![image](https://leaderboard.carla.org/assets/images/TR12.png)
  - ### **Parking:**
    - The ego-vehicle must exit a parallel parking bay into a flow of traffic.
      ![image](https://leaderboard.carla.org/assets/images/TR11.png)

## Available Sensors

  - **[GNSS](https://carla.readthedocs.io/en/latest/ref_sensors/#gnss-sensor)**
    - GPS sensor returning geo location data.
  - **[IMU](https://carla.readthedocs.io/en/latest/ref_sensors/#imu-sensor)**
    - 6-axis inertial measurement unit.
  - **[LIDAR](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor)**
    - Laser to detect obstacles.
  - **[RADAR](https://carla.readthedocs.io/en/latest/ref_sensors/#radar-sensor)**
    - Long-range RADAR (up to 100 meters).
  - **[RGB CAMERA](https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera)**
    - Regular camera for image capture
  - **[COLLISION DETECTOR](https://carla.readthedocs.io/en/latest/ref_sensors/#collision-detector)**
    - Used to detect collisions with other actors.
  - **[DEPTH CAMERA](https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera)**
    - Provides a distance-coded picture to create a depth map of the scene.
  - **[LANE INVASION DETECTOR](https://carla.readthedocs.io/en/latest/ref_sensors/#lane-invasion-detector)**
    - Uses road data to detect when the vehicle crosses a lane marking.
  - **[OBSTACLE DETECTOR](https://carla.readthedocs.io/en/latest/ref_sensors/#obstacle-detector)**
    - Detects obstacles in front of the vehicle within a capsular shape. Works geometry based so requires obstacles to exist as geometry in the scene. (seems like a pure simulation sensor)
  - **[RSS SENSOR](https://carla.readthedocs.io/en/latest/ref_sensors/#rss-sensor)**
    - Integrates the responisbility sensitive safety model in CARLA. Basically a framwork for mathematical guidelines on how to react in various scenarios. Disabled by default.
  - **[SEMANTIC LIDAR](https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-lidar-sensor)**
    - Similiar to LIDAR with a different data structure/focus.
  - **[SEMANTIC SEGMENTATION CAMERA](https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera)**
    - Classifies objects in sight by tag (seems like a pure simulation sensor).
  - **[INSTANCE SEGMENTATION CAMERA](https://carla.readthedocs.io/en/latest/ref_sensors/#instance-segmentation-camera)**
    - Classifies every object by class and instance ID (seems like a pure simulation sensor).
  - **[DVS CAMERA](https://carla.readthedocs.io/en/latest/ref_sensors/#dvs-camera)**
    - Different kind of camera that works in high-speed scenarios. Pixels asynchronously respond to local changes in brightness instead of globally by shutter.
  - **[OPTICAL FLOW CAMERA](https://carla.readthedocs.io/en/latest/ref_sensors/#optical-flow-camera)**
    - Captures motion(velocity per pixel) perceived from the point of view of the camera.
  - **[V2X SENSOR](https://carla.readthedocs.io/en/latest/ref_sensors/#v2x-sensor)**
    - Vehicle to everything sensor, allows vehicle to communicate with other vehicles and elements in the environment. More like a concept for the future, not widespread implemented yet.
