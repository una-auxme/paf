# Navigation Data Research

**Summary:** This page gives an overview and summary of how navigation data can be received, how it is structured and a visualisation of where the route instructions are placed on the ego vehicle route.

---

## Author

Niklas Vogel

## Date

14.12.2022

---
<!-- TOC -->
* [Navigation Data Research](#navigation-data-research)
  * [Author](#author)
  * [Date](#date)
  * [How to receive navigation data](#how-to-receive-navigation-data)
  * [Structure of navigation data](#structure-of-navigation-data)
  * [Visualisation of received navigation data](#visualisation-of-received-navigation-data)
* [Sources](#sources)
<!-- TOC -->

## How to receive navigation data

The navigation data (ego vehicle route) is published by carla alongside all other sensor data via ROS topics. The topic names are structured as follows: `/carla/hero/<sensor-id>`.

Sensor-id of all sensors can be set alongside the placement options in sensors method of [agent.py](../../../code/agent/src/agent/agent.py).

Example:

```python
sensors = [
            {'type': 'sensor.camera.rgb', 'id': 'Center',
             'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': 0.0, 'width': 300, 'height': 200, 'fov': 100},
            {'type': 'sensor.lidar.ray_cast', 'id': 'LIDAR',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0},
            {'type': 'sensor.other.radar', 'id': 'RADAR',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0, 'horizontal_fov': 30, 'vertical_fov': 30},
            {'type': 'sensor.other.gnss', 'id': 'GPS',
             'x': 0.7, 'y': -0.4, 'z': 1.60},
            {'type': 'sensor.other.imu', 'id': 'IMU',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0},
            {'type': 'sensor.opendrive_map', 'id': 'OpenDRIVE',
             'reading_frequency': 1},
            {'type': 'sensor.speedometer', 'id': 'Speed'},
        ]
 ```

Therefore, the Map is published as topic ``/carla/hero/OpenDrive`` in [OpenDRIVE](https://www.asam.net/standards/detail/opendrive/) format parsed as a string.
(Seems like the map is also available in the `/carla/world_info` topic)

The route is published in the following topics:

* ``/carla/hero/global_plan`` ([carla_msgs/CarlaRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaRoute.msg))
* ``/carla/hero/global_plan_gnss`` ([carla_msgs/CarlaGnnsRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaGnssRoute.msg))

## Structure of navigation data

Routes consist of tuples of a position and a high level route instruction command which should be taken at that point.
Positions are either given as GPS coordinates or as world coordinates:

* GPS coordinates:

```yaml
[({'z': 0.0, 'lat': 48.99822669411668, 'lon': 8.002271601998707}, RoadOption.LEFT),
 ({'z': 0.0, 'lat': 48.99822669411668, 'lon': 8.002709765148996}, RoadOption.RIGHT),
 ...
 ({'z': 0.0, 'lat': 48.99822679980298, 'lon': 8.002735250105061}, RoadOption.STRAIGHT)]
```

* World coordinates:

```yaml
[({'x': 153.7, 'y': 15.6, 'z': 0.0}, RoadOption.LEFT),
 ({'x': 148.9, 'y': 67.8, 'z': 0.0}, RoadOption.RIGHT),
 ...
 ({'x': 180.7, 'y': 45.1, 'z': 1.2}, RoadOption.STRAIGHT)]
```

* High-level route instruction commands (road options):

  * RoadOption.**CHANGELANELEFT**: Move one lane to the left.
  * RoadOption.**CHANGELANERIGHT**: Move one lane to the right.
  * RoadOption.**LANEFOLLOW**: Continue in the current lane.
  * RoadOption.**LEFT**: Turn left at the intersection.
  * RoadOption.**RIGHT**: Turn right at the intersection.
  * RoadOption.**STRAIGHT**: Keep straight at the intersection.

**Important:** Distance between route points can be up to hundreds of meters.
  
**Important:** If the semantics of left and right are ambiguous, the next position should be used to clarify the path.

## Visualisation of received navigation data

WIP

notes from team intern meeting:

* leaderboard evaluation visualisiert die route und scenarien evtl schon... evtl wert genauer zu betrachten

### Sources

<https://leaderboard.carla.org/get_started/>

<https://www.markdownguide.org/cheat-sheet/>

<https://carla.readthedocs.io/en/latest/core_map/#non-layered-maps>
