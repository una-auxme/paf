# Global Planner

**Summary:** [global_planner_node.py](../../code/planning/src/global_planner/global_planner_node.py):
The global planner is responsible for collecting and preparing all data from the leaderboard and other internal
components that is needed for the preplanning component.
After finishing that this node initiates the calculation of a trajectory based on the OpenDriveConverter
from preplanning_trajectory.py. In the end the computed trajectory and prevailing speed limits are published.

This component and so most of the documentation was taken from the previous project PAF22 (Authors: Simon Erlbacher, Niklas Vogel)

- [Getting started](#getting-started)
- [Description](#description)
  - [Inputs](#inputs)
  - [Outputs](#outputs)
- [Testing](#testing)

## Getting started

Preplanning scripts are all integrated in this project and the used libraries are part of the docker file.
No extra installation needed.

---

## Description

First the global planner is responsible for collecting and preparing all data from the leaderboard and other internal components that is needed for the preplanning component.

To get an instance of the OpenDriveConverter (ODC) the received OpenDrive Map prevailing in String format
has to be converted. In our case we use the
[xml.etree.ElementTree](https://docs.python.org/3/library/xml.etree.elementtree.html) module which implements a
simple and efficient API for parsing XML data from string or file. After parsing, we can find out all the roads,
junctions and corresponding ids which are necessary to initialise the OpenDriveConverter. Afterwards the roads,
junctions and geometries can be converted and processed internally.

```python
        root = eTree.fromstring(opendrive.data)

        roads = root.findall("road")
        road_ids = [int(road.get("id")) for road in roads]
        junctions = root.findall("junction")
        junction_ids = [int(junction.get("id")) for junction in junctions]

        odc = OpenDriveConverter(
            roads=roads, road_ids=road_ids,
            junctions=junctions, junction_ids=junction_ids)

        odc.convert_roads()
        odc.convert_junctions()
        odc.filter_geometry()
```

The received agent spawn position is valid if it´s closer to the first waypoint then the `distance_spawn_to_first_wp`
parameter expresses. This is necessary to prevent unwanted behaviour in the startup phase where the
current agent position is faulty.

When the ODC is initialised, the current agent position is received and the global plan is obtained from
the leaderboard. The trajectory can be calculated by iterating through the global route and passing it to the ODC.
After smaller outliners are removed the x and y coordinates as well as the yaw-orientation and the prevailing
speed limits can be acquired from the ODC in the following form:

```python
        waypoints     = self.odc.remove_outliner(self.odc.waypoints)
        waypoints_x   = waypoints[0]
        waypoints_y   = waypoints[1]
        waypoints_yaw = waypoints[2]
        speed_limits  = waypoints[3]
```

The speed limits now can be made public right away.
To make the calculated waypoints available to other components it is formed into a
[Path]((http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)) message and published.

### Inputs

This node subscribes to the following needed topics:

- OpenDrive Map:
  - `/carla/{role_name}/OpenDRIVE` ([String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)) or `/carla/world_info` ([CarlaWorldInfo](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_msgs/#carlaworldinfomsg))
- global Plan:
  - `/carla/{role_name}/global_plan` ([CarlaRoute](https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaRoute.msg))
- current agent position:
  - `/paf/{role_name}/current_pos` ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

### Outputs

This node publishes the following topics:

- preplanned trajectory:
  - `/paf/{role_name}/trajectory` ([Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- prevailing speed limits:
  - `/paf/{role_name}/speed_limits_OpenDrive`
  ([Float32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32MultiArray.html))

## Testing

To generate a initial trajectory in the developement environment, you can use the [dev_global_route.py](../../code/planning/src/global_planner/dev_global_route.py) node. This node generates the initial trajectory on town 12.

Activate the node you need the do the following steps also described in [global_planner_node.py](../../code/planning/src/global_planner/global_planner_node.py):

- Uncomment the following lines in [planning.launch](../../code/planning/launch/planning.launch):

```xml
<node pkg="planning" type="dev_global_route.py" name="DevGlobalRoute" output="screen">
        <param name="from_txt" value="True" />
        <param name="sampling_resolution" value="75.0" />
        <param name="routes" value="/opt/leaderboard/data/routes_devtest.xml" />
        <param name="global_route_txt" value="/code/planning/src/global_planner/global_route.txt" />
        <param name="role_name" value="hero" />
</node>
```

- Uncomment the following line in the bottom of the `init` method in [global_planner_node.py](../../code/planning/src/global_planner/global_planner_node.py):

```python
self.dev_load_world_info()
```
