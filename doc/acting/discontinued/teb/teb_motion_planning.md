# TEB planner for motion planning

**Summary**: In PAF2024 the TEB local planner for ROS was tested because it was supposed to help for navigation in complex scenarios.
As a first example the TEB planner was integrated into the motion_planner node.
This documentation provides an overview on how the discontinued files worked, interacted and how they need to be launched.

- [The files](#the-files)
  - [motion\_planner.py](#motion_plannerpy)
  - [launch/motion\_planning.launch](#launchmotion_planninglaunch)
  - [Configuration files](#configuration-files)
    - [config/teb\_herocar\_config.yaml](#configteb_herocar_configyaml)
    - [config/motionplan\_config.yaml](#configmotionplan_configyaml)
    - [config/MotionPlan.cfg](#configmotionplancfg)

## The files

### [motion_planner.py](motion_planner.py)

The motion_planner.py in linked is based on an older version of [motion_planner.py](../../../../code/planning/src/local_planner/motion_planning.py) (in the code folder). Because several changes were made to it since TEB testing the file is not a drop-in replacement anymore.

The most important changes are the following:

- At first a ServiceProxy is created in order to be able to send plan requests to the teb planner.

    ```python
    self.plan_service: ServiceProxy = self.new_client(
                srv_type=Plan, srv_name="/teb_planner_node_pa/plan"
            )
    ```

- Shortly after this multiple parameters are configured with dynamic-reconfigure.
- The most important code is in the method `_generate_overtake_trajectory()` (line 351)
  - At first the start and goal poses are determined. These are `PoseStamped` objects. For the start_pose either the actual current pose or a pose behind the car can be used.
  - After this (from line 392) waypoint poses are generated from the original trajectory. These help the planner to stick to the original route.
  - Next from line 423 and onward obstacles from the map (intermediate layer) are converted to `ObstacleMsg` which is the format the TEB planner is working.
  - After this the Plan Service is called (with start_pose, goal_pose, waypoints and obstacles) set.
  - Finally, the received path is merged with the original trajectory.
- Apart from some helper functions no more relevant code for the TEB Planner is present.
  
### [launch/motion\_planning.launch](launch/motion_planning.launch)

For the TEB planner to work multiple Nodes have to be launched.

In the file at first the motion_planning node is launched, which as described above adds TEB-planned paths into the global trajectory.

The PA Node is the planning node which needs to be launched.

```xml
<node pkg="teb_planner_pa" type="teb_planner_node_pa" name="teb_planner_node_pa"
        output="screen">
        <rosparam file="$(find planning)/config/teb_herocar_config.yaml" command="load" />
</node>
```

The `teb_planner_node_pa` will also publish its view of the world. In order to visualize this in Rviz a `static_transform_publisher` is used. With this you can see the obstacles and path in Rviz (next to the car).

```xml
<node pkg="tf" type="static_transform_publisher" name="teb_map"
        args="0.0 0.0 -703.0 0 0 0 1 hero teb_map 100" />

<node pkg="rviz" type="rviz" name="rviz_teb"
    args="-d $(find teb_planner_pa)/cfg/rviz_teb_planner_node.rviz" />
```

### Configuration files

The TEB Planner was already integrated with dynamic-reconfigure. For this several files are necessary.

#### [config/teb\_herocar\_config.yaml](config/teb_herocar_config.yaml)

The hero_car planning config is the most important config file. It describes how the planner comes up with a path.
In the acting documentation more info on the parameters is given. It is recommended not to manually test these parameters at launch, but rather play with RQT dynamic-reconfigure sliders to adjust them.

#### [config/motionplan\_config.yaml](config/motionplan_config.yaml)

For the motion_planning node 3 parameters can be set.
These define how much of the trajectory around the car the path should be planner by the motion_planner.
These are used by the `_generate_overtake_trajectory` method.

#### [config/MotionPlan.cfg](config/MotionPlan.cfg)

This file enables the above described parameters to be dynamically set with RQT.
