
# TEB Local Planner (Timed elastic bands)

**Summary**: This page provides an overview on what the Timed elastic bands local planner is.
How it works and the way it is integrated into our code.

- [Timed elastic bands](#timed-elastic-bands)
- [Integration with the PAF project](#integration-with-the-paf-project)
  - [Ros TEB Planner](#ros-teb-planner)
  - [Interface package](#interface-package)
- [Parameter Tuning](#parameter-tuning)
- [About move\_base](#about-move_base)

## Timed elastic bands

Timed elastic bands is a local planning approach which tries to optimize a trajectory around obstacles.
The image below shows the basic principle of operation.
A start pose, and an end pose is given and a trajectory between them is planned.
The trajectory tries to avoid the obstacles.

![MISSING: TEB-Example](./../assets/acting/single_teb.png)

## Integration with the PAF project

We decided to use a preexisting ROS-TEB-implementation.
The implementation is from the TU-Dortmund and is usually directly integrated into the move_base framework of ROS.
We do not use move_base and therefore need an interface package for this.

### [Ros TEB Planner](http://wiki.ros.org/teb_local_planner)

The ROS-TEB Planner is a local_planning_algorithm for base_local_planner ROS-package.
For our usage we need the package to be installed, but the interface package will take care of launching the required components.
The planner is integrated with dynamic reconfigure, therefor RQT reconfigure panel can be used to adjust the hyperparameters.
To read up on tuning the parameters I suggest this [article](https://mowito-navstack.readthedocs.io/en/latest/step_5c.html) and the official [documentation](http://wiki.ros.org/teb_local_planner).

Note that due to the interface package used maybe not all parameters are used in our case.
Also see below section for further talk about the parameters for our project.

### [Interface package](https://github.com/TUC-ProAut/ros_teb_planner)

We are using this interface package because it interfaces directly into the code of the teb_local_planner.
This allows us to not having to create costmap, odom and other mocks just to use move base.
The package gets started with the following launch syntax.

```xml
<node pkg="teb_planner_pa" type="teb_planner_node_pa" name="teb_planner_node_pa"
        output="screen">
        <rosparam file="$(find planning)/config/teb_herocar_config.yaml" command="load" />
</node>
```

As mentioned above this snipped launches the interface as well as the underlying planner.
The config file must at least contain information about the robot footprint as this cannot be set during runtime.
This could be of type:

```param
footprint_model:
    type: "circular"
    radius: 1.8
```

When launched a service is provided called "/teb_planner_node_pa/plan".
Documentation on this service request and response can be found [here](https://github.com/TUC-ProAut/ros_teb_planner/tree/main/teb_planner_pa_msgs)
In short a request with start and end position as well as viapoints and obstacles is sent and the service responds with a path.
This [launch config](https://github.com/TUC-ProAut/ros_teb_planner/blob/main/teb_planner_pa/launch/rviz.launch) is also useful as it shows the scene created by your requests.

## Parameter Tuning

There are many parameters which can be set in the param file or during live operation with RQT dynamic reconfigure.
It is good practice to tune the parameters in RQT and then bit by bit set them in the config file.
As there are many parameters we do not need and get ignored anyway I recommend not to export the config but only set relevant parameters which automatically lets non set parameters at default values.

Our current parameters are as follows:

```param
odom_topic: global
map_frame: global
```

These frame do not get checked in the code and are not strictly necessary but good practice.
But the response trajectory will have the frame_id correctly set if this is set.

```param
dt_ref: 0.3
dt_hysteresis: 0.1
```

The tooltip asks for dt_h beeing 0.1 * dt_ref.
These values however worked good at testing.
Smaller dts lead to more sampling points and therefor a smoother trajectory.

```param
no_inner_iterations: 30
no_outer_iterations: 3
optimization_activate: True
```

Obviously optimization should be activated.
The more iterations the longer the solver takes to calculate the trajectory.
Especially more outer loops drastically increase computation time, while inner loops way less harm

```param
enable_homotopy_class_planning: False
```

This setting is very important for performance.
If homotopy is activated multiple trajectories get calculated in each service call.
In our testing this did not perform better than "single-shot" optimization but drastically increased computation time.
With this set only ~4 Hz where possible.
Without this setting we reached nearly 20 Hz.

```param
min_obstacle_dist: 0.4
inflation_dist: 0.75

weight_obstacle: 10.0 
weight_inflation: 3.0
weight_shortest_path: 3.0
```

These parameters had the greatest impact on the performance of the planner in our testing.
With the obstacle and inflation distance we define how much space there should be between our vehicle and the obstacle.
The weights then try to accomplish this behavior.
Obstacle and inflation weight are obvious.
The shortest path weight needs to be in same range as the obstacle weights.
Slightly less is best.
This is because we want to avoid obstacles but still stay on an optimized trajectory and not ZIGZAG through the world because nobody optimizes for a smooth and short path.

```param
footprint_model:
    # maybe use two circles
    type: "circular"
    radius: 1.8
    #type: "polygon"
    #vertices: [ [2.5, -0.9], [-2.5, -0.9], [-2.5, 0.9], [2.5, 0.9] ] # for type "polygon" but car-like
    weight_viapoint: 10.0
wheelbase: 2.85               
min_turning_radius: 12        
```

In an ideal case the polygon shape would be optimal for this planning.
In testing the optimizer however failed miserable when trying this.
We are currently checking if "two_circles" might be a better trade off.
I strongly advise against following the polygon way but rather adjust parameters mentioned above for better performance.
The shape in the uncommented section was based on [this](https://www.motortrend.com/cars/lincoln/mkz/2020/specs/?trim=Base+Sedan)

## About [move_base](http://wiki.ros.org/move_base)

move_base is a ros package providing a full navigation stack.
The planning in move_base is separated into two stages: global and local planning.
The concept behind the local planning scheme is such that it can be easily expanded with custom planners.
[base_local_planner](http://wiki.ros.org/base_local_planner) acts a base implementation.
Several different local planners where implemented into this setup such as the [dwa_local_planner](http://wiki.ros.org/dwa_local_planner), or trajectory rollout.
But also the [teb_local_planner](http://wiki.ros.org/teb_local_planner) we are using.
An early goal was to integrate the full move_base interface into our code.
However the complete stack relies heavily on bitmapped costmaps which would be constantly changing in our situation. Also we are only interested in the local_planner which is not seperable from move_base (at least not without major efforts).
Because the teb_local_planner was already one of the preferred local_planners and a direct ros-interface ([ros_teb_planner](https://github.com/TUC-ProAut/ros_teb_planner)) was already available we decided to focus on this planner.
It is therefore not possible to exchange the teb_local_planner with any other planner easily.
For this we would need to integrate move_base.
