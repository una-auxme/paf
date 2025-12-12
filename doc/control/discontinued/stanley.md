# Stanley Controller

The [Stanley Controller's](./src/stanley_controller.py) main features to determine a steering-output is the so-called **cross-track-error** (e_fa in Image) and the **trajectory-heading** (theta_e in Image).
For more indepth information about the Stanley Controller, click [this link](https://medium.com/roboquest/understanding-geometric-path-tracking-algorithms-stanley-controller-25da17bcc219) and [this link](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf).

![MISSING: Stanley-SHOW-IMAGE](../../assets/control/Steering_Stanley.png)

At every moment it checks the closest point of the trajectory to itself and determines a two steering-angles:

- from checking the closest trajectory-point (and its neighbouring points) it calculates a **trajectory-heading $\theta_e$** as the optimal steering for this point on the trajectory.
- from getting the vector from the current position to the closest trajectory-point it calculates the **cross-track-error $e_{fa}$** with which it calculates a steering-angle to drive back onto the trajectory.

$$ \theta_e = \theta - \theta_p$$

$$ \delta = \theta_e - arctan({k_{ce} \cdot e_{fa} \over v})$$

To tune the Stanley Controller, you tune the factor **$k_{ce}$**, which amplifies (or diminishes) how strong the **cross-track-error**-calculated steering-angle will "flow" into the output steering-angle.

![MISSING: Stanley-Compared to PurePursuit](../../assets/control/Steering_Stanley_ComparedToPurePur.png)

As for the PurePursuit Controller, sadly the achieved good tuning in the Dev-Launch was by far too strong for the Leaderboard-Launch, which is why we needed to Hotfix the Steering in the last week to Tune Stanley alot "weaker". We do not exactly know, why the two launches are this different.
(Dev-Launch and Leaderboard-Launch differentiate in synchronicity, Dev-Launch is synchronous, Leaderboard-Launch is asynchronous?)

**NOTE:** In Dev-Launch, the Stanley Controller never surpassed PurePursuit in steering performance, so it was originally planned to simplify Acting and throw out the Stanley Controller.
Since this, for an unknown reason, did not apply to the Leaderboard-Launch, where Stanley performs ALOT better, you will have to look into this again.

## [stanley_controller.py](./src/stanley_controller.py)

Calculates steering angles that keep the ego vehicle on the path given by
the local path planning.

Inputs:

- **trajectory**: Path
- **current_pos**: PoseStamped
- **Speed**: CarlaSpeedometer
- **current_heading**: Float32

Outputs:

- **stanley_steer**: Float32
- **stanley_debug**: StanleyDebug

Subscription:

- ```/paf/hero/trajectory_global``` \(/PrePlanner\) ([nav_msgs/Path](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- ```/paf/hero/global_current_pos``` \(/position_heading_publisher_node\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/carla/hero/Speed``` \(/carla_ros_bridge\) ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- ```/paf/hero/global_current_heading``` \(/position_heading_publisher_node\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

Publishes:

- ```/paf/hero/stanley_steer``` \(/vehicle_controller\) ([std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))
- ```/paf/hero/stanley_debug``` \(/\) ([acting/msg/StanleyDebug](/../paf/code/acting/msg/StanleyDebug.msg))
