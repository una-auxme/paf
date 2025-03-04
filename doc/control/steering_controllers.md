# Overview of the Steering Controllers

**Summary:** This page provides an overview of the current status of the used steering controller PurePursuit.

- [General Introduction to Steering Controllers](#general-introduction-to-steering-controllers)
- [PurePursuit Controller](#purepursuit-controller)

## General Introduction to Steering Controllers

Since the CARLA Vehicle is supposed to follow a given trajectory (published by Planning), our project needs a Controller to calculate steering signals from the given ```trajectory``` and the vehicle's ```current_pos``` and ```current_heading``` from the GNSS and IMU sensors (Perception!).
Currently, one Steering Controller [PurePursuit Controller](#purepursuit-controller) is implemented.

Before, there was also the Stanley Controller which was discontinued in PAF24 as the PurePursuit was working overall better. You can find the important information about the old Stanley Controller in the subfolder 'discontinued' in [stanley.md](./discontinued/stanley.md).

**IMPORTANT:** The CARLA ```vehicle_control_cmd``` only allows you to use a ```steer``` value with an allowed range from 0-1 (!) to control the steering.

## PurePursuit Controller

The [PurePursuit Controller's](../../code/control/src/pure_pursuit_controller.py) main feature to determine a steering-output is the so-called **look-ahead-distance d_la** (l_d in Image).
For more indepth information about the PurePursuit Controller, click [this link](https://de.mathworks.com/help/nav/ug/pure-pursuit-controller.html) and [this link](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html).

At every moment it checks a point of the trajectory in front of the vehicle with a distance of **$d_{la}$** and determines a steering-angle so that the vehicle will aim straight to this point of the trajectory.

![MISSING: PurePursuit-ShowImage](../assets/control/Steering_PurePursuit.png)

This **look-ahead-distance $d_{la}$**  is velocity-dependent, as at higher velocities, the controller should look further ahead onto the trajectory.

$$ d_{la} = k_{ld} \cdot v $$

$$ \delta = arctan({2 \cdot L_{vehicle} \cdot sin(\alpha) \over d_{la}})$$

To tune the PurePursuit Controller, you can tune the factor of this velocity-dependence **$k_{ld}$**.
Also, for an unknown reason, we needed to add an amplification to the output-steering signal before publishing aswell **$k_{pub}$**, which highly optimized the steering performance in the dev-launch:

![MISSING: PurePursuit-Optimization_Image](../assets/control/Steering_PurePursuit_Tuning.png)

**NOTE:** The **look-ahead-distance $d_{la}$** should be highly optimally tuned already for optimal sensor data and on the dev-launch!
In the Leaderboard-Launch this sadly does not work the same, so it requires different tuning and needs to be optimized/fixed.