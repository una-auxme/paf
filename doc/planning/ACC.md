# ACC (Adaptive Cruise Control)

**Summary:** The ACC module is a ROS node responsible for adaptive speed control in an autonomous vehicle. It receives information about the leading vehicle (if there is one), the current speed, the trajectory, and the speed limits.
Based on this information, it calculates the desired speed and publishes it.

- [ROS Data Interface](#ros-data-interface)
  - [Published Topics](#published-topics)
  - [Subscribed Topics](#subscribed-topics)
  - [Services](#services)
- [Debugging](#debugging)
- [Functionality](#functionality)

## ROS Data Interface

### Published Topics

This module publishes the following topics:

- `/paf/hero/acc_velocity`: The desired speed for the vehicle.
- `/paf/hero/emergency`: A flag indicating whether an emergency brake is required.
- `/paf/hero/acc/debug_markers`: Markers that show debugging data, e.g., collision masks, leading vehicle, and trajectory intersections.

### Subscribed Topics

This module subscribes to the following topics:

- `/paf/hero/mapping/init_data`: The map published by the intermediate layer.
- `/paf/hero/speed_limit`: The current speed limit.
- `/paf/hero/trajectory_local`: The local trajectory of the vehicle.
- `/paf/hero/curr_behavior`: The current behavior of the vehicle.
- `/paf/hero/pure_pursuit_steer`: The current steering angle of the vehicle.

### Services

This module provides the following service:

- `/paf/hero/acc/speed_alteration`: A service to override the speed or set an external speed limit.

This service is currently used by the unstuck routine to drive backwards or stop. Apart from that, the intersection behavior uses it to be able to drive slowlier when going straight forward through an intersection (hotfix to avoid crashing into a firetruck).

## Debugging

During development, the debugger can be a helpful tool. In order to be able to use the debugger, the planning.launch file has to be adapted. For more details, check the [debugging guide](../development/debugging.md).

## Functionality

Each time the map from the intermediate layer is received, the ACC triggers the function `update_velocity` where the main logic of the ACC can be found.

The following key functions are performed by the ACC node:

1. **Initialization**:
   - Subscribes to various topics to receive data about the map, speed limits, trajectory, current behavior, and steering angle.
   - Publishes the desired speed, emergency brake flag, and debugging markers.
   - Provides a service to handle speed alterations.

2. **Update Velocity**:
   - Checks if the necessary data (map and trajectory) is available.
   - Defines a trajectory mask and a rectangle in front of the car that is used to set the area in which a leading vehicle is searched. The rectangle is able to detect possible collisions that are close to the car while the trajectory mask checks the area at a higher distance from the car.
   - Publishes markers to visualize the masks, the chosen leading vehicle, and trajectory intersections.
  
   ![Leading vehicle markers visualization](../assets/planning/ACC_trajectory_mask_visualization.PNG)
   - Identifies the leading vehicle based on the trajectory mask and the rectangle.
   - Calculates a reasonable speed depending on the leading vehicle using the function `calculate_velocity_based_on_lead`.
   - Considers speed limits, external speed limits, and trajectory-based cornering speeds.
   - Publishes the desired speed (minimum of the considered speeds), debugging markers, and emergency brake flag if necessary.

3. **Calculate Velocity Based on Lead**:
   - Calculation of the desired speed using the function `calculate_velocity_based_on_lead`. This function uses a PI controller to calculate the desired speed based on the distance and speed of the leading vehicle.

4. **Calculate Velocity Based on Trajectory**:
   - Approximates a maximum safe cornering speed by tracing lines at an angle from the front of the car and measuring the distance at which they intersect with the trajectory.
  
   ![Cornering speed visualization](../assets/planning/ACC_curve_speed_visualization.PNG)
   - Uses linear interpolation to calculate the desired speed based on the intersection distance.

5. **Emergency Braking**:
   - Emergency braking if a slow obstacle is detected and the speed difference is too high.
   - Triggers an emergency brake if a slow obstacle is detected, the speed difference is too high, and the vehicle is moving fast.
   - Publishes an emergency flag to notify the [vehicle controller](../../code/control/src/vehicle_controller.py).

6. **Handle Speed Alteration**:
   - Handles requests to override the speed or set an external speed limit. This enables other components (such as the unsuck routine or the intersection behavior) to set a maximum speed or a fixed speed if necessary.

The ACC node ensures that the vehicle maintains a safe speed by considering potential collisions, speed limits, trajectory-based cornering speeds, and emergency braking conditions.
