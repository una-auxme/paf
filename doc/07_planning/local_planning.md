# Local Planning

**Summary:** Documentation of Collision Check node and ACC in package local planner

---

## Author

Samuel Kühnel

## Date

17.12.2023

## Collsision Check

### Subscibed Topics

* `/carla/hero/Speed`: Get current vehicle speed
* `/paf/hero/Center/min_distance`: Get min distance from LIDAR

### Published Topics+

* `/paf/hero/emergency`: Boolean that indicates if emergency brake is needed
* `/paf/hero/collision`: Collision object (Float32 Array) with distance and speed of obstacle in front

### Tasks

#### Calculate speed of obstacle in front

When the node recieves a distance from the LIDAR it is saved together with a timestamp so when the next distance message arrives the speed can be approximated.

This could be removed in the future, as when the radar gets involved the speed no longer needs to be approximated.

#### Check if crash is ahead

The Collision Check checks based on the current speed and last distance if a collsision with the obstacle in front is ahead.

The code looks like this:

```python
    obstacle_speed, distance = obstacle
    collision_time = time_to_collision(obstacle_speed, distance)
    # Calculation of distance for emergency stop
    emergency_distance = calculate_rule_of_thumb(True, __current_velocity)
    if collision_time > 0:
        # Check if emergency brake is needed to stop
        if distance < emergency_distance:
            # Initiate emergency brake
            self.emergency_pub.publish(True)
            return
        # When no emergency brake is needed publish collision distance for
        # ACC and Behaviour tree
        data = Float32MultiArray(data=[distance, obstacle_speed])
        self.collision_pub.publish(data)
    else:
        # If no collision is ahead publish np.Inf
        data = Float32MultiArray(data=[np.Inf, obstacle_speed])
```

For calculating the distance the "rule of thumb" is used.

$$
    distance_{safety} = speed + (speed \cdot 0.36)^2
$$

$$
    distance_{emergency} = speed + \frac{(speed \cdot 0.36)^2}{2}
$$

## ACC

### Subscibed Topics

* `/carla/hero/Speed`: Get current vehicle speed
* `/paf/hero/collision`: Get the collision object
* `/paf/hero/speed_limits_OpenDrive`: Get speedlimits from waypoints
* `/paf/hero/trajectory`: Get current trajectory
* `/paf/hero/emergency`: Deactivate ACC if emergency brake is initiated
* `/paf/hero/current_pos`: Get current position

### Published Topics+

* `/paf/hero/acc_velocity`: Velocity to keep distance to object in front

### Tasks

#### Get current speed limit

The ACC subscribes to the trajectory, speed limit and current position.

Every time the current position is updated the node calculates the current speed limit based on the trajectory and the speedlimits ordered by the waypoints.

#### Calculate speed for motion planning

By default the node publishes the current speed limit.

If a collision is recieved by the Collision Check the loop gets deactivated and a appropriate speed is calculated by this formula:

$$
speed_{acc} = speed_{obstacle} \cdot \frac{distance_{obstacle}}{distance_{safety}}
$$

The ACC speed depends on the obstacles speed and distance.
