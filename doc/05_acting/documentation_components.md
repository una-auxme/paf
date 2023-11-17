# Documentation: Acting Components

**Summary**: This documentation gives a more indepth overview over the Components of Acting and how they work.

## Authors

Alexander Hellmann-Schweikardt

## Date

17.11.2023

## Components

### stanley_controller.py

* Inputs:
  * **trajectory**: Path
    * **current_pos**: PoseStamped
    * **Speed**: CarlaSpeedometer
  * **current_heading**: Float32
* Outputs:
  * **stanley_steer**: Float32
  * **stanley_debug**: StanleyDebug
* Stanley Controller with Kce = 0.1 and Kv = 1 !TODO tune!
* Gets closest point on **trajectory** and the target heading from **trajectory**
* Calculate position error (cross_err) with **current_pos** and heading error (heading_err) with **current_heading**
* Calculate steering angle with Stanley Controller and errors steering = heading_err + atan((Kce * cross_err) / current_velocity * Kv) Correct use of the controller??
* steering_angle = steering_angle * -1 why necessary? again, correct use??
* **stanley_steer** = steering_angle
* **stanley_debug** = heading, trajectory_heading, cross_err, heading_err and steering_angle

### pure_pursuit_controller.py

* Inputs:
  * **trajectory**: Path
  * **current_pos**: PoseStamped
  * **Speed**: CarlaSpeedometer
  * **current_heading**: Float32
* Outputs:
  * **pure_pursuit_steer**: Float32
  * **pure_pursuit_steer_target_wp**: Pose
  * **pure_p_debug**: Debug
* Pure Pursuit Controller with Kld = 1.0, look_ahead_dist = 3.5 and l_vehicle = 2.85 !TODO: Check if good!
* MIN_LD_V = (float) 3.0 !check if good!
  * if **Speed** < MIN_LD_V : look_ahead_distance += 0
  * else look_ahead_distance += Kld * (**Speed** - MIN_LD_V)
* creates vector from **current_pos** to next waypoint on **trajectory**
* get target heading from vector-angle
* alpha = target heading - **current_heading**
* steering_angle = atan( ( 2 * l_vehicle * sin (alpha) ) / look_ahead_distance) !check if correct use of controller!
* **pure_pursuit_steer** = steering_angle
* **pure_pursuit_steer_target_wp** = target waypoint pose from trajectory !why, this is not calculated, maybe for debugging?!
* **pure_p_debug** = heading, target heading, look_ahead_distance and steering angle

### velocity_controller.py

* Inputs:
  * **max_velocity**: Float32
  * **Speed**: CarlaSpeedometer
  * **max_tree_velocity**: Float32
  * **trajectory**: Path
  * **speed_limits_OpenDrive**: Float32MultiArray
  * **current_pos**: PoseStamped
* Outputs:
  * **speed_limit**: Float32
    * **throttle**: Float32
    * **velocity_as_float**: Float32: published only for rqt_plot which can not read the speed-data in rosbridge (investigate?)
* Position on OpenDrive-Map is calculated using **current_pos** and the waypoints of **trajectory**
* speed_limit = limit from **speed_limits_OpenDrive** for calculated position
* PID-Controller with Kp = 0.25 , Ki = 0 (why?) , Kd = 0.1 !not that great, TODO tune!  
* PID point: v = min(**max_velocity**, **max_tree_velocity**, speed_limit)
* **throttle** is calculated with **Speed** using the PID but also 0 <= **throttle** <= 1
* **speed_limit** = speed_limit

### vehicle_controller.py

* Inputs:
  * **emergency**: Bool
  * **Speed**: CarlaSpeedometer
  * **throttle**: Float32
  * **pure_pursuit_steer**: Float32
  * **stanley_steer**: Float32
* Outputs:
  * **vehicle_control_cmd**: CarlaEgoVehicleControl
  * **status**: Bool
  * **controller**: Float32
  * **emergency**: Bool
* (PURE_PURSUIT_CONTROLLER: int = 1 STANLEY_CONTROLLER: int = 2)
* STANLEY_CONTROLLER_MIN_V: float = 4.0 # ~14kph
* MAX_STEER_ANGLE: float = 0.75
* Steering:
  * Choose_Controller by current Speed on *Sigmoid Curve* around STANLEY_CONTROLLER_MIN_V -> p_stanley (how much “stanley” steering is used 0 < p < 1)
![stanley_anteil.png](https://github.com/una-auxme/paf23/blob/main/doc/00_assets/acting/stanley_anteil.png?raw=true)![](https://github.com/una-auxme/paf23/blob/main/doc/00_assets/acting/stanley_anteil.png)
  * steering = p_stanley * **stanley_steering** + (1-p_stanley) * **pure_pursuit_steer**
  * since this is only the total steering, another PID Controller Kp = 0.5 Ki = 0.1 Kd = 0.1 with range[-MAX_STEER_ANGLE; MAX_STEER_ANGLE] is used to steer “smoothly” !!TODO check if good tuning!!
    * steer = PID(steering) with 0 as “workingpoint”
    * CAREFUL in function map_steering another tune_k = -5 is used for tuning
* Throttle:   !!No backwards driving possible at the moment!!
  * if **throttle** > 0 : brake = 0 and throttle = **throttle**
  * else (**throttle** <= 0): brake = abs(**throttle**) and throttle = 0
* Handbrake, manual_gear_shift normally FALSE and gear = 1 ??why always back to gear 1 in loop?!
* **vehicle_control_cmd** = steer, throttle, handbrake, manual_gear_shift, gear and timestamp
* **controller** = the major Controller (PP or Stanley) used (p > 0.5) (PURE_PURSUIT_CONTROLLER: int = 1 STANLEY_CONTROLLER: int = 2)
* **status** = TRUE in run (saying if status=true the vehicle_controller is running, nothing more)
* **emergency** = FALSE only used AFTER an emergency break happened and the car fully stopped, then vehicle controller publishes, that the emergency break can end
* if **emergency** == TRUE is received, engage emergency break with !! MANY STRANGE VALUES, steer=1 is 90° and impossible, why throttle = 1 that means we try accelerating while braking? why reverse = true, noone switches into reverse gear for an emergency break, etc.!!!

### [acc.py (Adaptive Cruise Control)](https://github.com/una-auxme/paf23/blob/main/doc/05_acting/02_acc.md)

* Inputs:
  * **acc_distance** Float32: Distance to the car in front
  * **Speed** CarlaSpeedometer: current speed of the vehicle
* Outputs:
  * **max_velocity** Float32: calculated speed
  * **emergency** Bool: if collision seems likely (too little distance recieved), trigger emergency break
  * (**d_dist**) Float32: Debugging/Testing: Optimal Distance - self Distance
* Calculates the optimal speed max_velocity to drive at a safe optimal distance to the car in front. Optimal in simple situations like following another car on a road, careful with use (no detection in sharp curves can lead to unnecessary strong acceleration)
* Turns on only with acc_distance published, otherwise turned off
* Turns off after 1s of no new acc_distance published or acc_distance < 0 published
* publishes targetspeed max_velocity of 0 if turned off
* maybe needs more investigation on how good this actually works, formula is provided, see link
* Triggers emergency break if acc_distance < 0.5 * optimal distance.
* Testdummies available: AccDistancePublisherDummy

### TODO acting_velocity_publisher.py: Calculates max_velocity also based on trajectory and maybe upcoming curves

* Inputs:
  * **Speed**: CarlaSpeedometer !! Subscriber is Doubled in Code, human error?!
  * **trajectory**: Path
  * **current_heading**: Float32
  * **current_pos**: PoseStamped
* Outputs:
  * **max_velocity**: Float32
* PARKIN_DUR float = 5.0 duration for leaving the parking spot in the beginning,
* PARKING_V: float = 2.0 velocity while leaving parking spot, no use after PARKING_DUR is over !! TODO: Check if we really need so much extra for parking, does this only exist for the first 5 seconds to leave the parking spot?
* MAX_VELOCITY: float = 25.0 = 90km/h
* TODOTODOTODOTODOTODOTODOTODO

## Zusatzkomponenten

### trajectory_interpolation.py

* **points_to_vector(p_1: Tuple[float, float], p_2: Tuple[float, float]) -> Tuple[float, float]**
Create the vector starting at p1 and ending at p2
* **vector_len(vec: Tuple[float, float]) -> float**
Compute the given vector's length
* **add_vector(v_1: Tuple[float, float], v_2: Tuple[float, float]) -> Tuple[float, float]**
Add the two given vectors
* **rotate_vector(vector: Tuple[float, float], angle_rad: float) -> Tuple[float, float]**
Rotate the given vector by an angle
* **linear_interpolation(start: Tuple[float, float], end: Tuple[float, float], interval_m: float) -> List[Tuple[float, float]]**
Interpolate linearly between start and end, with a minimal distance of interval_m between points.
* **_clean_route_duplicates(route: List[Tuple[float, float]], min_dist: float) -> List[Tuple[float, float]]**
Remove duplicates in the given List of tuples, if the distance between them is less than min_dist.
* **interpolate_route(orig_route: List[Tuple[float, float]], interval_m=0.5)**
Interpolate the given route with points inbetween,holding the specified distance interval threshold.

### helper_functions.py

* **vectors_to_angle_abs(x1: float, y1: float, x2: float, y2: float) -> float**
Returns the angle (radians) between the two given vectors
* **vector_angle(x1: float, y1: float) -> float**
Returns the angle (radians) of a given vectors
* **vector_to_direction(x1, y1, x2, y2) -> float**  
    Returns the direction (angle to y-axis) of a vector  
* **quaternion_to_heading(x: float, y: float, z: float, w: float) -> float**
Translates quaternion to euler heading.
* **heading_to_quaternion(heading: float) -> (float, float, float, float)**
Translates euler heading to quaternion
* **calc_path_yaw(path: Path, idx: int) -> float**
Calculates the path yaw
* **normalize_angle(angle: float) -> float**
Normalizes an angle to [-pi, pi]
* **calc_egocar_yaw(pose: PoseStamped) -> float**
Calculates the yaw of the ego vehicle

### MainFramePublisher.py

* Inputs:
  * **current_pos**: PoseStampled
  * **current_heading**: Float32
* Outputs:
  * **transform**: broadcasts heroframe-transform via TransformBroadcaster
* This node handles the translation from the static main frame to the moving hero frame. The hero frame always moves and rotates as the ego vehicle does. The hero frame is used by sensors like the lidar. Rviz also uses the hero frame. The main frame is used for planning
* rotation = - **current_heading**
* position x = cos(rotation) * **current_pos**.x + sin(rotation) * **current_pos**.y
* position y = sin(rotation) * **current_pos**.x + cos(rotation) * **current_pos**.y
* position z = - **current_pos**.z
* rot_quat = rot as quaternion
* **transform** = position x/y/z, rot_quat, Timestamp(now), “global”, “hero”
