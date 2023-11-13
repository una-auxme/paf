# Research: PAF21_1 Acting


### Inputs
* waypoints of the planned route
* general odometry of the vehicle


### Curve Detection
* Can detect curves on the planned trajectory
* Calculates the speed in which to drive the detected Curve
![Curve](https://github.com/ll7/paf21-1/raw/master/imgs/curve.PNG)
### Speed Control
* [CARLA Ackermann Control](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_ackermann_control/)
* Speed is forwarded to the CARLA vehicle via Ackermann_message, which already includes a PID controller for safe driving/accelerating etc.
* no further controlling needed  -> speed can be passed as calculated


### Steering Control

##### Straight Trajectories
* **Stanley Steering Controller**
	* Calculates steering angle from offset and heading error
    * includes PID controller
 ![Stanley Controller](https://github.com/ll7/paf21-1/raw/master/imgs/stanley.png)

##### Detected Curves
* **Naive Steering Controller** (close to pure pursuit)
    * uses Vehicle Position + Orientation + Waypoints
    * Calculate direction to drive to as vector
    * direction - orientation = Steering angle at each point in time
    *  speed is calculated in Curve Detection and taken as is
