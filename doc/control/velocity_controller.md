# Overview of the Velocity Controller

**Summary:** This page provides an overview of the current status of the velocity_controller.

- [General Introduction to Velocity Controller](#general-introduction-to-velocity-controller)
- [Current Implementation](#current-implementation)

## General Introduction to Velocity Controller

The [velocity_controller](../../code/control/src/velocity_controller.py) implements our way to make the CARLA-Vehicle drive at a ```target_velocity``` (published by the ACC) by using a tuned PID-Controller to calculate a ```throttle``` and a ```brake``` for the CARLA-Vehicle-Command.
For more information about PID-Controllers and how they work, follow [this link](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).

**IMPORTANT:** The CARLA ```vehicle_control_cmd``` only allows you to use a ```throttle``` and a ```brake``` value, both with an allowed range from 0-1, to control the driven speed.

## Current Implementation

Currently, we use a tuned PID-Controller which was tuned for the speed of 14 m/s (around 50 km/h), as this is the most commonly driven velocity in this simulation:

![MISSING: PID-TUNING-IMAGE](../assets/control/VelContr_PID_StepResponse.png)

Be aware, that the CARLA-Vehicle shifts gears automatically, resulting in the bumps you see!
As PID-Controllers are linear by nature, the velocity-system is therefore linearized around 50 km/h, meaning the further you deviate from 50 km/h the worse the controller's performance gets:

![MISSING: PID-LINEARIZATION-IMAGE](../assets/control/VelContr_PID_differentVelocities.png)

As the Velocity Controller also has to handle braking, we currently use ```throttle```-optimized PID-Controller to calculate ```brake``` aswell (Since adding another Controller, like a P-Controller, did not work nearly as well!):

![MISSING: PID-BRAKING-IMAGE](../assets/control/VelContr_PID_BrakingWithThrottlePID.png)

## Reverse driving
In PAF24 the reverse driving was added properly.\
The velocity_controler has three velocity checks (if there is a velocity published).
### target_velocity < 0
use the PID to calculate throttle / breakpedal position.\
Setting and publishing the flag ```reverse``` for the ```vehicle_controller```, so it can tell Carla to drive backwards.

### 0 <= target_velocity < 0.1
The car will stop and stand stil until the ```target_velocity``` changes.

### target_velocity > 0.1
Drive forward with the PID

### NOTE
The PID was not changed in PAF24!\
Tuning the PID for negativ speeds can be looked at if necessary.

### Last updated 22.03.2025