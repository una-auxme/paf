# Overview of the Vehicle Controller Component

**Summary:** This page provides an overview of the current status of the Vehicle Controller Component.

- [General Introduction to the Vehicle Controller Component](#general-introduction-to-the-vehicle-controller-component)
- [Vehicle Controller Output](#vehicle-controller-output)
- [Emergency Brake](#emergency-brake)
- [Unstuck Routine](#unstuck-routine)
  - [Last updated 22.03.2025](#last-updated-22032025)

## General Introduction to the Vehicle Controller Component

The [Vehicle Controller](../../code/control/control/vehicle_controller.py) collects the control outputs ```throttle```, ```brake```, ```reverse```, and ```pure_pursuit_steer```
to fill the CARLA vehicle command message ```vehicle_control_cmd``` and send it to the CARLA simulator.

The controller no longer uses a sleep-based hotfix to pace the simulator.
In the current synchronous setup, it waits until the critical upstream stages
for the current frame report completion and only then publishes the final
command for that frame. The required stages are currently mapping, motion
planning, ACC, pure pursuit, and the velocity controller.

If the barrier does not complete within the configured ```frame_barrier_timeout```, the controller publishes a safe stop command instead of releasing a stale or partial command.

It also reacts to some special case - Messages from Planning, such as emergency-braking or executing the unstuck-routine.

## Vehicle Controller Output

As the ```vehicle_control_cmd```-Message requires all 4 Inputs to be in the range of 0 to 1, the Vehicle Controller has to convert the steering signal ```pure_puresuit_steer``` from Radians to [0,1].

The ```throttle``` and ```brake``` are already calculated in the correct range by the PID controller of the [Velocity Controller](../../code/control/control/velocity_controller.py).

This output still has to be sent in the same frequency the leaderboard expects, which currently is about ```20 Hz``` (every 0.05 seconds).

The difference is that the release condition is now frame completion on the critical path, not an arbitrary sleep delay inside the controller.

## Emergency Brake

The Vehicle Controller also reacts to ```emergency```-Messages, published by Planning:

Once the ```emergency_sub``` receives an emergency message from ```paf/hero/emergency```, the ```__emergency``` attribute gets set to either True or stays False.

In case the ```__emergency``` attribute is set to True, the main loop of the vehicle controller ignores any other vehicle command and publishes a full stop command until the emergency is resolved.

If an emergency is triggered, the controller sends the following vehicle command:

```Python
  message.throttle = 0.0
  message.steer = 0.0
    message.brake = 1
  message.reverse = False
    message.hand_brake = True
    message.manual_gear_shift = False
```

The ```__emergency``` attribute can ONLY be set back to False by the ```__get_velocity```, once the car came to a full stop (v < 0.1).

This is done to prevent firing the emergency brake each time the main loop is refreshing.

Comparison between normal braking and emergency braking:

![Braking Comparison](/doc/assets/control/emergency_brake_stats_graph.png)

## Unstuck Routine

The Vehicle Controller also reads ```current_behavior```-Messages, published by Planning, currently reacting to the **unstuck-behavior**:

This is done to drive in a specific way whenever we get into a stuck situation and the [Unstuck Behavior](/doc/planning/behaviors/Unstuck.md) is persued.

Inside the Unstuck Behavior we want to drive backwards with inverted steering, which is why the steering angle published by [Pure Pursuit Controller](../../code/control/control/pure_pursuit_controller.py) gets inverted.

### Last updated 22.03.2025
