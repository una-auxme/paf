# Overview of the Vehicle Controller Component

**Summary:** This page provides an overview of the current status of the Vehicle Controller Component.

- [General Introduction to the Vehicle Controller Component](#general-introduction-to-the-vehicle-controller-component)
- [Vehicle Controller Output](#vehicle-controller-output)
- [Emergency Brake](#emergency-brake)
- [Unstuck Routine](#unstuck-routine)

## General Introduction to the Vehicle Controller Component

The [Vehicle Controller](../../code/control/src/vehicle_controller.py) collects all information from the other controllers in Control ```throttle```, ```brake```, ```reverse```, ```pure_puresuit_steer```
to fill them into the CARLA-Vehicle Command Message ```vehicle_control_cmd``` and send this to the CARLA simulator.

It also reacts to some special case - Messages from Planning, such as emergency-braking or executing the unstuck-routine.

## Vehicle Controller Output

As the ```vehicle_control_cmd```-Message requires all 4 Inputs to be in the range of 0 to 1, the Vehicle Controller has to convert the steering signal ```pure_puresuit_steer``` from Radians to [0,1].

The ```throttle``` and ```brake``` are already calculated in the correct range by the PID Controller of the [Velocity Controller](../../code/control/src/velocity_controller.py).

This output (vehicle command) has to be sent in the same frequency the leaderboard is expecting them, which currently is about ```20 Hz``` (every 0.05 seconds).

If we send these commands in a lower frequency the leaderboard keeps waiting for an output, which leads to massive lags!

## Emergency Brake

The Vehicle Controller also reacts to ```emergency```-Messages, published by Planning:

Once the ```emergency_sub``` receives an emergency message from ```paf/hero/emergency```, the ```__emergency``` attribute gets set to either True or stays False.

In case the ```__emergency``` attribute is set to True, the main loop of the vehicle controller ignores any other vehicle command and goes straight into the ```__emergency_brake``` method until the emergency is resolved.

If an emergency is triggered the ```__emergency_brake``` method uses a little braking bug abuse, sending the following vehicle command:

```Python
    message.throttle = 1
    message.steer = 1
    message.brake = 1
    message.reverse = True
    message.hand_brake = True
    message.manual_gear_shift = False
```

The ```__emergency``` attribute can ONLY be set back to False by the ```__get_velocity```, once the car came to a full stop (v < 0.1).

This is done to prevent firing the emergency brake each time the main loop is refreshing.

Comparison between normal braking and emergency braking:

![Braking Comparison](/doc/assets/control/emergency_brake_stats_graph.png)

_Please be aware, that this bug abuse might not work in newer updates!_

## Unstuck Routine

The Vehicle Controller also reads ```current_behavior```-Messages, published by Planning, currently reacting to the **unstuck-behavior**:

This is done to drive in a specific way whenever we get into a stuck situation and the [Unstuck Behavior](/doc/planning/behaviors/Unstuck.md) is persued.

Inside the Unstuck Behavior we want to drive backwards with inverted steering, which is why the steering angle published by [Pure Pursuit Controller](../../code/control/src/pure_pursuit_controller.py) gets inverted.

```[Last updated 22.03.2025]```