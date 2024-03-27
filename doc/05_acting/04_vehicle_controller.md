# Overview of the Vehicle Controller Component

**Summary:** This page provides an overview of the current status of the Vehicle Controller Component.

---

## Authors

Robert Fischer, Alexander Hellmann

## Date

01.04.2024

<!-- TOC -->
- [Overview of the Vehicle Controller Component](#overview-of-the-vehicle-controller-component)
  - [Authors](#authors)
  - [Date](#date)
  - [General Introduction to the Vehicle Controller Component](#general-introduction-to-the-vehicle-controller-component)
  - [Vehicle Controller Output](#vehicle-controller-output)
  - [Emergency Brake](#emergency-brake)
  - [Unstuck Routine](#unstuck-routine)
<!-- TOC -->

## General Introduction to the Vehicle Controller Component

The [Vehicle Controller](../../code/acting/src/acting/vehicle_controller.py) collects all information from the other controllers in Acting ```throttle```, ```brake```, ```pure_puresuit_steer``` and ```stanley_steer```
to fill them into the CARLA-Vehicle Command Message ```vehicle_control_cmd``` and send this to the CARLA simulator.

It also reacts to some special case - Messages from Planning, such as emergency-braking or executing the unstuck-routine.

## Vehicle Controller Output

As the ```vehicle_control_cmd```-Message requires all 4 Inputs to be in the range of 0 to 1, the Vehicle Controller has to convert both steering signals ```pure_puresuit_steer``` and ```stanley_steer``` from Radians to [0,1].

The ```throttle``` and ```brake``` are already calculated in the correct range by the PID Controller of the Velocity Controller.

## Emergency Brake

The Vehicle Controller also reacts to ```emergency```-Messages, published by Planning:

## Unstuck Routine

The Vehicle Controller also reads ```current_behavior```-Messages, published by Planning, currently reacting to the **unstuck-behavior**:
