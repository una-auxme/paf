# Leave Parking Space Behavior

**Summary:** This file explains the Leave Parking Space behavior.

- [Leave Parking Space Behavior](#leaveparkingspace-behavior)
  - [General](#general)

## General

The leave parking space behaviour is only executed at the beginning of the simulation to leave the parking space.

The behaviour calculates the euclidian  distance between the starting position and the current position to determine whether the car has fully left the parking space. If that is the case a "called"
flag is set to true so that this behaviour is never executed again and FAILURE is returned to end the behaviour. Otherwise it stays in RUNNING.
