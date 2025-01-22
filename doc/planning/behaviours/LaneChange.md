# Lane Change Behavior

**Summary:** This file explains the Lane Change behavior.

- [Lane Change Behavior](#lanechange-behavior)
  - [General](#general)
  - [Lane Change Ahead](#lanechange-ahead)
  - [Approach](#approach)
  - [Wait](#wait)
  - [Enter](#enter)
  - [Leave](#leave)

## General

This behaviour executes a lane change. It slows the vehicle down until the lane change point is reached and then proceeds to switch lanes.

## Lane Change ahead

Checks whether the next waypoint is a lane change and inititates the lane change sequence accordingly.

## Approach

Calculates a virtual stop line at the lane change point and publishes lc_app_blocked for motion planner to slow down while too far away.

If the lane change is not blocked (currently not implemented) the car does not slow down (30 km/h).

Once the car is within a set distance of the virtual stop line and not blocked it returns SUCCESS. SUCCESS is also returned when the car stops at the stop line.

## Wait

Waits at the lane change point until the lane change is not blocked (not implemented).

## Enter

Inititates the lane change with 20 km/h and continues driving on the next lane until the lane change waypoint is far enough away.

## Leave

Simply exits the behaviour.
