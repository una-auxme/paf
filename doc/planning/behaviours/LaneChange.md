# Lane Change Behavior

**Summary:** This file explains the Lane Change behavior.

- [General](#general)
- [Ahead](#ahead)
- [Approach](#approach)
- [Wait](#wait)
- [Change](#change)

## General

This behaviour executes a lane change. It proceeds an early lane change when the change lane is free. It also detects if we are already on the desired lane (e.g. thorugh an overtake). \
It slows the vehicle down when the lane change point is reached (and no change occured till then) and then proceeds to switch lanes. \
The behavior detects if a change to left or right is planned.

## Ahead

Checks whether the next waypoint is a lane change and inititates the lane change sequence accordingly. \
When a lane change is ahead, a stop marker gets published at its position, preventing our car to drive on the change lane unchecked. This avoids crashed with traffic that is driving on the change lane.

## Approach

Calculates a virtual stop line at the lane change point and publishes lc_app_blocked for motion planner to slow down while too far away.

If the lane change is not blocked (currently not implemented) the car does not slow down (30 km/h).

Once the car is within a set distance of the virtual stop line and not blocked it returns SUCCESS. SUCCESS is also returned when the car stops at the stop line.

## Wait

Waits at the lane change point until the lane change is not blocked. This is executed with the
help of the is_lane_free function. \
Only wait when we are still on the old lane (not changed in approach before as the change lane was not free till now).

## Change

Executes the lane change. This will delete the stop marker that prevented driving to the next lane. As the change is only executed after the lane is free the change should be collision free.

When the car is already on its desired lane when entering the change state, this subbehavior only is only responsible for a correct end of the lane change behavior (see below).

As soon as we moved more than five meters away from the global lane change position (from the waypoint message) the change is considered as done and the whole lane change behavior completes.