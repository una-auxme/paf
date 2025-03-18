# Lane Change Behavior

**Summary:** This file explains the Lane Change behavior.

- [General](#general)
- [Ahead](#ahead)
- [Approach](#approach)
- [Wait](#wait)
- [Change](#change)

## General

This behaviour executes a lane change. It proceeds an early lane change when the change lane is free. It also detects if the hero is already on the desired lane (e.g. through an overtake).

It slows the vehicle down when the lane change point is reached (and no change occured till then) and then proceeds to switch lanes.

The behavior also detects if a change to left or right is planned and reacts to this accordingly.

## Ahead

Checks whether the next waypoint (/paf/hero/current_waypoint) is a lane change and inititates the lane change sequence accordingly.

When a lane change is ahead, a stop marker gets published at its position, preventing our car to drive on the change lane unchecked. This avoids crashed with traffic that is driving on the change lane.

## Approach

Tries to do an early lane change. \
For this purpose, the is_lane_free function is used while we are still driving (on the old lane).

If the lane is free, the trajectory immediatly gets planned to the desired change lane, this happens with the help of the request_start_overtake() service.

If the lane is not free, the hero continues to drive on the old lane with a continuous check whether the lane is now free.

Once the car is nearby the change point (< TARGET_DISTANCE_TO_STOP_LANECHANGE) and no change occured yet, a switch to the wait subbehavior takes place. Meanwhile, because of the stop marker set in the Ahead subbehavior, the hero stops just before the change point (still on the old lane).

Only tries to approach when we are still on the old lane. (Through a
scheduled overtake before it can happen that the hero is already on the disired change lane.)

## Wait

Waits at the lane change point until the lane change is not blocked. This is executed with the
help of the is_lane_free function.

Only wait when we are still on the old lane (not changed in approach before as the change lane was not free till now).

## Change

Executes the lane change. This will delete the stop marker that prevented driving to the next lane. As the change is only executed after the lane is free the change should be collision free.

When the car is already on its desired lane when entering the change state, this subbehavior only is only responsible for a correct end of the lane change behavior (see below).

As soon as we moved more than five meters away from the global lane change position (from the waypoint message) the change is considered as done and the whole lane change behavior completes.
