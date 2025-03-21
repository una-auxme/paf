# Overtake Behavior

**Summary:** This file explains the Overtake behavior.

- [General](#general)
- [Overtake ahead](#overtake-ahead)
- [Approach](#approach)
- [Wait](#wait)
- [Enter](#enter)
- [Leave](#leave)

## General

This behaviour is used to dynamically overtake an object in close proximity. An Overtake checks for traffic on the other lane and swaps lane when it is free.

After the overtake is finished, the vehicle returns to the original lane when it is free.

To handle the dynamic overtaking a overtake service has been implemented to allow overtake requests and status checks.

## Overtake ahead

Checks whether there is a object in front of the car that needs overtaking by using a map function.

Checks distance and speed of the object in front.

Increases a counter if an obstacle is detected within a certain threshold. Bicycles are checked with seperate conditions.

Returns SUCCESS if the counter exceeds the limit, FAILURE if no obstacle is found, and RUNNING while waiting.

## Approach

Handles the procedure for approaching an obstacle before overtaking.

Retrieves the distance to the obstacle and determines if the other lane is clear with a map function and a counter.

Sets a stopmarker shortly before the obstacle so ACC automatically slows down the car while approaching to stop with a sensible distance.

Returns SUCCESS if the vehicle has stopped or the oncoming lane is free, FAILURE if the overtake is aborted, and RUNNING while approaching the obstacle.

If the other lane is free while still approaching, a flag is set to skip the wait behavior and a overtake service request is sent to initiate the lane change.

## Wait

This handles wating for clear traffic on the other lane while the car has stopped behind the obstacle.

Determines if the other lane is clear with a map function and a counter.

When the other lane is free a overtake service request is sent to initiate the lane change.

Returns SUCCESS when the lane is clear, FAILURE if the obstacle moves away or disappears, and RUNNING while waiting.

## Enter

Handles switching the lane for overtaking.

Removes the stopmarker so that the car can move.

Requests the current overtake status with a service function.

Returns SUCCESS when the status is OVERTAKING, RUNNING while the OVERTAKE_QUEUED and FAILURE if the status is NO_OVERTAKE, OVERTAKE_ENDING or unknown.

## Leave

Runs until the overtake is fully finished by returning to the original lane and then leaves the behaviour.

Requests the current overtake status with a service function.

While OVERTAKING, checks if the original lane is free with a map function.

If it is free a request is sent to end the overtake and returns RUNNING.

If there is an obstalce in front in close proximity while overtaking a request is sent to end the overtake as well.

Returns FAILURE when the overtake is finished.
