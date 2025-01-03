# Overtake Behavior

**Summary:** This file explains the Overtake behavior.

- [General](#general)
- [Overtake ahead](#overtake-ahead)
- [Approach](#approach)
- [Wait](#wait)
- [Enter](#enter)
- [Leave](#leave)

## General

This behaviour is used to overtake an object in close proximity. This behaviour is currently not working and more like a initial prototype.

## Overtake ahead

Checks whether there is a object in front of the car that needs overtaking.

Estimates whether the car would collide with the object soon. If that is the case a counter gets incremented. When that counter reaches 4 SUCCESS is returned. If the object is not blocking the trajectory, FAILURE is returned.

## Approach

This is running while the obstacle is still in front of the car.

Checks whether the oncoming traffic is far away or clear, if that is the case then ot_app_free is published as the current behaviour for the motion_planner and returns SUCCESS. Otherwise ot_app_blocked is published for the car to slow down.

If the car stops behind the obstacle SUCCESS is also returned.

## Wait

This handles wating for clear oncoming traffic if the car has stopped behind the obstacle. If the overtake is clear ot_wait_free gets published and SUCCESS is returned. Otherwise ot_wait_stopped gets published and the behaviour stays in RUNNING.

If the obstacle in front is gone the behaviour is aborted with FAILURE.

## Enter

Handles switching the lane for overtaking.

Waits for motion planner to finish the trajectory changes and for it to set the overtake_success flag.

## Leave

Runs until the overtake is fully finished and then leaves the behaviour.
