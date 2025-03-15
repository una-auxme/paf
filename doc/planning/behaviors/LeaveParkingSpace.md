# Leave Parking Space Behavior

**Summary:** This file explains the Leave Parking Space behavior.

- [General](#general)

## General

The leave parking space behavior is only executed at the beginning of the simulation to leave the parking space. \
It is also responsible that we don't start to drive while no intermediate layer map, trajectory and acc speed is set (because of no init yet).

The behavior places a stop marker to the left side of the car, leading to stop the car from uncontrolled unparking. \
With the is_lane_free function it is checked whether the left lane is free. If this is the case the stop marker gets removed and the car starts its way.

Also, if the car started moving more than five meter while the stop marker is still there, we know that the car started its way and this behavior is done (can occur e.g. when the devroute is started and no parking space is needed to be left).

Once one of these two situation occurs, a variable 'self.finished' is set to true so that this behavior is never executed again and FAILURE is returned to end the behavior. Otherwise it stays in RUNNING.


