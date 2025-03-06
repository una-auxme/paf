# Unstuck Behavior

**Summary:** This file explains the unstuck behavior used as a fallback to recover from stuck situations.

- [Unstuck Behavior](#unstuck-behavior)
  - [Explanation](#explanation)

## Explanation

The Unstuck Behavior is triggered whenever we find ourselves in a stuck situation for some specified amount of time. This Behavior can overwrite any other behavior, since we can also get stuck waiting inside other behaviors.

The Unstuck Behavior works with two Timers:

1. ```Wait Stuck Timer```
2. ```Stuck Timer```

Whenever we are driving at some velocity > 0.1 the timers are reset.

Also if we are in a planned waiting behaviors (currently bs.int_wait.name, bs.lc_wait.name, bs.ot_wait_stopped) or don't have set a behavior yet
(means we are in unparking but not yet started driving -> Wait for lane free) the stuck timer gets reset. This will prevent triggering the unstuck
routine when we still wait scheduled. As we still have the wait stuck timer there is no chance to get in a 'hard' stuck here when something went
wrong in the waiting behaviors.

If we get into a situation where the velocity <= 0.1 the timers start running.

We can then either trigger the Wait Stuck Timer condition (currently 60 secs)
after not moving for longer than that duration -> Wait Stuck

OR

we trigger the quicker Stuck Timer condition (currently 20 secs), after not moving for longer than that duration WHILE being told to move -> Stuck.

Once one of the Timers is triggered we do the following:

1. **Reset** the planned **overtake** trajectory and **remove** all added **stopmakers**

2. **Inverting steering angle** that points towards the trajectory\
This turns the car towards the trajectory, while reversing\
This happens inside the [vehicle_controller.py](/doc/control/vehicle_controller.md)

3. Setting a n**egative target_velocity** with the **add_speed_override() Service**

4. **Calculate negative throttle** with PID\
This happens inside the [velocity_controller.py](/doc/control/velocity_controller.md)

5. **While driving in reverse**
    1. Check if there is an obstacle inside the obstacle mask
    2. if there is none, then reverse until the **UNSTUCK_CLEAR_DISTANCE** is reached, or an obstacle appears inside the collision mask.
    3. If an obstacle is detected but the car has'nt moved more than 0,5m
    -> keep reversing.\
    Otherwise the car could be stuck forever, because it is not moving at all.
    4. If there is an obstacle or the **UNSTUCK_CLEAR_DISTANCE** is reached\
    -> Set speed to 0.001 -> this triggers braking and standing still inside the **velocity_controller**
    5. Wait until the remaining time for this behaviour is over
    6. If the car still is in a stuck position\
    -> wait until the unstuck is detected and called again

This behavior is ONLY implememted to improve route completion and make debugging easier. It is the last fallback behavior when every other planning component can not get us back on track.

Files influenced by this behavior are:

- Planning:
  - [motion_planning.py](/code/planning/src/local_planner/motion_planning.py), for the target_speed and overtake
  - [behavior_speed.py](/code/planning/src/behavior_agent/behaviours/behavior_speed.py), for the target_speed
- Control:
  - [vehicle_controller.py](/doc/acting/vehicle_controller.md), because of inverting the pure pursuit steering angle
  - [velocity_controller.py](/doc/acting/velocity_controller.md), because of creating a negative throttle value
