# Unstuck Behavior

**Summary:** This file explains the unstuck behavior used as a fallback to recover from stuck situations.

- [Explanation](#explanation)

## Explanation

The Unstuck Behavior is triggered whenever we find ourselves in a stuck situation for some specified amount of time. This Behavior can overwrite any other behavior, since we can also get stuck waiting inside other behaviors.

The Unstuck Behavior works with two Timers:

1. ```Wait Stuck Timer```
2. ```Stuck Timer```

Whenever we are driving at some velocity > 0.1 m/s the timers are reset.

The Wait Stuck Timer duration until the Unstuck gets triggered is based on the current behavior published by curr_behavior:

| Duration (sec) | Behavior |
| -------------- | -------- |
| 60             | int_wait, int_app_to_stop (Wait for green light at intersection) |
| 30             | lc_wait, ot_wait, None = no published behavior in parking space (Wait for lane free)|
| 15             | Default (all other behaviors) |
| 5              | Unstuck executed before, not moved for more than 15 meter (still stuck) |

The different durations will prevent triggering the unstuck routine too fast when we have a scheduled wait.

If we get into a situation where the velocity <= 0.1 m/s the timers start running.

We can then either trigger the Wait Stuck Timer condition
(duration see table above) after not moving for longer than that duration -> Wait Stuck

OR

we trigger the quicker Stuck Timer condition (currently 8 sec), after not moving for longer than that duration WHILE being told to move -> Stuck.

Once one of the Timers is triggered we do the following:

1. **Drive reverse:** \
  Based on the count how many times the Unstuck routine was already executed while not moved for more than 15 meters (still stuck) we do following:
  
    | Unstuck Count | Behavior |
    | -------------- | -------- |
    | 0             | Do nothing additional. |
    | 1             | Reset the planned overtake trajectory and remove all added stopmakers. |
    | 2             | Add an overtake 2.75m to the left. |
    | 3              | Add an ovetake 1.00m to the right. Reset Unstuck Count to 0. |

   While driving in reverse:

    0. **Invert steering angle** that it points towards the trajectory.
    This turns the car towards the trajectory, while reversing
    1. **Check** if there is an **obstacle** inside the obstacle mask
    2. If there is none, then **reverse slowly** (2 m/s) via the add_speed_override() service, until the **UNSTUCK_CLEAR_DISTANCE** is reached, or an obstacle appears inside the collision mask.
    3. If an **obstacle** is **detected** but the car has not moved more than 0,5 m
    -> keep reversing.\
    Otherwise the car could be stuck forever, because it is not moving at all.
    4. If there is an **obstacle** or the **UNSTUCK_CLEAR_DISTANCE** is reached\
    -> Set speed to 0 m/s via the add_speed_override() service -> this triggers braking and **standing still**.
2. **Drive forward:** \
  After driving backward we try driving forward. This step is preventing any stop marker appear directly after driven backward or trajectory gets changed when other behaviors gets triggered again. Directly appeared stop marker or planned overtakes could lead directly to a new unstuck again!

The duration of the driving backward and driving forward each is defined by **UNSTUCK_DRIVE_DURATION**. Currently, every step gets executed for 5 seconds.

This behavior is ONLY implememted to improve route completion and make debugging easier. It is the last fallback behavior when every other planning component can not get us back on track.
