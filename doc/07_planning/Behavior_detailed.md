# Behaviors detailed

**Disclaimer**: The original behavior and decision tree are based on [PAF22](https://github.com/ll7/paf22)  and a previous [PAF project](https://github.com/ll7/psaf2).

**Summary:**
The behaviors are responsible for the higher level actions, like driving through an intersection or performing a lane change. This file describes the detailed process of performing the behavior.

---

## Author

Julius Miller, Robert Fischer

## Date

31.03.2023

---
<!-- TOC -->
- [Behaviors detailed](#behaviors-detailed)
  - [Author](#author)
  - [Date](#date)
  - [Intersection](#intersection)
    - [Detect Intersection](#detect-intersection)
    - [Approach Intersection](#approach-intersection)
    - [Wait Intersection](#wait-intersection)
    - [Enter Intersection](#enter-intersection)
    - [Leave Intersection](#leave-intersection)
  - [Lane Change](#lane-change)
  - [Cruise](#cruise)
  - [Unstuck Behavior](#unstuck-behavior)
<!-- TOC -->

---

## Intersection

### Detect Intersection

Subscriber: Waypoint

A Intersection is detected if the distance from the current position to the waypoint
 is less than 30 meters and the waypoint is at an intersection.

### Approach Intersection

Subscribers: Traffic Light, Waypoint (Stopline), Speed

![ApproachIntersection](../00_assets/planning/Flow_app_intersection.png)

If there is a green traffic light the car will drive towards the intersection with a speed of 30 km/h.

In every other case the car stops at the intersection and reduces dynamically the speed calculated by [motion_plannnig.py](../code/planning/src/local_planner/motion_planning.py)

The target_distance is currently 5 meters.

![ForwardWait](../00_assets/planning/Flow_app_forwardwait.png)

Conditions:

1. Describes if the car is stopped at the intersection.
2. Drive through intersection even if the traffic light turns yellow
3. Running over line

### Wait Intersection

Subscriber: Traffic Light

![Wait](../00_assets/planning/Flow_app_wait.png)

### Enter Intersection

Subscriber: Waypoint

![Enter](../00_assets/planning/Flow_app_enter.png)

New Waypoint is selected in [global_plan_distance_publisher.py](../code/perception/src/global_plan_distance_publisher.py) if the current position is closer than 2.5 meters to the waypoint or the next waypoint is closer than the currrent waypoint

### Leave Intersection

Signals behavior is over.

## Lane Change

WIP

## Cruise

Default behavior. Motion Planning uses acc speed in this case.

## Unstuck Behavior

The Unstuck Behavior is triggered whenever we find ourselves in a stuck situation for some specified amount of time. This Behavior can overwrite any other behavior, since we can also get stuck waiting inside other behaviors.

The Unstuck Behavior works with two Timers:

1. ```Wait Stuck Timer```
2. ```Stuck Timer```

Whenever we are driving at some velocity > 0.1 the timers are reset.

If we get into a situation where the velocity <= 0.1 the timers start running.

We can then either trigger the Wait Stuck Timer condition (currently 50 secs)
after not moving for longer than that duration -> Wait Stuck

OR

we trigger the quicker Stuck Timer condition (currently 20 secs), after not moving for longer than that duration WHILE being told to move -> Stuck.

Once one of the Timers is triggered we do the following:

1. First time getting stuck in the same position:
   1. Start driving backwards without steering for 1.2 secs
   2. Stop
   3. Reset all timers and let other behaviors work again
2. Second time getting stuck in the same position:
   1. Start driving backwards without steering for 1.2 secs
   2. Stop
   3. Add ```Overtake``` trajectory from [motion_planning.py](/code/planning/src/local_planner/motion_planning.py)
   4. Reset all timers and let other behaviors work again
3. Every other time we only do the same as in **1.** to prevent us from overtaking twice or more

To check wether we got stuck at the same position we also save the last position we triggered the Unstuck Routine at.

Furthermore we publish an unstuck_flag so the motion.py knows not to trigger more than one overtake if we stay in the same position.

If True we won't overtake anymore, if False we are able to overtake again.

We also publish an ```unstuck_distance``` so the motion_planning knows where to start the overtake trajectory.

This behavior is ONLY implememted to improve route completion and make debugging easier. It is the last fallback behavior when every other planning component can not get us back on track.

Files influenced by this behavior are:

- Planning:
  - [motion_planning.py](/code/planning/src/local_planner/motion_planning.py), for the target_speed and overtake
  - [behavior_speed.py](/code/planning/src/behavior_agent/behaviours/behavior_speed.py), for the target_speed
- Acting:
  - [vehicle_controller.py](/doc/05_acting/04_vehicle_controller.md), because of driving backwards without steering
  - [velocity_controller.py](/doc/05_acting/02_velocity_controller.md), because of the sepcial -3 target_speed case
