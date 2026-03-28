# Intersection Behavior

**Summary:** This file explains the Intersection behavior.

- [General](#general)
- [Intersection ahead](#intersection-ahead)
- [Approach](#approach)
- [Wait](#wait)
- [Enter](#enter)
- [Cross Traffic Detection](#cross-traffic-detection)
  - [Emergency Handling](#emergency-handling)
  - [Current Limitations](#current-limitations)
- [Parameters](#parameters)

## General

The intersection behavior is used to control the vehicle when it encounters an intersection. It handles stop signs as well as traffic lights.
The intersection sequence consists of the sub-behaviors "Approach", "Wait", and "Enter".

To enter the intersection sequence "Intersection ahead" must firstly be successful.

## Intersection ahead

Successful when there is an intersection ahead which is extracted from the next waypoint.

Sets a stopmarker as a virtual stopline.

Tries to stay as long as possible in overtake behavior if currently overtaking before switching.

## Approach

Handles approaching the intersection.

Returns RUNNING while still far from the intersection, SUCCESS when the vehicle has stopped at a stopline or has already entered the intersection.

Removes the stopline stopmarker in case of a green light.

In case of a left turn a stopmarker is created on the left lane relative to the car so that when the car is driving over the stopline it stops shortly afterwards.

## Wait

Handles wating at the stop line until the vehicle is allowed to drive.

Waits 0.5 seconds at a stopline without a traffic light.

In case of a left turn oncoming traffic is checked after driving over the stopline. This is based on a counter to avoid inconsistencies.

The map function is_lane_free_intersection(...) for checking the oncoming traffic utilizes a big polygon to cover two lanes and only regards traffic driving towards the car.

Returns SUCCESS when allowed to drive otherwise RUNNING.

## Enter

Handles driving through the intersection.

In case of driving straight a low speed limit is set for ACC to avoid emergency vehicles. This is a temporary solution.

Returns FAILURE to end the intersection behavior once the current intersection waypoint is far enough behind the car.

## Cross Traffic Detection

Oncoming and cross traffic is evaluated using dynamic entities from the map.

A rectangular check area is created in front of and across the intersection. All entities within this area are retrieved and analyzed.

Only entities with motion information are considered. Their velocity is evaluated to determine whether they pose a risk.

Fast moving objects (e.g. cross traffic) are detected based on a velocity threshold and can block the intersection.

A velocity threshold is used to filter relevant traffic. Static or slow-moving objects are ignored to reduce false positives.

### Emergency Handling

If fast cross traffic is detected while the ego vehicle is still moving above a certain speed, an emergency signal is triggered.

This signal is published to notify about a potentially dangerous situation.

### Current Limitations

The current cross traffic check does not yet consider the motion direction of detected objects.

As a result, the vehicle may brake whenever an object inside the check area moves above the configured speed threshold, even if that object is moving away from the ego vehicle and does not actually pose a risk.

In addition, the rectangular check area currently rotates together with the ego vehicle. This can affect the relevance of the checked region during turning maneuvers.

The current check should therefore be understood as a conservative safety mechanism.

In the long term, this check may become less relevant or unnecessary if the collision check becomes sufficiently reliable.

## Parameters

The following parameters are used for cross traffic detection:

- CROSS_TRAFFIC_SPEED_THRESHOLD: Minimum speed for detecting relevant cross traffic (2.5 m/s)
- CROSS_CHECK_DISTANCE: Distance in front of the ego vehicle used for checking traffic (15.0 m)
- CROSS_CHECK_LENGTH / WIDTH: Size of the rectangular check area (25.0 m / 50.0 m)
- PRIORITY_SPEED_THRESHOLD: Speed threshold for prioritizing traffic (25.0/3.6 m/s ≈ 6.94 m/s)
- PRIORITY_CHECK_DISTANCE: Distance for priority traffic detection (13.0 m)
- PRIORITY_CHECK_LENGTH / WIDTH: Size of the priority check area (25.0 m / 50.0 m)
- SELF_EMERGENCY_THRESHOLD: Ego speed above which emergency handling is triggered (10.0/3.6 m/s ≈ 2.78 m/s)
