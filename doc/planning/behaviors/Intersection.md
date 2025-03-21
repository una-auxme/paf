# Intersection Behavior

**Summary:** This file explains the Intersection behavior.

- [Intersection Behavior](#intersection-behavior)
  - [General](#general)
  - [Intersection Ahead](#intersection-ahead)
  - [Approach](#approach)
  - [Wait](#wait)
  - [Enter](#enter)

## General

The Intersection behaviour is used to control the vehicle when it encounters a intersection. It handles stop signs as well as traffic lights.
The Intersection sequence consists of the sub-behaviours "Approach", "Wait", and "Enter".

To enter the Intersection sequence "Intersection ahead" must firstly be successful.

## Intersection ahead

Successful when there is a intersection ahead which is extracted from the next waypoint.

Sets a stopmarker as a virtual stopline.

Tries to stay as long as possible in overtake behavior if currently overtaking before switching.

## Approach

Handles approaching the intersection.

Returns RUNNING while still far from the intersection, SUCCESS when the vehicle has stopped at a stopline or has already entered the intersection.

Removes the stopline stopmarker in case of a green light.

In case of a left turn a stopmarker is created on the left lane of the car so that when the car is driving over the stopline it stops shortly afterwards.

## Wait

Handles wating at the stop line until the vehicle is allowed to drive.

Waits 2 seconds at a stopline without a traffic light.

In case of a left turn oncoming traffic is checked after driving over the stopline. This is based on a counter to avoid inconsistencies.

The map function for checking the oncoming traffic utilises a big polygon to cover two lanes and only regards traffic driving towards the car.

Returns SUCCESS when allowed to drive otherwise RUNNING.

## Enter

Handles driving through the intersection.

In case of driving straight a low speed limit is set for ACC to avoid emergency vehicles. This is a temporary solution.

Returns FAILURE to end the intersection behavior once the current intersection waypoint is far enough behind the car.
