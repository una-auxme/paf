# Known Errors

This file contains a list with all known errors from the project iteration PAF24
The list is generated during the final sprint of the semester,
thus some issues might have been addressed by a hotfix
if you are reading this as a future participant of the PAF project.

## Table of content

- [Table of content](#table-of-content)
- [List of known errors](#list-of-known-errors)
  - [Route 1](#route-1)
  - [Route 2](#route-2)

## List of known errors

The impact is a ranking on how badly the Error impacts the performance of the car during a run.
A high impact means the car cant recover most of the time, or the run is ruined in another way.
A medium impact means the car won't get stuck, but gets a penalty affecting the score.
A low impact means the car is not behaving correctly, but it doesn't get punished for it.

----

### Route 1

| Error  | description  | impact | fixed |
|---------|--------------|---------|----------|
| Ghost Entities | At least one entity-publishing node is running on an increasing time delay, causing ghost entities to appear in the intermediate layer. This leads to multiple issues| High | No |
| Firetruck crash | The firetruck hits us | High |  No |
| Overtake | Reentering the lane after an overtake-routine takes a lot of time, which leads to a long distance driven in the oncoming lane | Low |  No |
| No lane free check for lane changes | The lane_free is not checked for the lane change after the bicycles | Medium |  No |
| Losing vision of traffic light | Getting to close to the stop line causes us to lose vision of the traffic light above the car | Medium |  No |
| Pedestrian hit | Pedestrian hit in the intersection due to delays in intermediate layer | Medium |  No |
| Overtake waiting | While waiting for a free lane during overtake, we slowly approach the obstacle in front of us to a point that we can't overtake it anymore. | High | No |
| Overtake crash | Crash into oncoming traffic at the parking car due to delays in intermediate layer | High |  No |
| Construction sign crash | Crash into construction sign due to delays in intermediate layer | High |  No |
| Crash when entering the highway | No check for a free lane when entering the highway | medium |  No |

----

### Route 2

| Error  | description  | impact | fixed |
|---------|--------------|---------|----------|
| Parking | No check for free lane when leaving the parking spot. | High | No |
| Intersection | No check for the traffic in the intersections | High | No |
| Overtake at intersection | Check for possible overtake at the second intersection, while car in front is waiting for green lights| Low | No |
| Construction sign |  Detecting the construction sign to late | High | No |
| Construction sign overtake | Offset of checkbox for lane-free check is too far in the front, so we never start the routine| High | No |
| Construction sign |  Detecting the construction sign to late | High | No |
| No lane free check for lane changes | The lane_free is not checked for the lane change| Medium |  No |
| Losing vision of traffic light | Getting to close to the stop line causes us to lose vision of the traffic light above the car | Medium |  No |
| Bad left turn | When turning left at an intersection we are too fast and crash into a sign | Medium | No |
| Pedestrian | Hitting the kid appearing behind the container | Medium | No |
