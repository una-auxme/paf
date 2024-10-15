# Requirements of Carla Leaderboard

**Summary:** This page contains the project informations from the CARLA leaderboard. More specific summary after page is finished.

- [Requirements of Carla Leaderboard](#requirements-of-carla-leaderboard)
  - [Task](#task)
  - [Participation modalities](#participation-modalities)
    - [Route format](#route-format)
    - [Sensors](#sensors)
  - [Evaluation](#evaluation)
    - [Main score](#main-score)
    - [Driving score for single route](#driving-score-for-single-route)
    - [Infraction penalty](#infraction-penalty)
    - [Shutdown criteria](#shutdown-criteria)
  - [Submission](#submission)
    - [Sources](#sources)

## Task

- an autonomous agent should drive through a set of predefined routes
- for each route:
  - initialization at a starting point
  - directed to drive to a destination point
  - route described by GPS coordinates **or** map coordinates **or** route instructions
- route situations:
  - freeways
  - urban areas
  - residential districts
  - rural settings
- weather conditions:
  - daylight
  - sunset
  - rain
  - fog
  - night
  - more ...

Possible traffic signs (not complete):

- Stop sign
- Speed limitation
- Traffic lights
- Arrows on street
- Stop sign on street

## Participation modalities

The two modes are **SENSORS** and **MAP**. They differ in the type of input data the agent can request from the platform.

### Route format

Routes consist of tuples of a position and a high level command which should be taken at that point.

There are two different ways to express a route:

First, GPS coordinates, a z component and a route option

```yaml
[({'z': 0.0, 'lat': 48.99822669411668, 'lon': 8.002271601998707}, RoadOption.LEFT),
 ({'z': 0.0, 'lat': 48.99822669411668, 'lon': 8.002709765148996}, RoadOption.RIGHT),
 ...
 ({'z': 0.0, 'lat': 48.99822679980298, 'lon': 8.002735250105061}, RoadOption.STRAIGHT)]

```

Second, world coordinates and a route option

```yaml
[({'x': 153.7, 'y': 15.6, 'z': 0.0}, RoadOption.LEFT),
 ({'x': 148.9, 'y': 67.8, 'z': 0.0}, RoadOption.RIGHT),
 ...
 ({'x': 180.7, 'y': 45.1, 'z': 1.2}, RoadOption.STRAIGHT)]

```

**Important:** Distance between route points can be up to hundreds of meters.

High-level commands (rood options) are:

- RoadOption.**CHANGELANELEFT**: Move one lane to the left.
- RoadOption.**CHANGELANERIGHT**: Move one lane to the right.
- RoadOption.**LANEFOLLOW**: Continue in the current lane.
- RoadOption.**LEFT**: Turn left at the intersection.
- RoadOption.**RIGHT**: Turn right at the intersection.
- RoadOption.**STRAIGHT**: Keep straight at the intersection.

**Important:** If the semantics of left and right are ambiguous, the next position should be used to clarify the path.

**Important**: You are not allowed to make use of any privilege information offered by the CARLA simulator, including planners or any type of ground truth. Submissions using these features will be rejected and teams will be banned from the platform.

### Sensors

| Name | CARLA Doc | Quantity |
| --- | --- | --- |
| GNSS| [sensor.other.gnss](https://carla.readthedocs.io/en/latest/ref_sensors/#gnss-sensor) | 0 - 1 units|
| IMU | [sensor.other.imu](https://carla.readthedocs.io/en/latest/ref_sensors/#imu-sensor) | 0 - 1 units|
| LIDAR | [sensor.lidar.ray_cast](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor) | 0 - 1 units|
| Radar | [sensor.other.radar](https://carla.readthedocs.io/en/latest/ref_sensors/#radar-sensor) | 0 - 2 units |
| RGB camera | [sensor.camera.rgb](https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera) | 0 - 4 units|
|Speedometer| sensor.other.speedometer (not in docu) | 0 - 1 units|
| OpenDRIVE map (only in MAP mode)| sensor.opendrive_map (not in docu) | 0 - 1 units|

## Evaluation

Determination how "good" the agent performs on the Leaderboard.

### Main score

The driving proficiency of an agent can be characterized by multiple metrics.

- `Driving score:` Product between route completion and infractions penalty
- `Route completion:` Percentage of the route distance completed by an agent
- `Infraction penalty:` The leaderboard tracks several types of infractions which reduce the score

Every agent starts with a base infraction score of 1.0 at the beginning.

Complete score is arithmetic mean of all routes driving scores.

### Driving score for single route

Product of route completion and infraction penalty of this route

### Infraction penalty

Not complying with traffic rules will result in a penalty. Multiple penalties can be applied per route. Infractions ordered by severity are:

- collisions with pedestrians: 0.50
- collisions with other vehicles: 0.60
- collisions with static elements: 0.65
- running a red light: 0.70
- running a stop sign: 0.80

It is possible that the vehicle is stuck in some scenario. After a timeout of **4 minutes** the vehicle will be released, however a penalty is applied

- scenario timeout (feature behaviours can block ego vehicle): 0.70

Agent should keep a minimum speed compared to the nearby traffic. The penalty is increases with the difference in speed.

- Failure to maintain minimum speed: 0.70

Agent should let emergency vehicles from behind pass.

- Failure to yield to emergency vehicle: 0.70

If the agent drives off-road that percentage does not count towards the road completion

- Off-road driving: not considered towards the computation of the route completion score

### Shutdown criteria

Some events will interrupt the simulation of that resulting in an incomplete route

- route deviation - more than 30 meters from assigned route
- agent blocked - if agent does not take an action for 180 seconds
- simulation timeout - no client-server communication in 60 seconds
- route timeout - simulation takes too long to finish

## Submission

5 submissions per month

---

### Sources

<https://leaderboard.carla.org/>
