# Current state of the simulation

**Summary:** The current state of the simulation is assessed by doing three runs of 20 mins (real world time), where all mistakes or anomalies are written down.

- [Goal](#goal)
- [Methodology](#methodology)
- [Observed Errors Grouped by Domains](#observed-errors-grouped-by-domains)
  - [Infrastructure](#infrastructure)
  - [Testing and Validation](#testing-and-validation)
  - [Perception](#perception)
  - [Localization and Mapping](#localization-and-mapping)
  - [Decision-Making](#decision-making)
  - [Path Planning](#path-planning)
  - [Control](#control)
- [Raw notes](#raw-notes)
  - [Run 1](#run-1)
  - [Run 2](#run-2)
  - [Run 3](#run-3)

## Goal

In order to understand the current state of the agent, it is crucial to assess the status quo and note the challenges it is faced with.

## Methodology

This assessment was done by three leaderboard runs in the CARLA simulator with the handover-state for PAF24. While doing so, all mistakes made by the agent have been noted, as well as possible anomalies occurring during the inspection.
After the review, the mistakes have been grouped by the roles defined in the project in order to make it easier to address the challenges in the respective domains. **Note:** Some mistakes overlap and communication is key when tackling these issues.

## Observed Errors Grouped by Domains

### Infrastructure

These issues relate to foundational aspects of the simulation environment and underlying software stability:

- **Simulator Performance Degradation:**
  - Simulation slows down over time (from .33 to .29 rate), potentially impacting reaction times and sensor data processing.
- **Vehicle Despawning:**
  - Random despawning of cars and potential timeout for stuck vehicles may interfere with the agent’s perception and response.

---

### Testing and Validation

These errors highlight the gaps in the testing and validation process, particularly areas that may need further testing to ensure proper functioning in the real environment:

- **Consistency in Object Detection:**
  - Image segmentation flickering (e.g., police car with indicators), suggesting inadequate validation for dynamic objects with flashing lights.
- **Vision Node Stability:**
  - Vision node appears to freeze occasionally, indicating possible untested scenarios or bugs in the perception pipeline.
- **Unrealistic Emergency Braking and Recovery Testing:**
  - Unstable lane holding and recovery, resulting in inappropriate emergency braking maneuvers, suggests insufficient validation in complex recovery scenarios.
- **Misclassification of Tree Trunks:**
  - Trees being detected as cars, indicating the need for validation of object detection in diverse environmental conditions.

---

### Perception

Errors within perception involve how the agent senses and understands its surroundings:

- **Object Misclassification and Collision:**
  - Tree trunks mistakenly detected as cars.
  - Crashes into bikers and parked cars, suggesting perception failures in identifying and avoiding static and moving obstacles.
- **Segmentation and Detection Instability:**
  - Vision node freezing.
  - Flickering segmentation for objects like police cars with indicators.
- **Lane Detection and Holding Errors:**
  - Difficulty in stable lane holding, leading to unexpected lane deviations and emergency braking.
  - Misinterpretation of open car doors, causing lane intrusions without sufficient clearance.

---

### Localization and Mapping

Issues with localization and mapping involve understanding and positioning within the environment:

- **Positioning Errors in Turns:**
  - Turns are too wide, leading the agent onto the walkway, indicating potential localization issues in tight maneuvers.
- **Lane Holding and Position Drift:**
  - Unstable lane holding with constant left and right drifting suggests potential mapping or localization inaccuracies.
  
---

### Decision-Making

Errors in decision-making relate to the agent's ability to make appropriate choices in response to various scenarios:

- **Right of Way Violations:**
  - Fails to yield to oncoming traffic when turning left and when merging into traffic.
  - Ignores open car doors when passing parked cars, causing dangerous close passes.
- **Erroneous Stopping and Acceleration:**
  - Stops unnecessarily at green lights and struggles to resume smoothly after stopping.
  - Abrupt stopping and starting at green lights, potentially due to aggressive speed control.
- **Repeated Mistakes in Overtaking and Lane Changes:**
  - Treats temporary parked cars as regular vehicles to overtake without checking oncoming traffic, leading to unsafe lane changes.

---

### Path Planning

Path planning issues include errors in determining the correct and safest path:

- **Incorrect Overtaking Paths:**
  - Attempts to overtake trees and temporary parked cars without considering oncoming traffic, showing flaws in path generation.
- **Wide Turning Paths:**
  - Takes overly wide turns that lead to walkway intrusions.
- **Aggressive Lane Changes:**
  - Lane change planning is overly aggressive, causing the vehicle to abruptly veer, triggering emergency stops to avoid collisions.

---

### Control

Control-related issues concern the vehicle’s execution of planned actions, like maintaining speed and stability:

- **Abrupt and Aggressive Speed Control:**
  - Speed controller is too aggressive when accelerating from green lights, leading to abrupt stopping and starting.
- **Instability in Lane Holding:**
  - Inconsistent lane holding, particularly after getting unstuck, results in unexpected deviations onto walkways.
- **Inconsistent Recovery Behavior:**
  - Repeatedly gets stuck in various situations (e.g., speed limit signs or temporary parked cars) and fails to recover smoothly, indicating control issues in re-engaging the driving path.

---

## Raw notes

Here are the raw notes in case misunderstandings have been made when grouping the mistakes

### Run 1

[Link to Video of the Run](https://drive.google.com/file/d/1Hb4arEC5ocfUD2KZh1bNVwRG2hoML8E9/view?usp=drive_link)

- Scared to get out of parking spot
- lane not held causing problems when avoiding open car door
- stopping for no apparent reason
- does not keep lane (going left and right)
- driving into still standing car at red light
- impatient when waiting for light to turn green (after the crash, going back and forth)
- abrupt stopping and going when light turns green without reason → speed controller too aggressive?
- Problems to keep lane is causing emergency(?) brake maneuvers
- vision node seems to be frozen ?
- Detects bikers, crashes into them nonetheless
- lane change very aggressive causing emergency stop in order to not go into oncoming traffic
- gets stuck as a result
- simulator despawns cars randomly
- left turn does not give way to oncoming traffic when seeing them
- does the turn too wide, gets onto walkway
- simulation gets slower as time progresses, started at .33 rate, now at .29
- gets stuck in front of speed limit sign after doing turn too wide
- gets unstuck, lane holding too aggressive goes onto walkway again (integrator windup while being stuck?)
- gets stuck again (→ unstuck behavior bad)
- when getting unstuck, merges onto street without giving way to traffic on the road
- drives into oncoming traffic, traffic on the same lane overtakes on the right side and does not stop
- really stuck now

### Run 2

[Link to Video of the Run](https://drive.google.com/file/d/1xot90LTNcSYOkLa1sXJJeMFBkXKmFO2x/view?usp=drive_link)

- merges without giving way to traffic
- does not respect open car door
- crashes into car in front when going after stop at red light
- stops at green light
- crashes into bikers
- kid runs onto street, agent crashes into oncoming traffic, gets stuck
- nudges away from the car it crashed into
- is now free but does not move
- crashes again
- police car with indicators on standing on the side is crashed into
- image segmentation for police car seems to be flickering
- tree trunk has bounding box (are trees detected as cars?)

### Run 3

[Link to Video of the Run](https://drive.google.com/file/d/1ERvN3nGddzuvtRqtIF2PKlFrcMzH2MrA/view?usp=drive_link)

- does not give way when exiting a parking spot
- LIDAR detects floor
- trajectory for overtaking is wrong / no overtake needed
- stops without reason
- tries to "overtake" tree (detects tree as car)
- playback ration temperature dependent likely
- after emergency brake stops too long
- left turn doesn't give way to oncoming traffic
- recovery leads to oncoming traffic (left turn situation maybe doesn't recognize street?) 9 min
- temporary parked car with indicators on counts as normal overtake (does not check oncoming traffic)
- temporary parked car with indicators is the crux
- Despawn time of cars ? Cars despawn when stuck → over time limit ?
- Trajectory correctly generated, just too deep in the mistakes
