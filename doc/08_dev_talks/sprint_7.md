# Sprint 7

Next presentation in 6 weeks!

Merge and review during the sprint!

## General ToDo's

- [ ]  Fix CI or merge without fix

## Perception

- [ ]  Maybe use the position of the center of the bounding boxes as the position of the objects
- [ ]  Recognize fire truck
- [ ]  Fix Vision Node delay on Leaderboard
  - [ ]  (Vision node looks different on the leaderboard than in Dev Launch)
- [ ]  (Kalman Filter Bug)
- [ ]  Fix Lidar error logs!
  - [ ]  Logs are spammed :(
- [ ]  Add axis position?
- [ ]  Traffic Light → where do you have to stop to still see the traffic light

## Planning

- [ ]  Integration with new Perception features
- [ ]  More testing / tuning
- [ ]  If the object to be overtaken is longer than expected
  - [ ]  if lateral detection is available
  - [ ]  plan new trajectory
- [ ]  (Merge Overtaking)
- [ ]  Maybe make Lane Change "Sloppy"
  - [ ]  Maybe make the curve for lane changing flatter
- [ ]  At intersection 2 wait until fire truck passes so the rest can be showcased
- [ ]  Filter out which cars are on which lane
  - [ ]  (Are class 2 cars?)

## Acting

- [ ]  (Clean up Acting)
- [ ]  (Write proper documentation)
- [ ]  For Stanley → throw out
- [ ]  Tune PurePursuit for higher speeds
- [ ]  Add axis position (towards Heading)

## Notes

### Perception*

- [ ]  Next Steps:
  - [ ]  Also pass Lidar "Heading"
  - [ ]  Calculate distance to traffic lights
  - [ ]  Vision Node is heavily delayed on Julius Branch
    - [ ]  Traffic light detection is also delayed
    - [ ]  → IMPORTANT
  - [ ]  Kalman Filter Bug

### Planning*

- [ ]  Overtaking:
  - [ ]  NICE
- [ ]  Curve speed:
  - [ ]  Maybe theoretically solve with curve curvature
  - [ ]  NICE!
- [ ]  Maybe make Lane Change "Sloppy"
  - [ ]  Maybe make the curve for lane changing flatter
- [ ]  Next Steps:
  - [ ]  Integration with new Perception features
  - [ ]  More testing / tuning
  - [ ]  If the object to be overtaken is longer than expected
  - [ ]  Plan new trajectory
  - [ ]  Merge Overtaking

### Acting*

- [ ]  Stanley
  - [ ]  Correctly implemented
  - [ ]  Does not work better than the PurePursuit
  - [ ]  Maybe not implement PurePursuit linearly
  - [ ]  Throw out Steering PID
- [ ]  Next Steps:
  - [ ]  Clean up Acting
  - [ ]  Write proper documentation
  - [ ]  For Stanley → Calculate front axle position
  - [ ]  For PurePursuit → Calculate rear axle position

### Misc*

- [ ]  Performance improvement
  - [ ]  40 hz
  - [ ]  More CPU performance
- [ ]  New dev launch possibility
  - [ ]  Autopilot of the NPC vehicles
  - [ ]  Change of the Spectator perspective
  - [ ]  Configurable start position
- [ ]  CI optimizations
  - [ ]  Less log spam
  - [ ]  Update of the Simulator Image to Leaderboard 2.0
- [ ]  NICE
- [ ]  Next Steps:
  - [ ]  Expand Dev Route
  - [ ]  Revise milestones
