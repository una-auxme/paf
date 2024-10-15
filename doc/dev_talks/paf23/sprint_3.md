# Sprint 3

## General ToDo's

- Leaderboard Submission (when unlocked)
- Fix steering when parking out {Samuel, Robert}
- Update Cuda or Torch version

## Perception

- Segmentation Node {Leon}
  - Speed
  - Segmentation Mask
  - Yolo Models
  - Tensorflow
- Agree on interfaces (Issue already created)
  - From Perception to Planning:
    - GPS
    - Speed (Carla Sensor)
- LIDAR research {Leon}
  - possible combination with Object Detection
- Possible use of Radar: {}
  - In combination with LIDAR for Emergency Break
  - Relative speed
- Filter out current position more accurately {Robert}
  - maybe Kalman Filter
  - in relation to which point?
  - for what exactly?
- Check if and how much noise is measured by the sensors {Robert}
- Location Error Bugfix {Robert}
- Canny-Edge-Detection [BACKLOG]  {Max}
- Traffic Lights [BACKLOG → after Object Detection] {Max}
  - what's already there?

## Planning

- Collision Check
- ACC
  - in behavior tree or not?
  - maybe look at others
- Connection Local Planner & Behavior Tree
-

## Acting

- maybe test ACC
- Finish building test case
  - push-bar
- Tuning of the controllers

## Notes

- Segmentation Node with all cameras maybe
- Camera to the back can look over cars
  - → you see the road when parking out
