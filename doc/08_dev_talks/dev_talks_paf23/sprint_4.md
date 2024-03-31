# Sprint 4

## General ToDo's

- [ ] Fix Submission {Samuel, Max}

## Perception

- [ ] LIDAR {Leon}
  - [ ] Calculate distance to each detected object
    - [ ] Define exactly to which point the distance is calculated
  - [ ] Reference coordinate system (maybe at the bumper)
- [ ] Get CUDA running {Leon}
- [ ] Traffic Light Node {Max}
  - [ ] Create new publisher
  - [ ] Detect orientation of the traffic light
    - [ ] Check relevance
- [ ] Complete and test Kalman Filter {Robert}
  - [ ] Position
  - [ ] Heading
- [ ] Document research on other filtering options if not accurate enough {Robert}
  - [ ] Maybe Monte Carlo
  - [ ] Maybe also implement directly

## Planning

- [ ] Collision Check needs to be converted into a node
- [ ] Motion planning missing
- [ ] Implement ACC
- [ ] Change publishers (only publish name of task)

## Acting

- [ ] Velocity Controller {Alex}
  - [ ] Different step responses
  - [ ] Follow-up control
  - [ ] Braking
  - [ ] Approx. 1 m/s as maximum deviation
- [ ] Steering {Alex, Robert}
  - [ ] Check implementation
  - [ ] PP Waypoint/ Trajectory Vector
- [ ] Check Emergency Brake [BACKLOG] {Robert}
