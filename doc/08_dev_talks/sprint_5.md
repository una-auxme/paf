# Sprint 5

## General ToDo's

- [ ] Merge PR
- [ ] Fix Cuda Errors [Backlog]
- [ ] Minimum Distance Publisher

## Perception

- [ ] Merge Lidar and Vision Nodes {Leon}
- [ ] Lidar Flickering {Leon}
- [ ] Heading Bug Fix {Robert}
  - [ ] Transformation to correct Yaw
  - [ ] How is the Heading used in the rest of the code
- [ ] Kalman Filter tuning {Robert}
  - [ ] Heading
  - [ ] Position
  - [ ] Publishing the data
- [ ] Additional cameras
- [ ] Traffic Light Detection
  - [ ] Detect traffic lights that are outside

## Planning

- [ ] Continue publishing ACC Node → Ignore Emergency Break
- [ ] Ignore red traffic lights if already in intersection [Backlog]
  - [ ] → Test
- [ ] Rebuild Planning Directory
- [ ] Verify behaviours
  - [ ] Build test node
  - [ ] Also test in leaderboard if test node works

## Acting

- [ ] Test Emergency Breaks
  - [ ] Via Testnode
- [ ] Steering Tuning, Testing
  - [ ] First Pure Pursuit (easier to tune)
  - [ ] Then maybe Stanley

## Notes

Perception:

- [ ] Limit Lidar data up and down (Z axis)
  - [ ] Or merge multiple images
  - [ ] Change Lidar Config because of flickering
- [ ] Merge Lidar and Vision Nodes
- [ ] Vision Node:
  - [ ] Instead of yolov8x seg → rtdetr-x so you can see far enough for traffic lights
- [ ] Traffic Light Detection:
  - [ ] Extension of traffic light detection classes:
    - [ ] Backside, Green, Yellow, Red, Side
  - [ ] Publisher:
    - [ ] Traffic_light_state
    - [ ] Four states:
      - [ ] 3 Colors + Unknown
  - [ ] Red traffic lights are detected from the wrong side when turning
    - [ ] → Planning has to work with this case
- [ ] Tuned Kalman filter is amazing!!!
  - [ ] Is published as kalman_pos
  - [ ] Should replace current_pos
- [ ] Heading Bug
  - [ ] Dirty fix
  - [ ] KalmannHeading is running
- [ ] Kalman filter is as accurate as the old filter
- [ ] YAAAAAAAAAAAAWWWW!
- [ ] We now have a filter for the Heading, which we didn't have before

Planning:

- [ ] At np.inf from collision check → no Collision
- [ ] Fully integrate ACC and collision check + testing
- [ ] Probably some adjustments
- [ ] Remove ACC in consultation with Acting
- [ ] ACC node should continue to publish messages even if full braking is present
- [ ] Build speed calculation not linear
  - [ ] → Brake harder the closer
- [ ] Maybe ACC with target acceleration instead of target speed
- [ ] ACC PID controller has been removed
- [ ] Local Planning:
  - [ ] Cleaned
  - [ ] Rebuild Planning Directory
  - [ ] Verify behaviours
    - [ ] Build test node
    - [ ] Also test in leaderboard if test node works

Acting:

- [ ] Velocity_Controller
  - [ ] Apply Integral Clipping only at higher or lower values

Misc:

- [ ] Extra msg folder that needs to be compiled first
  - [ ] → Then every package can recognize the messages
- [ ] Carla_Manual_Control → agent_manual.launch
  - [ ] Press P key
- [ ] Memory Leak:
  - [ ] Swap-File in Linux as workaround (approx. 10 GB per minute)
  - [ ] Limitation of RAM for the simulator
