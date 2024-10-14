# Sprint 2

## General ToDo’s

- Test Submit for new Leaderboard {Samuel, Max}
  - Submission Page stating what can be left in the code
  - Build Docker Image with Environment Variables
  - Make Docker Script
  - (info: 99 submissions per month possible)

## Perception

- dvc {Max, (Leon)}
  - Look more closely
  - Solution approach for Large File Storage
- Clean Up {Leon}
  - Throw out EfficientPS
    - If thrown out → remove dependencies for Image (BACKLOG)
- New Segmentation Node {Leon, Max}
  - Visualization in RViz
  - Baseline Model (Object Detection / Segmentation)
  - General procedure
- Sensor Data Filter {Robert}
  - Is there any filtering at all?
  - Filtering of data (Maybe the sensor noise can be removed in Sensorconfig for tests)
    - Sensorfile
    - Without Leaderboard → as many sensors as you want
    - Read out values without noise via Python API

## Planning

- Understanding of current Decision Tree {Samuel}
  - [ ]  Does the global routing by the Decision Tree work?
  - [ ]  Are lane decisions correct? (holding lane, switching lanes)
  - [ ]  How well does it perform
  - [ ]  Limitations?
- Visual Architecture {Julius}
- What is currently output {}
- What is currently arriving {}
  - What should arrive?
- Object Avoidance [BACKLOG] {}
  - Must-Have for next milestone
  - For passing obstacles

## Acting

- Simpler design of Acting
  - Maybe fewer controllers
  - Maybe alternative architecture
- Measure rule deviation
  - Ideal Dummy Sensors (Python API)
  - With ideal actual state
- Make analysis possible
- For testing
  - Turn off all vehicles
  - Maybe use Trajectories Dummy for this (Gabriel as contact person)
- Emergency Breaks
- Parking out scenario → currently without sensor input → only Acting / Hardcoded

## Notes

- Relative speed of objects published by perception via radar
- Documentation of issues during presentations
- Github file limit of 100MB
- Define interfaces
