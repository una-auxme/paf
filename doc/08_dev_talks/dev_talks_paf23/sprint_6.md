# Sprint 6

## General ToDo's

- [ ]

## Perception

- [ ] Display XYZ values in RGB
  - [ ] Instead of visualization → Array
- [ ] Test rear view camera
- [ ] Interface between Local Planning and Lidar
- [ ] Vehicle detection from the side
- [ ] Lane Detection → Coordination with Planning [BACKLOG]

## Planning

- [ ] Trajectory for overtaking
  - [ ] From when do you actually have to overtake
  - [ ] Recognize obstacles in the trajectory
  - [ ] How long do you have to overtake
    - [ ] Update during the drive
  - [ ] Stationary objects as the target of the overtaking process
- [ ] Behaviour
  - [ ] Recognize oncoming traffic

## Acting

- [ ] Emergency Brake Fix
  - [ ] Strong lag in leaderboard version
  - [ ] Maybe without bug abuse → Max Brake
- [ ] Stanley:
  - [ ] Better suited (sources) for higher speeds, tuning for these speeds in the next sprint.
- [ ] Vehicle Controller:
  - [ ] If Stanley has also been tuned, question, revise and hopefully improve the sigmoid function which decides which of the Steering-Controller-Inputs is considered (and especially the mixed form of both controller inputs at the transition limit).

## Notes

### Perception*

- [ ] LIDAR:
  - [ ] Position from Lidar fixed
  - [ ] Flickering
    - [ ] Low spin rate → more flickering / more data
    - [ ] High spin rate → less flickering / less data
    - [ ] 10 → best resolution
    - [ ] 15 → sometimes no flickering
    - [ ] 20 → no flickering, poor resolution
    - [ ] → 15
  - [ ] Distances from objects are determined
- [ ] Traffic light detection:
  - [ ] YOLO RTDETR
  - [ ] 5 states, only 3 published
  - [ ] WORKS GREAT!!
- [ ] CUDA workaround:
  - [ ] Ubuntu 22 Bib
  - [ ] PyTorch 2 usable!
- [ ] Next Steps:
  - [ ] Currently only functional with yolo models → adapt to other models
  - [ ] Display XYZ values in RGB
    - [ ] Instead of visualization → Array
  - [ ] Test rear view camera
  - [ ] Interface between Local Planning and Lidar
  - [ ] → Testing necessary!
  - [ ] Attack test submit again as soon as available
  - [ ] Lane Detection → Coordination with Planning [BACKLOG]

### Planning*

- [ ] Planning Package restructured
  - [ ] Only one package now
- [ ] ACC rework
- [ ] Collision Check Bug:
  - [ ] Was very laggy
  - [ ] FIXED
- [ ] Leave Parking Spot behaviour
  - [ ] Flag that you can only park out once
- [ ] WE DRIVE STRAIGHT AHEAD WELL WOHOOOO
- [ ] Emergency brake → vehicle controller
  - [ ] Simulator freezes
- [ ] We get over intersections yipiiiii
- [ ] Pylot → Implement Frenet Trajectory Planner for us
- [ ] Carla Route in Dev Launch
  - [ ] Advantage:
    - [ ] Waypoints for intersections
    - [ ] Possible to drive around the map once without obstacles
  - [ ] RouteOptions documentation in preplanning.md
- [ ] NEXT STEPS
  - [ ] Fix Emergency Brake
  - [ ] Local planning around objects
    - [ ] Might be enough to avoid and then when you are next to the object calculate the trajectory again
  - [ ] Maybe use middle lane as a limit when parking out
  - [ ] Maybe wait a little longer before parking out, because of wrong coordinates
  - [ ] Test with improved traffic light detection
  - [ ] Add check if intersection is free
  - [ ] Expand curve detection

### Acting*

- [ ] Pure Pursuit:
  - [ ] Low V
  - [ ] Lookahead d calculated from v
  - [ ] Look in the heading of the car on trajectory → calculate point
    - [ ] Calculate vector → angle → steering angle
  - [ ] Work:
    - [ ] Simplified
    - [ ] Min and maxdistance for lookahead set
    - [ ] Always swings when driving straight :/
    - [ ] At 10m/s very swinging → stanley
    - [ ] Finished for now in linear form up to 40 kmh it works quite well
- [ ] Stanley :
  - [ ] Higher V
  - [ ] Always looks at next point on trajectory
    - [ ] Target value from position and heading
    - [ ] Trajectory Heading also interesting if possible
  - [ ] Work:
    - [ ] Disabled → Tuning in the next sprint
- [ ] Sigmoid:
  - [ ] Switch between both from 4 m/s
- [ ] PID Controller still controls steering
  - [ ] Necessary?
- [ ] Tune variable -5 on steering left out for now because probably unnecessary
