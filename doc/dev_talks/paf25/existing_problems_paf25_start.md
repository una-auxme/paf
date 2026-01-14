# Overview of the main issues of the simulation 
(as of: Sprint 2)

## Perception:
- Merging into traffic (right turn, highway)
  - Target behavior: Wait until cross traffic passes; then merge
  - Actual behavior: Merge without observing cross traffic\
  → Issue: Lateral detection

- Circumventing the construction site:
  - Target behavior: Detect oncoming traffic early enough; safely circumvent the obstacle without affecting oncoming traffic     
  - Actual behavior: Oncoming traffic is detected; overtaking attempt is still carried out \
→ Issue: Checkbox for overtaking too small (to check for oncoming traffic) 

- Changing lanes in the intersection
  - Target behavior: The currently used lane is kept when crossing the intersection
  - Actual behavior: Lane change into oncoming traffic \
→ Issue: Lane detection is faulty
 
## Planning:
- Ignoring oncoming traffic when turning left
  - Target behavior: Oncoming traffic should always be considered before turning left
  - Actual behavior: Oncoming traffic is sometimes ignored \
→ Issue: Before turning left, oncoming traffic is only checked once

- Overtaking stationary traffic
  - Target behavior: Wait with the rest of the traffic
  - Actual behavior: Stationary traffic is detected as an obstacle and overtaken \
→ Issue: No distinction is made between a parked and a waiting car

## Acting:
- Car does not drive all the way up to the stop line at intersections
