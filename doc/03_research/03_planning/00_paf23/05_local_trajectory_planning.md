# Local trajectory planning

**Summary:** Explanation on how the frenet trajectory planner is used inside the motion planning component.

---

## Author

Samuel Kühnel

## Date

23.01.2024

## Position calculation from Obstacle

The position from a possible obstacle that we need to overtake is calculated via the current heading from the ego vehicle, the current position and the measured distance. The 