# Lane Change Behavior

**Summary:** This file explains the Lane Change behavior.

- [General](#general)
- [Ahead](#ahead)
- [Approach](#approach)
- [Wait](#wait)
- [Change](#change)
- [Stop Marker Handling](#stop-marker-handling)

## General

This behavior executes a lane change and manages the transition between lanes in a safe and controlled manner.

It supports:

- early lane changes when the target lane is free
- stopping before the lane change point if necessary
- detecting if the vehicle is already on the desired lane (e.g. after an overtake)

If the lane change cannot be executed immediately, the vehicle slows down and eventually stops before the lane change point using a stop marker.

The behavior distinguishes between left and right lane changes and reacts accordingly.

## Ahead

Checks whether the next waypoint (`/paf/hero/current_waypoint`) indicates a lane change and initiates the lane change sequence.

When a lane change is detected ahead, a stop marker is inserted at the lane change position. This ensures that the vehicle does not enter the lane change area unchecked.

The stop marker prevents unsafe behavior by forcing the vehicle to stop before entering the target lane if the situation is not yet safe.

## Approach

Attempts to perform an early lane change while still driving on the current lane.

This is done using the `is_lane_free` function:

- If the lane is free → the trajectory is immediately planned to the target lane using `request_start_overtake()`
- If the lane is not free → the vehicle continues on the current lane while continuously rechecking

If the vehicle approaches the lane change point (`< TARGET_DISTANCE_TO_STOP_LANECHANGE`) and no lane change has occurred yet:

- the behavior transitions to **Wait**
- the vehicle is already slowed down and will stop due to the stop marker set in the Ahead phase

This phase is only active while the vehicle is still on the original lane.

## Wait

Waits at the lane change point until the lane is free.

- Uses `is_lane_free` continuously
- Keeps the vehicle stopped using the stop marker

This state is only entered if:

- the lane change could not be executed in the Approach phase
- the vehicle is still on the original lane

## Change

Executes the lane change once the target lane is free.

- The stop marker is removed
- The trajectory is executed towards the target lane

Since the lane is checked before execution, the maneuver is expected to be collision-free.

If the vehicle is already on the desired lane when entering this state:

- the state only ensures proper completion of the behavior

The lane change is considered complete when:

- the vehicle has moved more than 5 meters away from the original lane change position

## Stop Marker Handling

Stop markers used for lane changes are inserted into the intermediate layer map via the service:
`/paf/hero/mapping/update_stop_marks`

(Service type: `mapping_interfaces/srv/UpdateStopMarks`)

They are not published as a standalone topic. Instead, they are integrated into the map as virtual stop obstacles by the mapping system.

In the lane change behavior:

- a rectangular stop marker is placed at the lane change position during the **Ahead** phase
- this forces the vehicle to stop before entering the lane change area if the maneuver is not yet safe

The stop marker prevents:

- entering a blocked or occupied target lane
- executing a lane change too early
- unsafe interactions with traffic on the target lane

Once the lane is free and the lane change is executed:

- the stop marker is removed again

The stop marker is managed using a dedicated identifier (`lanechange`) and updated dynamically during the behavior execution.
