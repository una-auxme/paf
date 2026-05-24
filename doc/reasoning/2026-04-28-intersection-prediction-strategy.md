# Intersection Prediction Strategy

## Task

Evaluate whether PAF should improve intersection handling with stronger motion prediction now, then implement the next useful vehicle-quality step.

## Local code inspected

- `code/planning/planning/behavior_agent/behaviors/intersection.py`
- `code/planning/planning/local_planner/motion_planning.py`
- `code/planning/test/test_intersection_priority_cross_traffic.py`
- `doc/planning/behaviors/Intersection.md`
- `doc/planning/motion_planning.md`

## External references checked

- Motion forecasting survey: <https://arxiv.org/abs/2502.08664>
- Autoware intersection module: <https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/>
- Autoware intersection collision checker: <https://autowarefoundation.github.io/autoware_universe/main/planning/planning_validator/autoware_planning_validator_intersection_collision_checker/>
- Autoware object collision estimator: <https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/object-collision-estimator.html>

## Main conclusion

The practical next step for PAF is not a heavier learned or multimodal motion predictor.

State-of-the-art motion forecasting depends on stronger tracking quality, confidence handling, lane context, and interaction modeling than the current PAF stack reliably provides.

The better fit is the production-style pattern seen in Autoware:

1. keep the prediction model simple and explicit,
2. use geometry plus TTC-style time margins,
3. add hysteresis to suppress stop/go chatter from noisy detections,
4. add a pass-judge or commit rule so the ego does not re-decide to stop after it is too committed to brake comfortably.

## What was implemented

- tightened priority cross-traffic filtering so fast vehicles that miss the conflict area do not trigger false stops,
- added a pass-judge rule in intersection `Enter`,
- added a simple TTC gate for `Enter` based on current tracked motion,
- added unsafe/safe hysteresis so noisy detections do not toggle the decision every cycle,
- refined the TTC gate to use the nearest conflict point inside the central priority corridor instead of the mask-center proxy, so edge overlaps do not trigger the same stop logic as a real path conflict.

## Why this is the right intermediate step

- It improves vehicle behavior quality immediately.
- It uses signals the stack already has: tracked position and velocity.
- It does not pretend the current perception can support reliable high-order prediction.
- It keeps the door open for later upgrades once perception/tracking quality improves.

## Recommended next follow-up

If more intersection quality work is needed, the next useful step is not a neural predictor. It is a slightly richer deterministic model:

- derive TTC from a better conflict point than the current mask center,
- reuse the existing `motion_planning.py` collision geometry where possible,
- add hold-time parameters and scenario tests for chattering, late detections, and committed entry.
