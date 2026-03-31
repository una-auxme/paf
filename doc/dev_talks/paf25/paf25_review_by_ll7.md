# PAF25 Review by LL7

## Table of Contents

- [1. Scope and Method](#1-scope-and-method)
- [2. Executive Summary](#2-executive-summary)
- [3. Repository Evolution](#3-repository-evolution)
- [4. Topic Review](#4-topic-review)
  - [4.1 Traffic Light](#41-traffic-light)
  - [4.2 Radar and Sensor Message Handling](#42-radar-and-sensor-message-handling)
  - [4.3 Auto Tests](#43-auto-tests)
  - [4.4 Cross Traffic Check](#44-cross-traffic-check)
  - [4.5 Lidar, Tracking and Fusion](#45-lidar-tracking-and-fusion)
- [5. Documentation Review Against the Requirements](#5-documentation-review-against-the-requirements)
  - [5.1 What Is Clearly Better Than at paf25.start](#51-what-is-clearly-better-than-at-paf25start)
  - [5.2 Where the Guidelines Are Not Fully Met Yet](#52-where-the-guidelines-are-not-fully-met-yet)
  - [5.3 Requirement-by-Requirement Verdict](#53-requirement-by-requirement-verdict)
- [6. Overall Verdict](#6-overall-verdict)

## 1. Scope and Method

This review compares `paf25.start` against `main`.

The comparison range contains:

- 221 commits
- 209 changed files
- 6019 insertions and 4867 deletions
- 52 changed Markdown files

The review is based on:

- `git diff` and `git log` for the full `paf25.start..main` range
- targeted inspection of the current implementations in perception, mapping, planning and test tooling
- targeted inspection of the updated documentation in `doc/`
- comparison against the rules defined in `doc/development/documentation_requirements.md`

The emphasis of this report is on:

- traffic light handling
- radar and sensor message handling
- auto tests
- cross traffic check
- lidar, tracking and related fusion work

## 2. Executive Summary

Compared to `paf25.start`, the repository has evolved from a more fragmented and partly legacy-heavy codebase into a more ROS2-focused, fusion-oriented stack with much stronger perception-to-mapping integration.

The most important technical shift is that perception is no longer only publishing geometric detections. It now publishes richer intermediate data that includes grouped traffic-light crops, lidar clusters, radar-derived motion and class information, and heading changes. Mapping and planning consume that richer data to do radar-lidar association, entity tracking, motion estimation and collision-aware behavior.

The most visible organizational changes are:

- major cleanup of legacy ROS1 components
- migration from Flake8 plus Black to Ruff in CI, docs and local tooling
- significantly expanded documentation effort
- introduction of a local automated route-test harness for ROS2 and CARLA leaderboard runs

Overall, the repository has clearly matured. The biggest strengths are perception and mapping integration, better traffic-light robustness, better lidar compensation and tracking, and much better route-level testing support. The biggest remaining weaknesses are documentation drift, incomplete test depth outside route tests, and known radar-motion limitations that are already documented by the team.

## 3. Repository Evolution

At a repository level, the last months were not only feature work. There was also a broad structural cleanup.

### 3.1 Technical Direction

The repository moved further toward a ROS2-centered active stack.

- Large parts of `code-ros1/` were removed or reduced in relevance.
- Build and container files were simplified and modernized.
- Perception, mapping and planning changed together, not independently.
- The intermediate layer now carries more semantic and motion information than before.

### 3.2 Tooling and Quality Workflow

The development workflow became stricter and more consistent.

- Ruff replaced Flake8 and Black.
- CI, VS Code settings, tasks and documentation were updated for Ruff.
- The repo now has a clearer single source of truth for linting and formatting.

This is a meaningful quality step because it reduces style drift between local development and automation.

### 3.3 Documentation Activity

Documentation effort increased strongly.

- 52 Markdown files changed in the review range.
- New or substantially updated docs were added for lidar, radar, cross-traffic behavior, local automatic tests, parameters and mapping internals.
- Mapping API documentation was regenerated and updated.

So the repository did not only change in code. It also attempted to explain the new architecture and workflows. The important caveat is that this documentation effort is uneven: some pages are strong and current, while others are already stale or inconsistent with the code.

## 4. Topic Review

### 4.1 Traffic Light

Traffic-light handling became noticeably more robust and more structured.

#### What changed

The key architectural change is the introduction of a grouped traffic-light message.

- `VisionNode` no longer publishes cropped traffic-light images one by one.
- It now collects all plausible traffic-light crops from one frame and publishes them together via the new `TrafficLightImages` message.
- `TrafficLightNode` subscribes to this grouped message instead of a single `sensor_msgs/Image`.

On top of that, the traffic-light classifier logic became more defensive.

- multiple crops can be evaluated per frame
- ambiguous frames are discarded
- impossible state transitions are filtered out
- turn-signal-like lights are filtered with a circle-based heuristic
- final state publication is buffered over time instead of reacting instantly to a single crop
- stale state is reset after a timeout

This is a significant improvement over a more direct per-image classification flow because it reduces flicker and reduces the chance that a single bad crop immediately changes the final state.

#### Why it matters

This change directly targets behavior reliability.

- The commit history shows explicit work on red-light issues.
- The final implementation now has temporal voting and plausibility filtering.
- A dedicated `routes_traffic_light.xml` route was added, which indicates that traffic-light behavior became important enough to justify isolated testing.

#### Remaining weaknesses

The code is ahead of the documentation here.

- `doc/perception/traffic_light_detection.md` still describes outdated APIs and behavior in several places.
- It still refers to a single image input, outdated helper functions and an older node model.

So the implementation improved substantially, but the documentation did not keep up fully.

### 4.2 Radar and Sensor Message Handling

This area saw one of the largest conceptual changes in the repository.

#### What changed

Radar is no longer treated mainly as a standalone clustering source.

Instead, the pipeline increasingly uses radar as a motion sensor that enriches lidar-based entities.

Important changes include:

- `radar_node.py` now subscribes to ego speed in addition to radar and IMU data.
- IMU-based pitch estimation is used for ground reflection filtering.
- The node can buffer radar messages over a configurable time window.
- When clustering is disabled, the node publishes ego-motion-compensated per-point motion on `/paf/hero/Radar/compensated_points`.
- `ClusteredPointsArray` was extended to carry `motion_array` and `object_class`.
- Mapping now consumes those richer messages and can associate radar motion with lidar entities.

This is a major evolution in sensor-message handling because the interface between perception and mapping became semantically richer. The messages are no longer just containers for point positions and cluster ids.

#### Fusion direction

The current direction is clear:

- lidar provides geometry and object extent
- radar provides motion cues
- mapping performs association and integrates both

The addition of `RadarPointAssignmentFilter` and the new association buffer parameter in mapping makes that explicit. The system is moving away from treating radar detections as fully independent final objects.

#### Why it matters

This improves several downstream behaviors:

- cross-traffic detection
- collision prediction
- dynamic entity tracking
- classification of moving vs stationary objects

#### Remaining weaknesses

This part is improved, but not fully solved.

- `doc/dev_talks/paf25/radar_velocity_issue.md` documents that stationary objects can still receive unrealistic velocities.
- `doc/perception/radar_node.md` explicitly warns that angular ego motion is still not considered in the velocity estimation.
- The current pipeline is therefore clearly more advanced than before, but it is still an engineering compromise rather than a finished radar-motion solution.

### 4.3 Auto Tests

The repository made clear progress on automated regression testing, especially at route level.

#### What changed

A local ROS2-oriented test harness was added.

- `code/test/run_test.py` is a large CARLA leaderboard-based local test runner.
- `code/test/index_dict.py` defines named scenarios with time thresholds.
- `code/routes/test.xml` was expanded heavily.
- `code/leaderboard_launcher/scripts/launch_leaderboard.test.sh` was added.
- `doc/general/ros2_local_test.md` documents how to run and extend the tests.

The runner supports:

- route execution through the leaderboard framework
- statistics collection
- checkpoint output
- resume support
- post-run evaluation of infractions such as collisions and red-light violations

This is a strong practical step because it gives the team a repeatable way to validate end-to-end behavior locally.

#### Additional testing progress

There is also at least one focused perception test now:

- `code/perception/tests/test_ego_motion_compensation.py`

That test validates the lidar ego-motion compensation math using controlled pose scenarios.

#### Remaining weaknesses

Testing improved, but it is still not comprehensive in the sense required by the documentation rules.

- Most of the new coverage is route-level and scenario-level.
- The codebase still has relatively few focused unit or integration tests for the many changes in planning, radar fusion and tracking.

So the repository evolved from almost no convincing automation in these new areas to a useful regression harness, but not yet to broad test coverage.

### 4.4 Cross Traffic Check

Cross-traffic handling became much more data-driven.

#### What changed

The old approach was simplified away and replaced by logic that uses dynamic map entities and their motion.

Key changes:

- obsolete helpers like `has_cross_traffic` and `check_cross_traffic` were removed
- `intersection.py` now contains explicit priority cross-traffic checks based on overlapping dynamic entities
- the behavior uses speed thresholds and rectangular check zones in `Wait` and `Enter`
- if fast cross traffic is detected while the ego vehicle is still moving, emergency signaling can be triggered

This is a clear upgrade from static or heuristic-only intersection logic.

#### Interaction with planning

Cross-traffic logic also became closer to collision reasoning.

- `motion_planning.py` now subscribes to the current map
- it predicts local trajectories for entities
- it checks trajectory collisions using `time_horizon` and `crash_threshold`
- collision trajectories are published for visualization

This means the repo is moving from simple intersection gating toward actual trajectory-based conflict estimation.

#### Remaining weaknesses

The repository itself documents that this logic is still conservative.

`doc/planning/behaviors/Intersection.md` correctly states that:

- the current cross-traffic check does not yet use motion direction robustly
- the rectangular check area rotates with the ego vehicle
- false-positive braking is therefore possible
- long term, the dedicated cross-traffic check may be replaced by a sufficiently reliable collision check

This is an honest and technically correct limitation statement.

### 4.5 Lidar, Tracking and Fusion

This is probably the strongest technical growth area in the whole comparison range.

#### What changed in lidar itself

`lidar_distance.py` was substantially redesigned.

The most important step is the strategy-based compensation architecture.

The node now supports:

- `NoCompensation`
- `Buffer`
- `EgoMotionCompensation`
- `LocalCompensation`

Depending on the mode, the node consumes:

- previous lidar frames
- EKF pose
- ego speed
- IMU heading

The node also now publishes delta heading for downstream use.

Clustering improved as well.

- an additional upper z filter was added
- clustering parameters changed
- the clustering logic moved toward a distance-aware, polar-like representation to better handle lidar point density

This is much more mature than a plain frame-by-frame DBSCAN over raw points.

#### What changed in tracking and mapping

The intermediate layer changed from static integration toward actual temporal tracking.

Important additions:

- `TrackingFilter` was added in mapping
- tracking uses two-frame history and Hungarian matching
- tracking supports type persistence across frames
- motion updates can be toggled
- delta heading from lidar compensation is fed into tracking
- sensor ids are propagated and deduplicated

At the same time, lidar entities can now receive radar-derived motion through `RadarPointAssignmentFilter`.

This is the clearest example of the repository evolving from "perception publishes detections" to "perception and mapping together build tracked dynamic entities".

#### Why it matters

This work directly enables:

- better dynamic obstacle understanding
- motion-aware planning
- cross-traffic reasoning
- collision prediction
- more stable object identity across frames

#### Remaining weaknesses

The technical direction is strong, but some modules are now very large.

- `lidar_distance.py`
- `radar_node.py`
- parts of mapping integration and filtering

These files are more capable than before, but also harder to maintain and review. So the architecture improved, while local code complexity also increased.

## 5. Documentation Review Against the Requirements

## 5.1 What Is Clearly Better Than at paf25.start

The repository clearly invested real effort into documentation.

Positive examples:

- `doc/perception/lidar_distance.md` is detailed, structured and technically useful.
- `doc/perception/radar_node.md` explains design decisions, sensor placement and known limitations.
- `doc/planning/behaviors/Intersection.md` documents both the behavior and its current limitations.
- `doc/general/ros2_local_test.md` explains how the local automatic test workflow works.
- mapping documentation was refreshed heavily.
- the linting/documentation requirements themselves were improved.

In other words: the repo did not just accumulate code. It also tried to preserve engineering context.

## 5.2 Where the Guidelines Are Not Fully Met Yet

The problem is not lack of documentation effort. The problem is consistency and maintenance.

### Example 1: Traffic-light documentation is partially stale

`doc/perception/traffic_light_detection.md` still contains outdated descriptions, for example:

- it describes older APIs and node behavior
- it still talks about a single image flow where the implementation now uses `TrafficLightImages`
- it references logic that is no longer present in the code in the same form

This violates the maintenance requirement and partially also the explicit-usage requirement.

### Example 2: Architecture documentation is stale

`doc/general/architecture_current.md` still contains outdated topic and message references, for example:

- old message paths for clustered point messages
- ROS1-style or outdated references around dynamic reconfigure and message types
- a traffic-light pipeline description that no longer reflects the current grouped-image message flow

This weakens the value of the architecture document because it can mislead new contributors.

### Example 3: Not all new docs follow the required Markdown structure

The documentation requirements ask for a table of contents and numbered headings for all documents.

This is not applied consistently.

- `doc/perception/lidar_distance.md` and `doc/perception/radar_node.md` follow the structure reasonably well.
- `doc/perception/traffic_light_detection.md` has a linked list but not the same numbering discipline.
- `doc/planning/behaviors/Intersection.md` is clear, but not numbered as required.
- `doc/general/ros2_local_test.md` is useful, but also does not follow the numbering convention from the template.
- `doc/perception/radar_raw_debugger.md` is minimal and does not follow the more complete template structure.

### Example 4: At least one Markdown file is malformed

`doc/perception/radar_raw_debugger.md` currently ends with an opening fenced code block and no closing fence.

That directly violates the documentation-quality requirement.

### Example 5: In-code documentation improved, but not uniformly

The new lidar compensation code and some perception code now have proper docstrings and clearer internal structure.
However, some of the largest and most important modules are still very large and multi-purpose.

This means readability and maintainability improved in parts, but the codebase does not uniformly meet the documented standard.

## 5.3 Requirement-by-Requirement Verdict

The following verdict is limited to what can be assessed from the local repository.

| Requirement Area | Verdict | Assessment |
| --- | --- | --- |
| Remove or clearly mark deprecated code | Partially met | Good progress: large ROS1 cleanup, discontinued docs, removed obsolete logic. Not fully met because some documentation still describes outdated behavior. |
| Python linting and formatting | Met | Ruff was introduced across CI, tasks, docs and local tooling. |
| Python docstrings | Partially met | Better than before, especially in lidar/radar related code, but still uneven. |
| Readability and maintainability | Partially met | Several subsystems became more modular, but some files are now very large and combine too many responsibilities. |
| Code structure and modularity | Partially met | Strategy pattern and filters are improvements, but complexity is still concentrated in a few large modules. |
| Efficiency and performance | Mostly met | The repo shows concrete algorithmic work on compensation, clustering and tracking. I did not find evidence of performance being ignored. |
| Error handling | Partially met | Some nodes improved logging and guarded behavior, but robustness still varies by module. |
| Testing | Partially met | Strong route-level autotest improvement and one focused perception test, but not comprehensive coverage for all major changes. |
| Markdown structure and organization | Partially met | Many docs improved, but TOC and numbered-heading rules are not applied consistently. |
| Content detail and usage docs | Mostly met | Radar, lidar and autotest docs contain substantial technical detail. Traffic-light and architecture docs are weaker because they drifted. |
| Visual aids | Partially met | Some useful images exist, but several new docs omit diagrams or screenshots. |
| Maintenance and updates | Partially met | Documentation was updated often, but not always kept aligned with the final code. |
| Deprecation handling in docs | Partially met | Good use of `discontinued/` and some cleanup, but stale active docs remain. |
| GitHub repository hygiene | Not fully assessable locally | Issues, PR closure and remote branch hygiene cannot be verified from the local checkout alone. |

## 6. Overall Verdict

Compared to `paf25.start`, the repository has evolved substantially and in the right direction.

The strongest improvements are:

- a much more capable perception-to-mapping pipeline
- better traffic-light robustness
- radar-lidar fusion with motion-aware entity handling
- actual multi-frame tracking in mapping
- new local automatic route testing
- better tooling and linting discipline

The most important technical evolution is that motion information now matters throughout the stack. Radar, lidar, mapping and planning are more tightly connected than before, and the repository looks much more like a system for dynamic-scene reasoning rather than a collection of independent nodes.

The main remaining weaknesses are:

- documentation drift in a few important active docs
- insufficient test depth beyond route-level regression tests
- known radar-motion limitations that are documented but not fully solved
- increasing complexity in a few large core modules

In summary, the repository did not merely accumulate new features during the last months. It became architecturally more coherent, more ROS2-focused, more fusion-aware and more testable. The remaining work is mostly in consolidation: align the documentation with the final code, keep reducing legacy leftovers, and add more targeted automated tests for the new planning and fusion logic.
