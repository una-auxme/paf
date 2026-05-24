# Future Work

## Table of Contents

- [Table of Contents](#table-of-contents)
- [1. Goal](#1-goal)
- [2. Recommended Development Principles](#2-recommended-development-principles)
  - [2.1 Consolidation Before Expansion](#21-consolidation-before-expansion)
  - [2.2 Interfaces Must Become Explicit](#22-interfaces-must-become-explicit)
  - [2.3 Development Must Be Evidence-Driven](#23-development-must-be-evidence-driven)
  - [2.4 Integration Should Happen Weekly](#24-integration-should-happen-weekly)
- [3. Proposed Work Packages](#3-proposed-work-packages)
- [3.1 WP1: Documentation and Interface Consolidation](#31-wp1-documentation-and-interface-consolidation)
  - [Objective](#objective)
  - [Why this should be first](#why-this-should-be-first)
  - [Main tasks](#main-tasks)
  - [Expected output](#expected-output)
- [3.2 WP2: Radar Motion Quality](#32-wp2-radar-motion-quality)
  - [Objective](#objective-1)
  - [Why this matters](#why-this-matters)
  - [Main tasks](#main-tasks-1)
  - [Expected output](#expected-output-1)
- [3.3 WP3: Lidar, Mapping and Tracking Stabilization](#33-wp3-lidar-mapping-and-tracking-stabilization)
  - [Objective](#objective-2)
  - [Why this matters](#why-this-matters-1)
  - [Main tasks](#main-tasks-2)
  - [Expected output](#expected-output-2)
- [3.4 WP4: Collision Prediction and Intersection Logic](#34-wp4-collision-prediction-and-intersection-logic)
  - [Objective](#objective-3)
  - [Why this matters](#why-this-matters-2)
  - [Main tasks](#main-tasks-3)
  - [Expected output](#expected-output-3)
- [3.5 WP5: Automated Testing and CI Expansion](#35-wp5-automated-testing-and-ci-expansion)
  - [Objective](#objective-4)
  - [Why this matters](#why-this-matters-3)
  - [Main tasks](#main-tasks-4)
  - [Expected output](#expected-output-4)
- [3.6 WP6: Performance, Metrics and Observability](#36-wp6-performance-metrics-and-observability)
  - [Objective](#objective-5)
  - [Why this matters](#why-this-matters-4)
  - [Main tasks](#main-tasks-5)
  - [Expected output](#expected-output-5)
- [3.7 WP7: Optional Feature Expansion](#37-wp7-optional-feature-expansion)
  - [Objective](#objective-6)
  - [Candidate topics](#candidate-topics)
  - [Important note](#important-note)
- [4. Proposed Execution Order](#4-proposed-execution-order)
- [5. Suggested Team Split for 4 Students](#5-suggested-team-split-for-4-students)
  - [Student 1: Perception and Radar](#student-1-perception-and-radar)
  - [Student 2: Mapping and Tracking](#student-2-mapping-and-tracking)
  - [Student 3: Planning and Intersection Logic](#student-3-planning-and-intersection-logic)
  - [Student 4: Testing, Tooling and Documentation](#student-4-testing-tooling-and-documentation)
- [6. Definition of Done for the Next Phase](#6-definition-of-done-for-the-next-phase)

## 1. Goal

This document proposes how development should continue after the current state summarized in [paf25_review_by_ll7.md](./paf25_review_by_ll7.md) and [improvements_assessment.md](./improvements_assessment.md).

The core recommendation is simple:

Do not immediately expand feature breadth again.
First consolidate the new architecture so that perception, mapping, planning and testing become more reliable and easier to maintain.

## 2. Recommended Development Principles

The next phase should follow these principles:

### 2.1 Consolidation Before Expansion

The repository already has enough technically interesting ideas.

The next phase should focus on making those ideas stable, testable and well-documented before adding many more features.

### 2.2 Interfaces Must Become Explicit

The semantics of messages, topics and intermediate representations should be fixed more clearly.

That is especially important for:

- motion data
- tracked entities
- classification data
- planning-relevant map entities

### 2.3 Development Must Be Evidence-Driven

Future changes should be justified with measurable effects.

Examples:

- fewer false positives in cross-traffic detection
- lower radar velocity error
- more stable track ids
- lower collision rate on selected routes

### 2.4 Integration Should Happen Weekly

The project already shows signs of integration complexity.

That means each work package should include recurring integration checkpoints instead of long isolated branches.

## 3. Proposed Work Packages

## 3.1 WP1: Documentation and Interface Consolidation

### Objective

Bring the active documentation back in sync with the actual implementation and freeze the most important subsystem contracts.

### Why this should be first

Right now some documentation is already stale although the code improved a lot. That is a warning sign that architectural knowledge is still too implicit.

### Main tasks

- update active docs so they reflect current code behavior
- fix stale topic and message references in architecture docs
- document the semantics of `TrafficLightImages`, `ClusteredPointsArray`, radar compensated points and tracked entities
- define which topics and messages are considered stable interfaces
- fix malformed or incomplete Markdown pages

### Expected output

- current architecture documentation
- current perception and planning docs
- no major active doc contradicts the code
- clear subsystem interfaces for further development

## 3.2 WP2: Radar Motion Quality

### Objective

Improve radar velocity estimation so that moving vs stationary classification becomes more reliable.

### Why this matters

This is one of the most important known weaknesses in the current stack. It directly affects cross-traffic detection, entity classification and collision reasoning.

### Main tasks

- re-evaluate radial velocity interpretation at different azimuth angles
- include ego angular motion where appropriate
- compare per-point vs per-cluster velocity aggregation
- validate sensor-frame vs vehicle-frame transformation assumptions
- create a benchmark set for stationary and moving reference cases

### Expected output

- more stable radar motion estimates
- fewer false positives for stationary objects
- documented benchmark results
- updated radar documentation with verified assumptions

## 3.3 WP3: Lidar, Mapping and Tracking Stabilization

### Objective

Make the fusion and tracking pipeline more robust and easier to maintain.

### Why this matters

This is currently one of the strongest areas technically, but also one of the most complex.

### Main tasks

- split overly large modules into smaller units where possible
- add tests for tracking behavior across multiple frames
- validate radar-lidar assignment thresholds and association behavior
- improve handling of track loss, re-acquisition and noisy detections
- consider explicit confidence values or track quality indicators

### Expected output

- easier-to-review fusion code
- more stable tracked entities
- lower regression risk in mapping and tracking

## 3.4 WP4: Collision Prediction and Intersection Logic

### Objective

Move from conservative area-based cross-traffic checks toward more direction-aware and trajectory-aware intersection behavior.

### Why this matters

The current cross-traffic logic is useful, but it is still explicitly documented as conservative and potentially prone to false-positive braking.

### Main tasks

- use motion direction more explicitly in cross-traffic decisions
- validate the collision prediction logic in `motion_planning.py`
- connect trajectory conflict reasoning more tightly to behavior decisions
- reduce the dependence on simple rectangular speed-threshold checks
- define scenario-based acceptance tests for blocked, safe and ambiguous intersections

### Expected output

- fewer unnecessary emergency stops
- better intersection decisions
- a clearer path to replacing the current conservative cross-traffic guard

## 3.5 WP5: Automated Testing and CI Expansion

### Objective

Expand the current route-level testing into a broader reliability net.

### Why this matters

The local automatic test harness is a good start, but the most complex logic still lacks sufficient focused test coverage.

### Main tasks

- add unit tests for perception utility functions and compensation math
- add tests for radar-lidar assignment and tracking filters
- add tests for traffic-light state buffering and invalid transitions
- add focused tests for collision and intersection helpers
- define a small smoke-test subset that can run regularly in CI

### Expected output

- better protection against regressions
- faster debugging of feature work
- clearer confidence in system changes

## 3.6 WP6: Performance, Metrics and Observability

### Objective

Make technical quality measurable.

### Why this matters

Without metrics, it is hard to know whether changes improve the system or only change behavior.

### Main tasks

- measure perception and mapping latency
- log basic tracking quality metrics
- measure collision and infraction rates on standard routes
- track traffic-light and radar-specific failure cases
- create lightweight debug dashboards or structured result summaries

### Expected output

- repeatable quality indicators
- evidence for technical tradeoffs
- easier prioritization for future work

## 3.7 WP7: Optional Feature Expansion

### Objective

Only after consolidation, expand system capability further.

### Candidate topics

- improved semantic classification consistency across sensors
- richer track prediction models
- stronger emergency vehicle handling
- more formal occupancy or risk modeling in planning
- additional route and scenario coverage for testing

### Important note

This work package should start only after WP1 to WP5 are in a reasonably good state.

## 4. Proposed Execution Order

The recommended order is:

1. WP1: Documentation and interface consolidation
2. WP5: Automated testing and CI expansion
3. WP2: Radar motion quality
4. WP3: Lidar, mapping and tracking stabilization
5. WP4: Collision prediction and intersection logic
6. WP6: Performance, metrics and observability
7. WP7: Optional feature expansion

This order is intentional.

It starts by making the current system understandable and testable, then fixes the most important motion-quality weakness, then strengthens fusion and planning on top of that.

## 5. Suggested Team Split for 4 Students

If development continues with 4 students, a practical split would be:

### Student 1: Perception and Radar

- radar motion quality
- traffic-light reliability follow-up
- perception-side tests

### Student 2: Mapping and Tracking

- tracking filter stabilization
- radar-lidar assignment
- entity confidence and fusion cleanup

### Student 3: Planning and Intersection Logic

- collision prediction validation
- cross-traffic behavior refinement
- route-level scenario evaluation

### Student 4: Testing, Tooling and Documentation

- interface documentation
- CI and smoke tests
- metrics and evaluation tooling
- architecture consistency checks

This split should not create silos.

All four should still meet weekly to review:

- interface changes
- route-test results
- regressions
- updated priorities

## 6. Definition of Done for the Next Phase

The next development phase should be considered successful only if the following conditions are met:

1. Active documentation matches the current implementation.
2. The most important fusion and planning helpers have focused automated tests.
3. Radar motion quality is improved and backed by explicit benchmark results.
4. Cross-traffic and collision behavior is validated on a defined route set.
5. The largest core modules are easier to understand or split into clearer units.
6. Future feature work can proceed on top of a more stable baseline.

That would turn the current progress from a strong student milestone into a much more robust development platform.
