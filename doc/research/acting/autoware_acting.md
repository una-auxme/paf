# Research: [Autoware Acting](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/control/#autoware-control-design)

## Inputs

- Odometry (position and orientation, from Localization module)
- Trajectory (output of Planning)
- Steering Status (current steering of vehicle, from Vehicle Interface)
- Actuation Status (acceleration, steering, brake actuations, from Vehicle Interface)
- (“vehicle signal commands” directly into Vehicle Interface -> Handbrake, Hazard Lights, Headlights, Horn, Stationary Locking, Turn Indicators, Wipers etc.)

### General Component Architecture

![Node diagram](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/images/Control-Bus-ODD-Architecture.drawio.svg)

### With the Control Module

![control-component](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/control/image/control-component.drawio.svg)

## [Trajectory Follower](https://autowarefoundation.github.io/autoware.universe/main/control/trajectory_follower_base/)

- generates control command to follow reference trajectory from Planning
- computes lateral (steering) and longitudinal (velocity) controls separately
- lateral controller: mpc (model predictive) or pure pursuit
- longitudinal: “currently only” PID controller

## Vehicle Command Gate

- filters control commands to prevent abnormal values
- sends commands to [Vehicle Interface](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/)

## Outputs

- steering angle
- steering torque
- speed
- acceleration
