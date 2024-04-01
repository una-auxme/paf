# Acting

**Summary:** This package contains all functions implemented for the acting component.

---

## Authors

Alexander Hellmann

## Date

01.04.2024

---

<!-- TOC -->
- [Acting](#acting)
  - [Authors](#authors)
  - [Date](#date)
  - [Acting Documentation](#acting-documentation)
  - [Test/Debug/Tune Acting-Components](#testdebugtune-acting-components)
  - [Longitudinal controllers (Velocity Controller)](#longitudinal-controllers-velocity-controller)
  - [Lateral controllers (Steering Controllers)](#lateral-controllers-steering-controllers)
  - [Vehicle controller](#vehicle-controller)
  - [Visualization of the HeroFrame in rviz](#visualization-of-the-heroframe-in-rviz)
<!-- TOC -->

## Acting Documentation

In order to further understand the general idea of the taken approach to the acting component please refer to the documentation of the [research](../../doc/03_research/01_acting/Readme.md) done and see the planned [general definition](../../doc/01_general/04_architecture.md#acting).

It is also highly recommended to go through the indepth [Acting-Documentation](../../doc/05_acting/Readme.md)!

## Test/Debug/Tune Acting-Components

The Acting_Debug_Node can be used as a simulated Planning package, publishing adjustable target velocities, steerings and trajectories as needed.

For more information about this node and how to use it, please read the [documentation](../../doc/05_acting/05_acting_testing.md).
You can also find more information in the commented [code](./src/acting/Acting_Debug_Node.py).

## Longitudinal controllers (Velocity Controller)

The longitudinal controller is implemented as a PID velocity controller.

For more information about this controller, either read the [documentation](../../doc/05_acting/02_velocity_controller.md) or go through the commented [code](./src/acting/velocity_controller.py).

## Lateral controllers (Steering Controllers)

There are two steering controllers currently implemented, both providing live telemetry via Debug-Messages:

- Pure Persuit Controller (paf/hero/pure_p_debug)
- Stanley Controller (paf/hero/stanley_debug)

For further information about the steering controllers, either read the [documentation](./../../doc/05_acting/03_steering_controllers.md) or go through the commented code of [stanley_controller](./src/acting/stanley_controller.py) or [purepursuit_controller](./src/acting/pure_pursuit_controller.py).

## Vehicle controller

The VehicleController collects all necessary msgs from the other controllers and publishes the [CarlaEgoVehicleControl](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#carlaegovehiclecontrol) for the [Carla ros bridge](https://github.com/carla-simulator/ros-bridge).

It also executes emergency-brakes and the unstuck-routine, if detected.

For more information about this controller, either read the [documentation](../../doc/05_acting/04_vehicle_controller.md) or go through the commented [code](./src/acting/vehicle_controller.py).

## Visualization of the HeroFrame in rviz

For information about vizualizing the upcomming path in rviz see [Main frame publisher](../../doc/05_acting/06_main_frame_publisher.md)
