# Documentation of control component

This folder contains the documentation of the control component.

The control component applies control theory based on a local trajectory provided
by the [acting component](./../acting/README.md). It uses knowledge of the current state
of the vehicle in order to send [CarlaEgoVehicleControl](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#CarlaEgoVehicleControlmsg) commands to the Simulator.


1. [Architecture](./architecture_documentation.md)
2. [Overview of the Velocity Controller](./velocity_controller.md)
3. [Overview of the Steering Controllers](./steering_controllers.md)
4. [Overview of the Vehicle Controller Component](./vehicle_controller.md)
5. [How to test/tune control components independently](./control_testing.md)