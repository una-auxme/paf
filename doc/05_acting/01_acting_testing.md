# How to test acting components

**Summary:** This page shows ways to verify that acting components work as expected

---

## Author

Alexander Hellmann

## Date

09.01.2023

<!-- TOC -->
- [How to test acting components](#how-to-test-acting-components)
  - [Author](#author)
  - [Date](#date)
  - [velocity\_controller](#velocity_controller)
  - [Dummys](#dummys)
<!-- TOC -->

## velocity_controller

To test the velocity controller, set the parameter ```enabled``` in the acting launch-file true. Now the velocity_publisher_dummy will publish dummy target speeds. Use ```rqt_plot /carla/hero/velocity_as_float /carla/hero/max_velocity``` to visualize target and current speeds.

![image not found](./../00_assets/testing_velocity_pid.png)

## Dummys

To test the steering controllers and the acc there are two dummys:

* the ```DummyTrajectoryPub``` publishes a simple path to test the steering controllers on,
* the ```AccDistancePublisherDummy``` publishes a dummy distances to an imaginary vehicle in front

To activate these dummys, make sure that they are started in the launch file and ```<param name="enabled" value="True" />``` is set in the launch file.
