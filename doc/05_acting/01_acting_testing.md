# Testing of Acting

**Summary:** This page shows ways to verify/test/tune/debug acting components.

---

## Author

Alexander Hellmann

## Date

19.03.2024

---
<!-- TOC -->
- [Testing of Acting](#testing-of-acting)
  - [Author](#author)
  - [Date](#date)
  - [Acting\_Debug\_Node](#acting_debug_node)
  - [velocity\_controller](#velocity_controller)
  - [Steering Controllers](#steering-controllers)
  - [Dummys](#dummys)
<!-- TOC -->

---

## Acting_Debug_Node

To test acting components independently from Perception and Planning, use the Acting_Debug_Node.py class in Acting.

For this to work properly you have to manually disable all Planning and most of the Perception Components in your testing-branch:

- Go to the planning.launch file (.../code/planning/launch/planning.launch) and disable every active node by commenting them out.
- Go to the perception.launch file (.../code/perception/launch/perception.launch) and disable every other node EXCEPT the Position_Publisher_Node.py and the kalman_filter.py by commenting them out.

If you want to test/debug/tune under perfect conditions we also recommend following changes:

- In the docker-compose.yml file (.../build/docker-compose.yml) you can switch to a developer enviroment by uncommenting the dev.launch (Line 65) and commenting the leaderboard (Line 66).
- In the dev_objects.json file (.../code/agent/config/dev_objects.json) you will find the GPS and IMU sensors. To use ideal sensors, set all noises to 0.
- In the perception.launch file, set the Position_Publisher_Node.py's pos_filter and heading_filter to None to get the ideal sensors' data (position and heading) unfiltered.

## velocity_controller

To test the velocity controller, set the parameter ```enabled``` in the acting launch-file true. Now the velocity_publisher_dummy will publish dummy target speeds. Use ```rqt_plot /carla/hero/velocity_as_float /carla/hero/max_velocity``` to visualize target and current speeds.

![image not found](./../00_assets/testing_velocity_pid.png)

## Steering Controllers

## Dummys

To test the steering controllers and the acc there are two dummys:

- the ```DummyTrajectoryPub``` publishes a simple path to test the steering controllers on,
- the ```AccDistancePublisherDummy``` publishes a dummy distances to an imaginary vehicle in front

To activate these dummys, make sure that they are started in the launch file and ```<param name="enabled" value="True" />``` is set in the launch file.
