# How to test/tune acting components independedly

**Summary:** This page shows ways to test and tune acting components and to verify that they work as intended.

- [Acting\_Debug\_Node](#acting_debug_node)
  - [Setup for Testing with the Debug-Node](#setup-for-testing-with-the-debug-node)
  - [Operating the Debug-Node](#operating-the-debug-node)

## Acting_Debug_Node

The [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py) allows you to tune/test/verify all acting and control components independently of Planning-Inputs and also lets you easily output the data needed to plot/verify the tuning/testing you have done.

There is a dedicated [acting_debug.launch](../../code/acting/launch/acting_debug.launch) file which starts the acting component **as well as the necesarry perception components** automatically. It is recommended to first test the [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py)
with an empty road scenario. The following guide provides instructions on how to launch this setup.

### Setup for Testing with the Debug-Node

To use the [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py) you first have to edit a few files in your current branch:
>[!WARNING]! Make sure to revert these changes when pushing your branch!

- In [dev.launch](../../code/agent/launch/dev.launch) the [agent.launch](../../code/agent/launch/agent.launch) file is included. Change this to include [acting_debug.launch](../../code/acting/launch/acting_debug.launch) from the **acting** package.
As mentioned above this includes everything we need.
- In [docker-compose.devroute.yaml](../../build/docker-compose.devroute.yaml) change the command to 'bash -c "sleep 10 && roslaunch agent/launch/dev.launch"'
- You can now 'docker compose up' the [docker-compose.devroute.yaml](../../build/docker-compose.devroute.yaml).
  This should result in an empty road scenario with rviz and rqt.

To now decide whether to work on noisy sensor data, with or without the kalman-filter, here is how to change this:

- In the **position_heading_publisher_node** in [perception.launch](../../code/perception/launch/perception.launch) you can set a ```pos_filter``` and an ```heading_filter```. Here you can decide if and which filter to use on the GNSS and IMU sensor data, with *Kalman* being the standard option.
- The developer_enviroment also allows you to adjust the sensors used, in particular their noise deviations.
If you want to test a component on ideal sensor data (position/heading/both), go to the [dev_objects.json](../../code/agent/config/dev_objects.json) and find the ```GNSS```(position) and the ```IMU```(heading) sensors (lines 142-173) and set their noise-attributes to 0.
It is then also recommended to set the ```pos_filter``` and ```heading_filter``` of the **position_heading_publisher_node** to None (as filtering ideal sensor data will make them inaccurate again).

In [acting_debug.launch](../../code/acting/launch/acting_debug.launch) you can also set up different plot configurations for the rqt-window.

### Operating the Debug-Node

When you open the [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py) you will see, that alot of Testing-Options are selectable via Global Variables.
You can choose between different kinds of Test-Cases, select which trajectory and target_velocity the node should publish, what data to print for verification/plotting and a time (in simulation-seconds) after which the selected data is printed (to the terminal).
See the comments inside the Node for more information about different Test-Cases implemented and how to correctly use them.
