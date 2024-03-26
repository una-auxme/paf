# How to test/tune acting components independedly

**Summary:** This page shows ways to test and tune acting components and to verify that they work as intended.

---

## Author

Alexander Hellmann

## Date

26.03.2024

<!-- TOC -->
- [How to test/tune acting components independedly](#how-to-testtune-acting-components-independedly)
  - [Author](#author)
  - [Date](#date)
  - [Acting\_Debug\_Node](#acting_debug_node)
    - [Setup for Testing with the Debug-Node](#setup-for-testing-with-the-debug-node)
    - [Operating the Debug-Node](#operating-the-debug-node)
<!-- TOC -->

## Acting_Debug_Node

The [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py) allows you to tune/test/verify all acting components independently of Planning-Inputs and also lets you easily output the data needed to plot/verify the tuning/testing you have done.

### Setup for Testing with the Debug-Node

To use the [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py) you first have to edit a few files in your current branch:

- In the [acting.launch](../../code/acting/launch/acting.launch) file, you need to un-comment the ```Acting_Debug_Node```-node.
- Disable Planning from sending *trajectory*, *target_velocity*, *unstuck* or *emergency* Messages.
The easiest way to achieve this is to go to the [planning.launch](../../code/planning/launch/planning.launch) and just comment-out every active node.
- As the Perception-nodes use alot of processing power and ressources and you most likely do not need them, you can disable them, again commenting-out unwanted nodes in the [perception.launch](../../code/perception/launch/perception.launch) file.
**IMPORTANT:** As you need the **position_heading_publisher_node** and most likely the **kalman_filter** for the GNSS/IMU data, do keep those 2 active!
- There is a developer-enviroment ready to use, if you want to test your components on an empty road.
To switch to this, go to [docker_compose.yml](../../build/docker-compose.yml), where you will find which enviroment is currently chosen in lines 64-66. Uncomment the ```dev_launch``` and comment-out the ```leaderboard_evaluator``` if you want to use this developer-enviroment.

To now decide whether to work on noisy sensor data, with or without the kalman-filter, here is how to change this:

- In the **position_heading_publisher_node** in [perception.launch](../../code/perception/launch/perception.launch) you can set a ```pos_filter``` and an ```heading_filter```. Here you can decide if and which filter to use on the GNSS and IMU sensor data, with *Kalman* being the standard option.
- The developer_enviroment also allows you to adjust the sensors used, in particular their noise deviations.
If you want to test a component on ideal sensor data (position/heading/both), go to the [dev_objects.json](../../code/agent/config/dev_objects.json) and find the ```GNSS```(position) and the ```IMU```(heading) sensors (lines 142-173) and set their noise-attributes to 0.
It is then also recommended to set the ```pos_filter``` and ```heading_filter``` of the **position_heading_publisher_node** to None (as filtering ideal sensor data will make them inaccurate again).

### Operating the Debug-Node

When you open the [Acting_Debug_Node](../../code/acting/src/acting/Acting_Debug_Node.py) you will see, that alot of Testing-Options are selectable via Global Variables.
You can choose between different kinds of Test-Cases, select which trajectory and target_velocity the node should publish, what data to print for verification/plotting and a time (in simulation-seconds) after which the selected data is printed (to the terminal).
See the comments inside the Node for more information about different Test-Cases implemented and how to correctly use them.
