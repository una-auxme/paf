# Debugging

**Summary:** This page explains multiple debugging methods for ROS nodes.

- [Debugging possibilities](#debugging-possibilities)
  - [Message based debugging](#message-based-debugging)
    - [Viewing the messages](#viewing-the-messages)
    - [Problems of message based debugging](#problems-of-message-based-debugging)
  - [VS Code debugger](#vs-code-debugger)
    - [Setup steps required once](#setup-steps-required-once)
    - [Required once for the node you want to debug](#required-once-for-the-node-you-want-to-debug)
    - [Debugging workflow after setup](#debugging-workflow-after-setup)
    - [Known problems of debugging with VS Code](#known-problems-of-debugging-with-vs-code)
    - [Debug Components Architecture](#debug-components-architecture)
- [Rebuild docker containers](#rebuild-docker-containers)
- [Sources](#sources)

## Debugging possibilities

There are two main debugging possibilities: Message based and VS Code based

### Message based debugging

Messages can be logged into the ROS console vie the `rospy.logdebug`, `rospy.loginfo`, `rospy.logwarn`, `rospy.logerror` and `rospy.logfatal` functions.
Most messages are then published to the /rosout topic.

`rospy.logdebug` is not recommended, because these messages are not published to /rosout by default.

Note that the node has to be initialized with `rospy.init_node()` before logging, otherwise the messages are not published to /rosout and just end up in stdout/stderr.

#### Viewing the messages

There are several ways to view the ROS log messages

- The most convenient way to view logs is via the **rqt_console** GUI, which starts when the leaderboard is started.
  It allows filtering for debug levels, nodes and message content.

  Caveat: It only includes messages published to the /rosout topic.

- Execute `rosconsole echo` inside the *build-agent* container. This shows the /rosout messages from this point on.

- The leaderboard logs usually end up in [code/log/ros](../../code/log/ros).
  
  This includes the [agent.log](../../code/log/ros/agent.log) file where most of all the nodes output is captured.
  It seems to exclude stdout of the nodes and excludes the debug log level. stderr and all other levels are included.
  Exceptions that occur at node initialization can also be found here.

- More accurate "per-node" logs including the debug level and stdout end up in the individual node log files.
  These can be found in the directory returned by running `roslaunch-logs` inside the *build-agent* container. (Usually `~/.ros/log`)

- Manually starting a node with `rosrun <package> <node>` inside the *build-agent* container helps to view stdout, stderr and exceptions.

More information can be found in the [Wiki](https://wiki.ros.org/rospy/Overview/Logging)

#### Problems of message based debugging

- Time intensive and cumbersome: leaderboard has to be restarted to add a new message
- Frequent log messages "spam" the log
- Exceptions on node initialization do not show up in the /rosout topic
- If messages are sent right after `rospy.init_node()`, they might not be published to the /rosout topic

### VS Code debugger

Debug individual nodes with the VS Code debugger. This allows usage of breakpoints and code stepping.

#### Setup steps required once

1. Make sure the docker images are up-to-date: [Rebuild docker containers](#rebuild-docker-containers)
2. Start the `build/docker-compose.dev.yaml` containers and attach VS Code to the **build-agent-dev** container. Always use the VS Code remote for debugging
3. Make sure the recommended extensions are installed in the VS Code remote.

#### Required once for the node you want to debug

Adjust the launch configuration of the node to use the [debug_wrapper.py](../../code/debug_wrapper.py) node.

Change the type to **debug_wrapper.py** and set `args="--debug_node=<original type> --debug_port=<debug_port>"`.
*debug_port* can be a port in range **53000-53100**, because those are exposed from the [leaderboard docker configuration](../../build/docker-compose.leaderboard.yaml).

Configuration example for the [lidar_distance.py](../../code/perception/src/lidar_distance.py) node: [launch configuration](../../code/perception/launch/perception.launch)

- Original entry:

  ```launch
  <node pkg="perception" type="lidar_distance.py" name="lidar_distance" output="screen">
    <param name="point_cloud_topic" value="/carla/hero/LIDAR_filtered"/>
    <param name="range_topic" value="/carla/hero/LIDAR_range"/>
  </node>
  ```

- Entry modified for debugging:

  ```launch
  <node pkg="perception" type="debug_wrapper.py" name="lidar_distance" output="screen" args="--debug_node=lidar_distance.py --debug_port=53000">
    <param name="point_cloud_topic" value="/carla/hero/LIDAR_filtered"/>
    <param name="range_topic" value="/carla/hero/LIDAR_range"/>
  </node>
  ```

By default, the affected node will start up as usual. When you attach the debugger later, you are then only able to debug the callbacks the node receives.
But if you set `--debug_wait`, it will block the node from starting until VS Code attaches and allow you to debug the initialization of the node.

If the leaderboard hangs on `Setting up the agent` after the configuration has been adjusted, there most likely is a mistake in the launch configuration.

#### Debugging workflow after setup

1. Start/Restart the [leaderboard](../../build/docker-compose.leaderboard.yaml) as usual
2. Wait for the application to start up
3. Add breakpoints to your node.
4. Start the VS Code debugger (debugging tab). A default launch configuration named *53000 leaderboard attach* is available. It uses port **53000**.

Any errors/exceptions the debug_wrapper encounters are published to /rosout via the [debug_logger](../../code/debugging/src/debug_logger.py).
With the **rqt_console** GUI you can filter by node *debug_logger* to see all messages related to the wrapper.

#### Known problems of debugging with VS Code

- Adjusting the launch configurations is a bit cumbersome
- The side effects of just "stopping" a node with the debugger are unknown

#### Debug Components Architecture

More technical usage information can be found in the:

- [debug_wrapper](../../code/debug_wrapper.py): Acts as a proxy that wraps the target node and enables remote debugging capabilities.
- [debug_logger](../../code/debugging/src/debug_logger.py): Handles the logging of debugging-related events and exceptions, ensuring they are properly published to the ROS ecosystem.

The wrapper uses the logger to ensure that any debugging-related issues (connection problems, initialization errors, etc.) are visible through the /rosout topic.

## Rebuild docker containers

The docker images on your pc might not match the latest Dockerfile in the repository

To update them, open a terminal, change into the *build* directory and execute:

```bash
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$(id -u -n)
docker compose -f ./docker-compose.leaderboard.yaml up -d --build
docker compose -f ./docker-compose.dev.yaml up -d --build
```

## Sources

<https://wiki.ros.org/rospy/Overview/Logging>
