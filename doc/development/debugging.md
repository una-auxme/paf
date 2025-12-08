# Debugging

**Summary:** This page explains multiple debugging methods for ROS nodes.

- [Debugging possibilities](#debugging-possibilities)
  - [Message based debugging](#message-based-debugging)
    - [Viewing the messages](#viewing-the-messages)
    - [Problems of message based debugging](#problems-of-message-based-debugging)
  - [VS Code debugger](#vs-code-debugger)
    - [Setup steps required once](#setup-steps-required-once)
    - [Required once for the node you want to debug](#required-once-for-the-node-you-want-to-debug)
      - [Important function parameters](#important-function-parameters)
    - [Debugging workflow after setup](#debugging-workflow-after-setup)
    - [Known problems of debugging with VS Code](#known-problems-of-debugging-with-vs-code)

## Debugging possibilities

There are two main debugging possibilities: Message based and VS Code based

### Message based debugging

Messages can be logged into the ROS console vie the `self.get_logger().debug`, `self.get_logger().info`, `self.get_logger().warn`, `self.get_logger().error` and `self.get_logger().fatal` functions.
Most messages are then published to the /rosout topic.

`self.get_logger().debug` is not recommended, because these messages are not published to /rosout by default.

#### Viewing the messages

There are several ways to view the ROS log messages

- The most convenient way to view logs is via the **rqt_console** GUI, which starts when the leaderboard is started.
  It allows filtering for debug levels, nodes and message content.

  Caveat: It only includes messages published to the /rosout topic.

- The leaderboard logs usually end up in [/internal_workspace/log/ros](/internal_workspace/log/ros).
  
  This includes the [agent.log](/internal_workspace/log/ros/agent.log) file where most of all the nodes output is captured.
  It seems to exclude stdout of the nodes and excludes the debug log level. stderr and all other levels are included.
  Exceptions that occur at node initialization can also be found here.

- Manually starting a node with `ros2 run <package> <node>` helps to view stdout, stderr and exceptions.

#### Problems of message based debugging

- Time intensive and cumbersome: leaderboard has to be restarted to add a new message
- Frequent log messages "spam" the log
- Exceptions on node initialization do not show up in the /rosout topic

### VS Code debugger

Debug individual nodes with the VS Code debugger. This allows usage of breakpoints and code stepping.

#### Setup steps required once

1. Make sure the docker images are up-to-date: [Rebuild docker containers](#rebuild-docker-containers)
2. Start the dev container and wait for VS Code to open. Always use the integrated VS Code instance for debugging

#### Required once for the node you want to debug

Inside the node.py script, add the following lines into the `main` function:

```python3
from paf_common.debugging import start_debugger

start_debugger(wait_for_client=False)
```

Like this:

```python
def main(args=None):
    from paf_common.debugging import start_debugger

    start_debugger(wait_for_client=False)

    rclpy.init(args=args)

    try:
        node = PrePlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
```

`start_debugger` starts the debugpy debugger on the default port **53000** before running the actual python script.

**Important**: Multiple nodes must never start the debugger on the same port (they will crash otherwise). **ALWAYS comment out the start_debugger call (+import) when you are done debugging!!**

##### Important function parameters

- `wait_for_client=True` makes the function wait until a debugger is attached.
- `port`: debugger port. Defaults to *53000*. Do not start multiple debuggers with the default port.

#### Debugging workflow after setup

1. Start/Restart the leaderboard+agent as usual
2. Wait for the application to start up
3. Add breakpoints to your node
4. Start the VS Code debugger (debugging tab). A default launch configuration named *53000 leaderboard attach* is available. It uses port **53000**.

#### Known problems of debugging with VS Code

- The side effects of just "stopping" a node with the debugger are unknown
