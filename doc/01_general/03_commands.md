# ⌨️ Available commands

A specific `b5` workflow for gpu installation in this project is specified in an issue comment: <https://github.com/una-auxme/paf/issues/260#issuecomment-2196852330>

## General commands

### `b5 run`

Starts the Project (docker-compose up).

### `b5 halt`

Stops the Project (docker-compose down).

### `b5 shell`

Makes it possible to get a shell of any docker container contained in the project.

Possible arguments:

| `argument`  | `description`          | `optional` | `default` |
|-------------|------------------------|------------|-----------|
| `container` | Container name         | True       | flake8    |
| `command`   | Command to be executed | True       |           |

Usage: `b5 shell <container> <command>`

#### Examples

```shell
# Execute flake8 lint in `components`-folder:
b5 shell flake8 components

# Get Shell in perception container (hypothetic example)
b5 shell perception
```

## Project setup / maintenance

### `b5 install`

Setup the project. Has to be run after cloning the project.

### `b5 update`

Update the project.

## Project linting

### `b5 lint`

Runs the project linters. More documentation about linting can be found [here](../02_development/02_linting.md).

### `b5 python:lint`

Runs the python linter. More documentation about linting can be found [here](../02_development/02_linting.md).

### `b5 markdown:lint`

Runs the markdown linter. More documentation about linting can be found [here](../02_development/02_linting.md).

## Shortcuts for ROS

Shortcuts to run the ROS commands directly in the container. Detailed documentation
about this commands can be found [here](http://wiki.ros.org/ROS/CommandLineTools#Common_user_tools).

For more complex tasks it's easier to just get a shell into the container with `b5 shell` and run the commands there.

`b5 rosbag`
`b5 ros_readbagfile`
`b5 rosbash`
`b5 roscd`
`b5 rosclean`
`b5 roscore`
`b5 rosdep`
`b5 rosed`
`b5 roscreate-pkg`
`b5 roscreate-stack`
`b5 rosrun`
`b5 roslaunch`
`b5 roslocate`
`b5 rosmake`
`b5 rosmsg`
`b5 rosnode`
`b5 rospack`
`b5 rosparam`
`b5 rossrv`
`b5 rosservice`
`b5 rosstack`
`b5 rostopic`
`b5 rosversion`
`b5 rqt_graph`

## 🚨 Common Problems

`
REQUIRED process [carla_ros_bridge-1] has died!
`

If the execution of `b5 run` is stopping because of this error the reason might be a duplicate Carla ROS bridge.

To eliminate this problem, run `b5 halt --remove-orphans`.
