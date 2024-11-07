# Debugging

**Summary:** This page explains multiple debugging methods for ROS nodes.

- [Debugging possibilities](#debugging-possibilities)
  - [Message based debugging](#message-based-debugging)
  - [VS Code debugger](#vs-code-debugger)
- [Rebuild docker containers](#rebuild-docker-containers)
- [Sources](#sources)

## Debugging possibilities

There are two main debugging possibilities

### Message based debugging

Messages can be logged into the ROS console vie the `rospy.logdebug`, `rospy.loginfo`, `rospy.logwarn`, `rospy.logerror` and `rospy.logfatal` functions.

### VS Code debugger



## Rebuild docker containers

The docker images on your pc might not match the latest Dockerfile in the repository

To update them, open a terminal and change into the *build* directory. Execute:

```bash
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$(id -u -n)
docker compose -f ./docker-compose.leaderboard.yaml up -d --build
docker compose -f ./docker-compose.dev.yaml up -d --build
```

## Sources

<https://www.markdownguide.org/cheat-sheet/>
