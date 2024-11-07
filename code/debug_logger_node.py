#!/usr/bin/env python3
"""
This is a dedicated debug logger node

Without a fully initialized node, log messages do not show up in the rqt_console

This is why this node is used by the debug_wrapper for logging
before it has initialized it's node
"""

import time
import sys
from multiprocessing.connection import Listener

import rospy


def eprint(msg: str):
    print(f"[debug_logger_node]: {msg}", file=sys.stderr)


def log(name: str, msg: str, level: str):
    msg = f"[debug_logger for {name}]: {msg}"

    level = level.lower()
    if level == "debug":
        rospy.logdebug(msg)
    elif level == "info":
        rospy.loginfo(msg)
    elif level == "warn":
        rospy.logwarn(msg)
    elif level == "error":
        rospy.logerr(msg)
    else:
        rospy.logfatal(msg)


def main():
    try:
        address = ("localhost", 52999)  # family is deduced to be 'AF_INET'
        listener = Listener(
            address, authkey=b"debug_logger"
        )  # Only one listerner can be active at once
        eprint("Debug logger node started")
    except OSError as error:
        eprint(f"Failed to start listener: {error}. Exiting...")
        exit(0)

    rospy.init_node(name="debug_logger")
    time.sleep(
        5.0
    )  # We need to wait a bit until the node is fully initialized to send log messages

    # Based on
    # https://stackoverflow.com/questions/6920858/interprocess-communication-in-python
    while not rospy.is_shutdown():
        conn = listener.accept()
        print(f"[debug_logger]: connection accepted from {listener.last_accepted}")
        msg = conn.recv()
        conn.close()
        log_msg = "Wrong log message format"
        log_name = "NAMERR"
        log_level = "fatal"
        if isinstance(msg, dict):
            if "name" in msg:
                log_name = msg["name"]
            if "msg" in msg:
                log_msg = msg["msg"]
            if "level" in msg:
                log_level = msg["level"]
        log(log_name, log_msg, log_level)

    listener.close()


if __name__ == "__main__":
    main()
