#!/usr/bin/env python3
"""Debug wrapper node

Node that wraps a python ros node
and is able to open a debugpy remote debugger instance.

Logs any exceptions from the node and other information
into the ros console via the debug_logger node.

Always tries to at least start the node,
even if dependencies like debugpy are missing.

Usage:
    Already done for existing ros packages: symlink this file into the package. Example:
    `cd code/perception/src && ln -s ../../debug_wrapper.py debug_wrapper.py`

    Adjust the launch configuration to use the debug_wrapper.py
    instead of the node file and set the required args

    More info in [debugging.md](doc/development/debugging.md)

Arguments:
    --debug_node: Required: The filename of the node to debug
    --debug_port: The port the debugger listens on. If not set,
    --debug_host: The host the debugger binds to.
        Defaults to the environment variable `DEBUG_WRAPPER_DEFAULT_HOST` if set,
        otherwise `localhost`
    --debug_wait: If True, the wrapper waits until a client (VS Code) is connected
        and only then starts node execution

Raises:
    ArgumentParserError: Missing required parameters
    error: If the started debug_node raises an exception,
        it is logged and then raised again
"""


import importlib.util
import os
import runpy
import sys
import argparse
import time
from multiprocessing.connection import Client

import rospy

NODE_NAME = "NAMEERR"


def eprint(msg: str):
    """Log msg into stderr.

    Used instead of print, because only stderr seems to reliably land in agent.log

    Args:
        msg (str): Log message
    """
    print(f"[debug_wrapper]: {msg}", file=sys.stderr)


def log(msg: str, level: str):
    """Log msg via the debug_logger node

    Args:
        msg (str): Log message
        level (str): Log level. One of (debug), info, warn, error, fatal.
            debug level not recommended, because these are not published to /rosout
    """
    error = None
    success = False
    start_time = time.monotonic()
    while not success and start_time + 5.0 > time.monotonic():
        try:
            address = ("localhost", 52999)
            conn = Client(address, authkey=b"debug_logger")
            conn.send({"name": NODE_NAME, "msg": msg, "level": level})
            conn.close()
            success = True
        except Exception as e:
            error = e
    if not success:
        eprint(msg)
        if error is not None:
            eprint(f"Failed to send to logger: {error}")


def logfatal(msg: str):
    log(f"FAILED TO START NODE - NODE WILL NOT SHOW UP: {msg}", "fatal")


def logerr(msg: str):
    log(msg, "error")


def logwarn(msg: str):
    log(msg, "warn")


def loginfo(msg: str):
    log(msg, "info")


def run_module_at(path: str):
    """Runs a python module based on its file path

    Args:
        path (str): python file path to run
    """
    basename = os.path.basename(path)
    module_dir = os.path.dirname(path)
    module_name = os.path.splitext(basename)[0]
    sys.path.append(module_dir)
    runpy.run_module(module_name, run_name="__main__")


def start_debugger(
    node_module_name: str, host: str, port: int, wait_for_client: bool = False
):
    """_summary_

    Args:
        node_module_name (str): Name of the underlying node. Only used for logging
        host (str): host the debugger binds to
        port (int): debugger port
        wait_for_client (bool, optional): If the debugger should wait
            for a client to attach. Defaults to False.
    """
    debugger_spec = importlib.util.find_spec("debugpy")
    if debugger_spec is not None:
        try:
            import debugpy

            debugpy.listen((host, port))
            logwarn(f"Started debugger on {host}:{port} for {node_module_name}")
            if wait_for_client:
                logwarn("Waiting until debugging client is attached...")
                debugpy.wait_for_client()
        except Exception as error:
            # Yes, all exceptions should be catched and sent into rosconsole
            logerr(f"Failed to start debugger: {error}")
    else:
        logerr("debugpy module not found. Unable to start debugger")


class ArgumentParserError(Exception):
    pass


class ThrowingArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        logfatal(f"Wrong node arguments. Check launch config. : {message}")
        raise ArgumentParserError(message)


def main(argv):
    default_host = "localhost"
    if "DEBUG_WRAPPER_DEFAULT_HOST" in os.environ:
        default_host = os.environ["DEBUG_WRAPPER_DEFAULT_HOST"]

    node_args = rospy.myargv(argv=argv)
    parser = ThrowingArgumentParser(
        prog="debug wrapper",
    )
    parser.add_argument("--debug_node", required=True, type=str)
    parser.add_argument("--debug_port", required=False, type=int)
    parser.add_argument("--debug_host", default=default_host, type=str)
    parser.add_argument("--debug_wait", action="store_true")
    args, unknown_args = parser.parse_known_args(node_args)

    debug_node = args.debug_node
    global NODE_NAME
    NODE_NAME = debug_node
    base_dir = os.path.abspath(os.path.dirname(__file__))

    if args.debug_port is not None:
        start_debugger(
            args.debug_node,
            args.debug_host,
            args.debug_port,
            wait_for_client=args.debug_wait,
        )
    else:
        logerr(
            """Missing parameter to start debugger: --debug_port
                Add it in the launch configuration"""
        )

    target_type_path = os.path.join(base_dir, debug_node)
    loginfo(f"Node {args.debug_node} starting at {base_dir}")
    try:
        run_module_at(target_type_path)
    except BaseException as error:
        # Yes, all exceptions including SystemExit should be catched.
        # We want to always know when a node exits
        logfatal(f"Failed to run node {debug_node}: {error}")
        raise error


if __name__ == "__main__":
    main(argv=sys.argv)
