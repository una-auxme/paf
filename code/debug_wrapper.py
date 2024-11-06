#!/usr/bin/env python3

import importlib
import importlib.util
import os
import sys
import argparse

import rospy

DEBUG_WRAPPER_MSG_PREFIX = "[debug_wrapper]:"
TYPE_PARAM = "~debug_type"
PORT_PARAM = "~debug_port"
WAIT_PARAM = "~debug_wait"


def eprint(*args, **kwargs):
    """Print to stderr"""
    print(*args, file=sys.stderr, **kwargs)


def logfatal(msg: str):
    """Only works after ros node has been initialized"""
    rospy.logfatal(
        f"""{DEBUG_WRAPPER_MSG_PREFIX} FAILED TO START NODE - NODE WILL NOT SHOW UP:
        {msg}"""
    )


def logerr(msg: str):
    """Only works after ros node has been initialized"""
    rospy.logerr(f"{DEBUG_WRAPPER_MSG_PREFIX} {msg}")


def loginfo(msg: str):
    """Only works after ros node has been initialized"""
    rospy.loginfo(f"{DEBUG_WRAPPER_MSG_PREFIX} {msg}")


def logdebug(msg: str):
    """Only works after ros node has been initialized"""
    print(f"{DEBUG_WRAPPER_MSG_PREFIX} {msg}")


def get_required_params():
    try:
        result = {}
        if not rospy.has_param(TYPE_PARAM):
            logfatal(
                """Missing parameter to start debug wrapper: debug_type
                Add it in the launch configuration"""
            )
        result["type"] = str(rospy.get_param(TYPE_PARAM))
        if rospy.has_param(PORT_PARAM):
            result["port"] = int(rospy.get_param(PORT_PARAM))
        else:
            logerr(
                """Missing parameter to start debugger: debug_port
                Add it in the launch configuration"""
            )
            result["port"] = None
        result["wait"] = bool(rospy.get_param(WAIT_PARAM, False))
        return result
    except BaseException as error:
        logerr(f"Failed to get required node parameters: {error}")
        raise error


def run_node_at(path: str):
    try:
        runpy.run_path(path, run_name="__main__")
    except BaseException as error:
        logfatal(
            f"""Failed to run node module at {path}:
                 {error}"""
        )
        raise error


def start_debugger(port: int, node_module_name: str, wait_for_debugee: bool = False):
    debugger_spec = importlib.util.find_spec("debugpy")
    if debugger_spec is not None:
        try:
            import debugpy

            debugpy.listen(("localhost", port))
            loginfo(f"Started debugger on port {port} for {node_module_name}")
            if wait_for_debugee:
                debugpy.wait_for_client()
        except BaseException as error:
            # Yes, all exceptions should be catched and sent into rosconsole
            logerr(f"Failed to start debugger: {error}")
    else:
        logerr("debugpy module not found. Unable to start debugger")


def main(argv):
    """# http://wiki.ros.org/Nodes: 7. Special keys -- __name is the node's name.
    # In this case the name is set by the launch file
    # Todo: when using rosrun, the name also has to be set somehow
    ros_node_name: str = __name
    # We have to init the node to be able to log and access parameters
    rospy.init_node(name=ros_node_name)"""
    node_args = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(
        prog="debug wrapper",
    )
    parser.add_argument("--debug_node")
    parser.add_argument("--debug_port")
    parser.add_argument("--debug_wait")
    args, unknown = parser.parse_known_args(node_args)

    base_dir = os.path.abspath(os.path.dirname(__file__))
    logdebug(f"Node {ros_node_name} started at {base_dir}")
    params = get_required_params()
    if params["port"] is not None:
        start_debugger(params["port"], params["type"], wait_for_debugee=params["wait"])
    target_type_path = os.path.join(base_dir, params["type"])
    run_node_at(target_type_path)


if __name__ == "__main__":
    main(argv=sys.argv)
