#!/usr/bin/env python3

import importlib.util
import os
import sys
import argparse

import rospy

DEBUG_WRAPPER_MSG_PREFIX = "[debug_wrapper]:"
TYPE_PARAM = "~debug_type"
PORT_PARAM = "~debug_port"
WAIT_PARAM = "~debug_wait"


def printdebug(msg: str):
    print(f"{DEBUG_WRAPPER_MSG_PREFIX} {msg}")


def printerror(msg: str):
    """Print to stderr"""
    print(f"{DEBUG_WRAPPER_MSG_PREFIX} {msg}", file=sys.stderr)


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


def import_module_at(path: str):
    basename = os.path.basename(path)
    module_dir = os.path.dirname(path)
    module_name = os.path.splitext(basename)[0]
    # Based on https://docs.python.org/3/library/importlib.html#importing-a-source-file-directly
    """ spec = importlib.util.spec_from_file_location(
        name=module_name, location=path, submodule_search_locations=[module_dir]
    )
    if spec is None:
        raise Exception(f"Failed to load {path} as module {module_name}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module) """
    sys.path.append(module_dir)
    module = importlib.import_module(module_name)
    return module


def start_debugger(port: int, node_module_name: str, wait_for_client: bool = False):
    debugger_spec = importlib.util.find_spec("debugpy")
    if debugger_spec is not None:
        try:
            import debugpy

            debugpy.listen(("localhost", port))
            loginfo(f"Started debugger on port {port} for {node_module_name}")
            if wait_for_client:
                debugpy.wait_for_client()
        except BaseException as error:
            # Yes, all exceptions should be catched and sent into rosconsole
            logerr(f"Failed to start debugger: {error}")
    else:
        logerr("debugpy module not found. Unable to start debugger")


class ArgumentParserError(Exception):
    pass


class ThrowingArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        raise ArgumentParserError(message)


def main(argv):
    node_args = rospy.myargv(argv=argv)
    parser = ThrowingArgumentParser(
        prog="debug wrapper",
    )
    parser.add_argument("--debug_node", required=True)
    parser.add_argument("--debug_port", required=False, type=int)
    parser.add_argument("--debug_wait", default=False, type=bool)
    args, unknown_args = parser.parse_known_args(node_args)
    debug_node = args.debug_node

    base_dir = os.path.abspath(os.path.dirname(__file__))
    printdebug(f"Node {args.debug_node} starting at {base_dir}")

    target_type_path = os.path.join(base_dir, debug_node)
    module = import_module_at(target_type_path)

    module.init_ros()

    if args.debug_port is not None:
        start_debugger(
            args.debug_port, args.debug_node, wait_for_client=args.debug_wait
        )
    else:
        logerr(
            """Missing parameter to start debugger: --debug_port
                Add it in the launch configuration"""
        )

    try:
        module.main(unknown_args)
    except BaseException as error:
        logfatal(f"Failed to run node {debug_node}: {error}")
        raise error


if __name__ == "__main__":
    main(argv=sys.argv)
