from typing import Optional
import inspect
import importlib.util

import rclpy.logging
from rclpy.impl.rcutils_logger import RcutilsLogger


def get_logger() -> RcutilsLogger:
    return rclpy.logging.get_logger("debugger")


def get_caller_file() -> str:
    stack = inspect.stack()
    if len(stack) < 1:
        return "unknown"
    caller = stack[0]
    return caller.filename


def start_debugger(
    node_module_name: Optional[str] = None,
    host: str = "127.0.0.1",
    port: int = 53000,
    wait_for_client: bool = False,
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
            if node_module_name is None:
                node_module_name = get_caller_file()
            import debugpy

            debugpy.listen((host, port))
            get_logger().warn(
                f"Started debugger on {host}:{port} for {node_module_name}"
            )
            if wait_for_client:
                get_logger().warn("Waiting until debugging client is attached...")
                debugpy.wait_for_client()
        except Exception as error:
            # Yes, all exceptions should be catched and sent into rosconsole
            get_logger().error(f"Failed to start debugger: {error}")
    else:
        get_logger().error("debugpy module not found. Unable to start debugger")
