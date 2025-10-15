from typing import Optional
from rclpy.impl.rcutils_logger import RcutilsLogger
import rclpy.logging


_behaviors_logger: Optional[RcutilsLogger] = None


def set_logger(logger: RcutilsLogger):
    global _behaviors_logger
    _behaviors_logger = logger


def get_logger():
    if _behaviors_logger is None:
        return rclpy.logging.get_logger("behaviors")
    else:
        return _behaviors_logger
