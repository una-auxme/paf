from typing import List

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    SetParametersResult,
)


def update_attributes(obj: Node, params: List[Parameter]) -> SetParametersResult:
    """Update attributes of obj with params

    Important: Attribute names must match parameter names

    Args:
        obj (Node): Node which attributes are updated
        params (List[Parameter]): parameters with new values

    Returns:
        SetParametersResult:
    """
    result = SetParametersResult()
    result.successful = True
    for param in params:
        error_reason = None
        if hasattr(obj, param.name):
            new_value = param.value
            orig_value = getattr(obj, param.name)
            if type(orig_value) is not type(new_value):
                error_reason = "type mismatch"
            else:
                setattr(obj, param.name, param.value)
                obj.get_logger().info(f"Updated parameter {param.name} to {new_value}")
        else:
            error_reason = "attribute not found"

        if error_reason is not None:
            result.successful = False
            result.reason = error_reason
            obj.get_logger().warn(
                f"Failed to update parameter {param.name}: {result.reason}"
            )

    return result
