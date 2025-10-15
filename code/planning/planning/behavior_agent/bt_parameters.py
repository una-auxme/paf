from typing import Optional
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange


def _register_parameter(
    node: Node,
    name: str,
    default_value: str | bool | int | float,
    description: Optional[str] = None,
    min_value: Optional[int | float] = None,
    max_value: Optional[int | float] = None,
    step: Optional[int | float] = None,
) -> str | bool | int | float:
    """Registers a ros parameter

    Args:
        node (Node): Node to register parameter on
        name (str): Parameter name
        default_value (str | bool | int | float): Default parameter value.
            Also determines the type of the parameter
        description (Optional[str], optional): Parameter description.
            Shows up as tooltip in the rqt gui. Defaults to None.
        min_value (Optional[int  |  float], optional): Min value for gui slider.
            Defaults to None.
        max_value (Optional[int  |  float], optional): Max value for gui slider.
            Defaults to None.
        step (Optional[int  |  float], optional): Step for gui slider.
            Defaults to None.
    """
    param_desc = ParameterDescriptor()
    if description is not None:
        param_desc.description = description
    if min_value is not None or max_value is not None or step is not None:
        if isinstance(default_value, int):
            int_range = IntegerRange()
            int_range.from_value = 0 if min_value is None else int(min_value)
            int_range.to_value = 0 if max_value is None else int(max_value)
            int_range.step = 0 if step is None else int(step)
            param_desc.integer_range = [int_range]
        elif isinstance(default_value, float):
            float_range = FloatingPointRange()
            float_range.from_value = 0.0 if min_value is None else float(min_value)
            float_range.to_value = 0.0 if max_value is None else float(max_value)
            float_range.step = 0.0 if step is None else float(step)
            param_desc.floating_point_range = [float_range]

    parameter = node.declare_parameter(name, default_value, param_desc)
    if isinstance(default_value, str):
        return_value = parameter.get_parameter_value().string_value
    elif isinstance(default_value, bool):
        return_value = parameter.get_parameter_value().bool_value
    elif isinstance(default_value, int):
        return_value = parameter.get_parameter_value().integer_value
    elif isinstance(default_value, float):
        return_value = parameter.get_parameter_value().double_value
    else:
        return_value = "type error"
    return return_value


def register_parameters(node: Node):
    """Registers all blackboard ros parameters for the behavior tree node

    Args:
        node (Node): behavior tree node
    """
    _register_parameter(
        node, "left_check_debug", False, "Stay in left check indefinitely"
    )
    _register_parameter(
        node,
        "left_check_length",
        45.0,
        "Left check mask length",
        min_value=5.0,
        max_value=100.0,
        step=0.1,
    )
    _register_parameter(
        node,
        "left_check_x_transform",
        12.0,
        "Left check mask length",
        min_value=0.0,
        max_value=50.0,
        step=0.1,
    )
