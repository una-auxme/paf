# Dynamic Reconfigure

**Summary:** Dynamic Reconfigure is a powerful tool which allows to dynamically adjust parameters of ROS-Nodes during runtime.

- [How to integrate into your Node](#how-to-integrate-into-your-node)
  - [Parameter Updates](#parameter-updates)
- [Adjust Parameters in RQT](#adjust-parameters-in-rqt)

Read more about dynamic reconfigure: [DynamicReconfigure](https://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile/catkin)

## How to integrate into your Node

In order to use parameters with your Node, you need to declare them inside your node's `__init__`.

A simple parameter declaration `enable_stop_marks` with default value `True`:

```python
self.enable_stop_marks = ( # attribute the initial parameter value is written to
    self.declare_parameter(
        "enable_stop_marks", # parameter name. Prefer naming it the same as the attribute
        True # default value
    )
    .get_parameter_value()
    .bool_value
)
```

A parameter declaration with a description (The description shows up in RViz when hovering over the parameter):

```python
from rcl_interfaces.msg import (
    ParameterDescriptor,
)

self.enable_stop_marks = ( 
    self.declare_parameter(
        "enable_stop_marks",
        True,
        descriptor=ParameterDescriptor(
            description="Enable stop marks from the UpdateStopMarks service",
        ),
    )
    .get_parameter_value()
    .bool_value
)
```

Parameters can also include bounds:

```python
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
)

self.filter_merge_growth_distance = (
    self.declare_parameter(
        "filter_merge_growth_distance",
        0.3,
        descriptor=ParameterDescriptor(
            description="Amount shapes grow before merging in meters",
            floating_point_range=[
                FloatingPointRange(from_value=0.01, to_value=5.0, step=0.01)
            ],
        ),
    )
    .get_parameter_value()
    .double_value
)
```

### Parameter Updates

After setting an attribute like `self.enable_stop_marks` at the declaration of the parameter, it will not get updated automatically.

To enable updates to the node's parameter-attributes, add the following to the node:

```python
from rclpy.parameter import Parameter
from paf_common.parameters import update_attributes

#...

# Node
def __init__(self):
    # declare parameters
    #...

    # register callback that is called when a parameter is updated
    self.add_on_set_parameters_callback(self._set_parameters_callback)

def _set_parameters_callback(self, params: List[Parameter]):
    """Callback for parameter updates."""
    # Updates the node's attibutes based on the parameter name
    # FOR THIS TO WORK, THE PARAMETER NAMES HAVE TO MATCH THE ATTRIBUTES
    return update_attributes(self, params)
```

## Adjust Parameters in RQT

The following image shows the parameters for the `test_node` node, which was configured as described above. The RQT-Plugin can be found under `Plugins->Configuration->DynamicReconfigure`.

![rqtimage](../assets/general/rqt.png)
