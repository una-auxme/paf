# Dynamic Reconfigure

**Summary:** Dynamic Reconfigure is a powerful tool which allows to dynamically adjust parameters of ROS-Nodes during runtime.

- [How to integrate into your Node](#how-to-integrate-into-your-node)
  - [Dynamic Reconfigure Server, Config and Callback](#dynamic-reconfigure-server-config-and-callback)
  - [Declaring Parameters](#declaring-parameters)
- [Adjust Parameters in RQT](#adjust-parameters-in-rqt)

Read more about dynamic reconfigure: [DynamicReconfigure](https://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile/catkin)

## How to integrate into your Node

In order to integrate Dynamic-Reconfigure with your Node the following steps need to be made.

### Dynamic Reconfigure Server, Config and Callback

At first a `.cfg` file needs to be created in a `config` folder in your ROS package.
The following shows a minimal version of such file.

```python
#!/usr/bin/env python
PACKAGE = "mapping_visualization"
from dynamic_reconfigure.parameter_generator_catkin import *
gen = ParameterGenerator()
tab_inputs = gen.add_group("Select 0 for Any -1 for isNot and 1 for Is in order to filter", type="tab")
tab_inputs.add("flag_motion", int_t, 0, "Filter for motion.", 0, -1, 1)
exit(gen.generate(PACKAGE, "mapping_visualization", "MappingVisualization"))
```

The first lines stay the same regardless of your project. Only the `PACKAGE` variable needs to be adjusted.
After this a tab group is generated, which allows for the dynamic reconfigure page to have multiple tabs in order to organize different aspects of your node.

Then the two most important lines:

```python
tab_inputs.add("flag_motion", int_t, 0, "Filter for motion.", 0, -1, 1)
```

A Parameter is added with the name `flag_motion`. This needs to conform with the ROS parameter naming conventions. Then the type is specified `int_t`.

### Declaring Parameters

The parameters you want to be able to dynamically reconfigure first need to be declared just like in any other Node. This can either be done via a `.yaml` file, or with arguments in the `.launch`.

## Adjust Parameters in RQT

The following image shows the parameters for the mapping_visualization node.
