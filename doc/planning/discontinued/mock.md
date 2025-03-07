# Mock Package

This subfolder 'mock' was an own package till end of PAF24.
As it wasn't used anymore till whole PAF24 we decided to move it to discontinued.

Following tests were integrated in this package:

- This node publishes stop sign light information. It can be used for
testing.
- This node publishes traffic light information. It can be used for testing.
- This node publishes intersection clear information. It can be used for
testing.

We also don't know if it's even working anymore. Also the intention for these tests seems
not so clear.

For reintegration move the mock folder inside the 'code' folder.
Then you need to include following

```xml
<include file="$(find mock)/launch/mock.launch">
    <arg name="control_loop_rate" value="$(arg control_loop_rate)" />
    <arg name="role_name" value="$(arg role_name)" />
</include>
```

into the code/agent/launch/agent.launch file.
