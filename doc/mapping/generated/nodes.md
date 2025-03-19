<!-- markdownlint-disable -->
# Nodes

## Table of Contents

* [mapping\_data\_integration](#mapping_data_integration)
  * [MappingDataIntegrationNode](#mapping_data_integration.MappingDataIntegrationNode)
    * [stop\_marks](#mapping_data_integration.MappingDataIntegrationNode.stop_marks)
    * [dynamic\_reconfigure\_callback](#mapping_data_integration.MappingDataIntegrationNode.dynamic_reconfigure_callback)

<a id="mapping_data_integration"></a>

# mapping\_data\_integration

<a id="mapping_data_integration.MappingDataIntegrationNode"></a>

## MappingDataIntegrationNode

```python
class MappingDataIntegrationNode(CompatibleNode)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L39)

Creates the initial map data frame based on all kinds of sensor data

It applies several filters to the map and
then sends it off to other consumers (planning, acting)

This node sends the maps off at a fixed rate.
(-> It buffers incoming sensor data slightly)

<a id="mapping_data_integration.MappingDataIntegrationNode.stop_marks"></a>

#### stop\_marks: `Dict[str, List[StopMark]]`

StopMarks from the UpdateStopMarks service

**Important:** the transform of these entities uses global coordinates

<a id="mapping_data_integration.MappingDataIntegrationNode.dynamic_reconfigure_callback"></a>

#### dynamic\_reconfigure\_callback

```python
def dynamic_reconfigure_callback(config: "MappingIntegrationConfig", level)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L157)

All currently used reconfigure options are queried dynamically.

The configuration options are located
[here](/code/mapping/launch/mapping.launch)

