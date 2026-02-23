<!-- markdownlint-disable -->
# Nodes

## Table of Contents

* [mapping.data\_integration](#mapping.data_integration)
  * [MappingDataIntegrationNode](#mapping.data_integration.MappingDataIntegrationNode)
    * [lidar\_data](#mapping.data_integration.MappingDataIntegrationNode.lidar_data)
    * [hero\_speed](#mapping.data_integration.MappingDataIntegrationNode.hero_speed)
    * [lidar\_clustered\_points\_data](#mapping.data_integration.MappingDataIntegrationNode.lidar_clustered_points_data)
    * [radar\_clustered\_points\_data](#mapping.data_integration.MappingDataIntegrationNode.radar_clustered_points_data)
    * [vision\_clustered\_points\_data](#mapping.data_integration.MappingDataIntegrationNode.vision_clustered_points_data)
    * [stop\_marks](#mapping.data_integration.MappingDataIntegrationNode.stop_marks)
    * [current\_pos](#mapping.data_integration.MappingDataIntegrationNode.current_pos)
    * [current\_heading](#mapping.data_integration.MappingDataIntegrationNode.current_heading)
    * [\_\_init\_\_](#mapping.data_integration.MappingDataIntegrationNode.__init__)
    * [update\_stopmarks\_callback](#mapping.data_integration.MappingDataIntegrationNode.update_stopmarks_callback)
    * [heading\_callback](#mapping.data_integration.MappingDataIntegrationNode.heading_callback)
    * [current\_pos\_callback](#mapping.data_integration.MappingDataIntegrationNode.current_pos_callback)
    * [hero\_speed\_callback](#mapping.data_integration.MappingDataIntegrationNode.hero_speed_callback)
    * [lidar\_clustered\_points\_callback](#mapping.data_integration.MappingDataIntegrationNode.lidar_clustered_points_callback)
    * [radar\_clustered\_points\_callback](#mapping.data_integration.MappingDataIntegrationNode.radar_clustered_points_callback)
    * [vision\_clustered\_points\_callback](#mapping.data_integration.MappingDataIntegrationNode.vision_clustered_points_callback)
    * [lidar\_callback](#mapping.data_integration.MappingDataIntegrationNode.lidar_callback)
    * [entities\_from\_lidar\_marker](#mapping.data_integration.MappingDataIntegrationNode.entities_from_lidar_marker)
    * [entities\_from\_radar\_marker](#mapping.data_integration.MappingDataIntegrationNode.entities_from_radar_marker)
    * [lanemarkings\_callback](#mapping.data_integration.MappingDataIntegrationNode.lanemarkings_callback)
    * [entities\_from\_lidar](#mapping.data_integration.MappingDataIntegrationNode.entities_from_lidar)
    * [create\_entities\_from\_clusters](#mapping.data_integration.MappingDataIntegrationNode.create_entities_from_clusters)
    * [create\_hero\_entity](#mapping.data_integration.MappingDataIntegrationNode.create_hero_entity)
    * [publish\_new\_map\_handler](#mapping.data_integration.MappingDataIntegrationNode.publish_new_map_handler)
    * [publish\_new\_map](#mapping.data_integration.MappingDataIntegrationNode.publish_new_map)
    * [get\_current\_map\_filters](#mapping.data_integration.MappingDataIntegrationNode.get_current_map_filters)
  * [main](#mapping.data_integration.main)
* [mapping.visualization](#mapping.visualization)
  * [MARKER\_NAMESPACE](#mapping.visualization.MARKER_NAMESPACE)
  * [Visualization](#mapping.visualization.Visualization)
    * [\_\_init\_\_](#mapping.visualization.Visualization.__init__)
    * [create\_map\_sub](#mapping.visualization.Visualization.create_map_sub)
    * [map\_callback](#mapping.visualization.Visualization.map_callback)
    * [create\_deleteall\_marker](#mapping.visualization.Visualization.create_deleteall_marker)
  * [main](#mapping.visualization.main)

<a id="mapping.data_integration"></a>

# mapping.data\_integration

<a id="mapping.data_integration.MappingDataIntegrationNode"></a>

## MappingDataIntegrationNode

```python
class MappingDataIntegrationNode(Node)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L40)

Creates the initial map data frame based on all kinds of sensor data.

It applies several filters to the map and
then sends it off to other consumers (planning, acting)

This node sends the maps off at a fixed rate.
(-> It buffers incoming sensor data slightly)

#### Services

This node provides the following services:

- **UpdateStopMarks**:
  Allows a client to insert virtual StopMark Entities into the Map.

  Each list of StopMarks has a unique id. With this id, a client
  can update their specific list of marks.

<a id="mapping.data_integration.MappingDataIntegrationNode.lidar_data"></a>

#### lidar\_data: `Optional[PointCloud2]`

<a id="mapping.data_integration.MappingDataIntegrationNode.hero_speed"></a>

#### hero\_speed: `Optional[CarlaSpeedometer]`

<a id="mapping.data_integration.MappingDataIntegrationNode.lidar_clustered_points_data"></a>

#### lidar\_clustered\_points\_data: `Optional[ClusteredPointsArray]`

<a id="mapping.data_integration.MappingDataIntegrationNode.radar_clustered_points_data"></a>

#### radar\_clustered\_points\_data: `Optional[ClusteredPointsArray]`

<a id="mapping.data_integration.MappingDataIntegrationNode.vision_clustered_points_data"></a>

#### vision\_clustered\_points\_data: `Optional[ClusteredPointsArray]`

<a id="mapping.data_integration.MappingDataIntegrationNode.stop_marks"></a>

#### stop\_marks: `Dict[str, List[StopMark]]`

StopMarks from the UpdateStopMarks service

**Important:** the transform of these entities uses global coordinates

<a id="mapping.data_integration.MappingDataIntegrationNode.current_pos"></a>

#### current\_pos: `Optional[Point2]`

<a id="mapping.data_integration.MappingDataIntegrationNode.current_heading"></a>

#### current\_heading: `Optional[float]`

<a id="mapping.data_integration.MappingDataIntegrationNode.__init__"></a>

#### \_\_init\_\_

```python
def __init__()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L74)

<a id="mapping.data_integration.MappingDataIntegrationNode.update_stopmarks_callback"></a>

#### update\_stopmarks\_callback

```python
def update_stopmarks_callback(
        request: UpdateStopMarks.Request,
        response: UpdateStopMarks.Response) -> UpdateStopMarks.Response
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L407)

<a id="mapping.data_integration.MappingDataIntegrationNode.heading_callback"></a>

#### heading\_callback

```python
def heading_callback(data: Float32)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L429)

<a id="mapping.data_integration.MappingDataIntegrationNode.current_pos_callback"></a>

#### current\_pos\_callback

```python
def current_pos_callback(data: PoseStamped)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L432)

<a id="mapping.data_integration.MappingDataIntegrationNode.hero_speed_callback"></a>

#### hero\_speed\_callback

```python
def hero_speed_callback(data: CarlaSpeedometer)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L435)

<a id="mapping.data_integration.MappingDataIntegrationNode.lidar_clustered_points_callback"></a>

#### lidar\_clustered\_points\_callback

```python
def lidar_clustered_points_callback(data: ClusteredPointsArray)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L438)

<a id="mapping.data_integration.MappingDataIntegrationNode.radar_clustered_points_callback"></a>

#### radar\_clustered\_points\_callback

```python
def radar_clustered_points_callback(data: ClusteredPointsArray)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L441)

<a id="mapping.data_integration.MappingDataIntegrationNode.vision_clustered_points_callback"></a>

#### vision\_clustered\_points\_callback

```python
def vision_clustered_points_callback(data: ClusteredPointsArray)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L444)

<a id="mapping.data_integration.MappingDataIntegrationNode.lidar_callback"></a>

#### lidar\_callback

```python
def lidar_callback(data: PointCloud2)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L447)

<a id="mapping.data_integration.MappingDataIntegrationNode.entities_from_lidar_marker"></a>

#### entities\_from\_lidar\_marker

```python
def entities_from_lidar_marker() -> List[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L450)

<a id="mapping.data_integration.MappingDataIntegrationNode.entities_from_radar_marker"></a>

#### entities\_from\_radar\_marker

```python
def entities_from_radar_marker() -> List[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L488)

<a id="mapping.data_integration.MappingDataIntegrationNode.lanemarkings_callback"></a>

#### lanemarkings\_callback

```python
def lanemarkings_callback(data: MapMsg)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L527)

<a id="mapping.data_integration.MappingDataIntegrationNode.entities_from_lidar"></a>

#### entities\_from\_lidar

```python
def entities_from_lidar() -> List[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L531)

<a id="mapping.data_integration.MappingDataIntegrationNode.create_entities_from_clusters"></a>

#### create\_entities\_from\_clusters

```python
def create_entities_from_clusters(sensortype="") -> List[Entity]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L576)

<a id="mapping.data_integration.MappingDataIntegrationNode.create_hero_entity"></a>

#### create\_hero\_entity

```python
def create_hero_entity() -> Optional[Car]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L698)

<a id="mapping.data_integration.MappingDataIntegrationNode.publish_new_map_handler"></a>

#### publish\_new\_map\_handler

```python
def publish_new_map_handler()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L708)

<a id="mapping.data_integration.MappingDataIntegrationNode.publish_new_map"></a>

#### publish\_new\_map

```python
def publish_new_map()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L714)

Publishes a new map with the currently available data.

**Arguments**:

- `timer_event` __type_, optional_ - Defaults to None.

<a id="mapping.data_integration.MappingDataIntegrationNode.get_current_map_filters"></a>

#### get\_current\_map\_filters

```python
def get_current_map_filters() -> List[MapFilter]
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L794)

Creates an array of filters for the Map
based on the parameters of this node.

**Returns**:

- `List[MapFilter]` - Filters to apply to the Map

<a id="mapping.data_integration.main"></a>

#### main

```python
def main(args=None)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/data_integration.py#L820)

<a id="mapping.visualization"></a>

# mapping.visualization

<a id="mapping.visualization.MARKER_NAMESPACE"></a>

#### MARKER\_NAMESPACE: `str`

<a id="mapping.visualization.Visualization"></a>

## Visualization

```python
class Visualization(Node)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/visualization.py#L23)

The visualization for the intermediate layer.

This Node will publish the marker array composed of the different entities.

<a id="mapping.visualization.Visualization.__init__"></a>

#### \_\_init\_\_

```python
def __init__()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/visualization.py#L29)

<a id="mapping.visualization.Visualization.create_map_sub"></a>

#### create\_map\_sub

```python
def create_map_sub()
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/visualization.py#L91)

<a id="mapping.visualization.Visualization.map_callback"></a>

#### map\_callback

```python
def map_callback(data: MapMsg)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/visualization.py#L99)

<a id="mapping.visualization.Visualization.create_deleteall_marker"></a>

#### create\_deleteall\_marker

```python
def create_deleteall_marker() -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/visualization.py#L134)

Creates a marker that deletes all markers in RViz for this topic.

Prepend this to the MarkerArray to delete all markers
before the new ones are displayed.

<a id="mapping.visualization.main"></a>

#### main

```python
def main(args=None)
```

[[view_source]](/doc/mapping/../../code/mapping/mapping/visualization.py#L143)

