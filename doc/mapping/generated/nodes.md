<!-- markdownlint-disable -->
# Nodes

## Table of Contents

* [mapping\_data\_integration](#mapping_data_integration)
  * [MappingDataIntegrationNode](#mapping_data_integration.MappingDataIntegrationNode)
    * [lidar\_data](#mapping_data_integration.MappingDataIntegrationNode.lidar_data)
    * [hero\_speed](#mapping_data_integration.MappingDataIntegrationNode.hero_speed)
    * [lidar\_clustered\_points\_data](#mapping_data_integration.MappingDataIntegrationNode.lidar_clustered_points_data)
    * [radar\_clustered\_points\_data](#mapping_data_integration.MappingDataIntegrationNode.radar_clustered_points_data)
    * [vision\_clustered\_points\_data](#mapping_data_integration.MappingDataIntegrationNode.vision_clustered_points_data)
    * [stop\_marks](#mapping_data_integration.MappingDataIntegrationNode.stop_marks)
    * [current\_pos](#mapping_data_integration.MappingDataIntegrationNode.current_pos)
    * [current\_heading](#mapping_data_integration.MappingDataIntegrationNode.current_heading)
    * [\_\_init\_\_](#mapping_data_integration.MappingDataIntegrationNode.__init__)
    * [dynamic\_reconfigure\_callback](#mapping_data_integration.MappingDataIntegrationNode.dynamic_reconfigure_callback)
    * [update\_stopmarks\_callback](#mapping_data_integration.MappingDataIntegrationNode.update_stopmarks_callback)
    * [heading\_callback](#mapping_data_integration.MappingDataIntegrationNode.heading_callback)
    * [current\_pos\_callback](#mapping_data_integration.MappingDataIntegrationNode.current_pos_callback)
    * [hero\_speed\_callback](#mapping_data_integration.MappingDataIntegrationNode.hero_speed_callback)
    * [lidar\_clustered\_points\_callback](#mapping_data_integration.MappingDataIntegrationNode.lidar_clustered_points_callback)
    * [radar\_clustered\_points\_callback](#mapping_data_integration.MappingDataIntegrationNode.radar_clustered_points_callback)
    * [vision\_clustered\_points\_callback](#mapping_data_integration.MappingDataIntegrationNode.vision_clustered_points_callback)
    * [lidar\_callback](#mapping_data_integration.MappingDataIntegrationNode.lidar_callback)
    * [entities\_from\_lidar\_marker](#mapping_data_integration.MappingDataIntegrationNode.entities_from_lidar_marker)
    * [entities\_from\_radar\_marker](#mapping_data_integration.MappingDataIntegrationNode.entities_from_radar_marker)
    * [lanemarkings\_callback](#mapping_data_integration.MappingDataIntegrationNode.lanemarkings_callback)
    * [entities\_from\_lidar](#mapping_data_integration.MappingDataIntegrationNode.entities_from_lidar)
    * [create\_entities\_from\_clusters](#mapping_data_integration.MappingDataIntegrationNode.create_entities_from_clusters)
    * [create\_hero\_entity](#mapping_data_integration.MappingDataIntegrationNode.create_hero_entity)
    * [publish\_new\_map\_handler](#mapping_data_integration.MappingDataIntegrationNode.publish_new_map_handler)
    * [publish\_new\_map](#mapping_data_integration.MappingDataIntegrationNode.publish_new_map)
    * [get\_current\_map\_filters](#mapping_data_integration.MappingDataIntegrationNode.get_current_map_filters)

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

<a id="mapping_data_integration.MappingDataIntegrationNode.lidar_data"></a>

#### lidar\_data: `Optional[PointCloud2]`

<a id="mapping_data_integration.MappingDataIntegrationNode.hero_speed"></a>

#### hero\_speed: `Optional[CarlaSpeedometer]`

<a id="mapping_data_integration.MappingDataIntegrationNode.lidar_clustered_points_data"></a>

#### lidar\_clustered\_points\_data: `Optional[ClusteredPointsArray]`

<a id="mapping_data_integration.MappingDataIntegrationNode.radar_clustered_points_data"></a>

#### radar\_clustered\_points\_data: `Optional[ClusteredPointsArray]`

<a id="mapping_data_integration.MappingDataIntegrationNode.vision_clustered_points_data"></a>

#### vision\_clustered\_points\_data: `Optional[ClusteredPointsArray]`

<a id="mapping_data_integration.MappingDataIntegrationNode.stop_marks"></a>

#### stop\_marks: `Dict[str, List[StopMark]]`

StopMarks from the UpdateStopMarks service

**Important:** the transform of these entities uses global coordinates

<a id="mapping_data_integration.MappingDataIntegrationNode.current_pos"></a>

#### current\_pos: `Optional[Point2]`

<a id="mapping_data_integration.MappingDataIntegrationNode.current_heading"></a>

#### current\_heading: `Optional[float]`

<a id="mapping_data_integration.MappingDataIntegrationNode.__init__"></a>

#### \_\_init\_\_

```python
def __init__(name, **kwargs)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L63)

<a id="mapping_data_integration.MappingDataIntegrationNode.dynamic_reconfigure_callback"></a>

#### dynamic\_reconfigure\_callback

```python
def dynamic_reconfigure_callback(config: "MappingIntegrationConfig", level)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L157)

All currently used reconfigure options are queried dynamically.

The configuration options are located
[here](/code/mapping/launch/mapping.launch)

<a id="mapping_data_integration.MappingDataIntegrationNode.update_stopmarks_callback"></a>

#### update\_stopmarks\_callback

```python
def update_stopmarks_callback(
        req: UpdateStopMarksRequest) -> UpdateStopMarksResponse
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L171)

<a id="mapping_data_integration.MappingDataIntegrationNode.heading_callback"></a>

#### heading\_callback

```python
def heading_callback(data: Float32)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L192)

<a id="mapping_data_integration.MappingDataIntegrationNode.current_pos_callback"></a>

#### current\_pos\_callback

```python
def current_pos_callback(data: PoseStamped)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L195)

<a id="mapping_data_integration.MappingDataIntegrationNode.hero_speed_callback"></a>

#### hero\_speed\_callback

```python
def hero_speed_callback(data: CarlaSpeedometer)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L198)

<a id="mapping_data_integration.MappingDataIntegrationNode.lidar_clustered_points_callback"></a>

#### lidar\_clustered\_points\_callback

```python
def lidar_clustered_points_callback(data: ClusteredPointsArray)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L201)

<a id="mapping_data_integration.MappingDataIntegrationNode.radar_clustered_points_callback"></a>

#### radar\_clustered\_points\_callback

```python
def radar_clustered_points_callback(data: ClusteredPointsArray)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L204)

<a id="mapping_data_integration.MappingDataIntegrationNode.vision_clustered_points_callback"></a>

#### vision\_clustered\_points\_callback

```python
def vision_clustered_points_callback(data: ClusteredPointsArray)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L207)

<a id="mapping_data_integration.MappingDataIntegrationNode.lidar_callback"></a>

#### lidar\_callback

```python
def lidar_callback(data: PointCloud2)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L210)

<a id="mapping_data_integration.MappingDataIntegrationNode.entities_from_lidar_marker"></a>

#### entities\_from\_lidar\_marker

```python
def entities_from_lidar_marker() -> List[Entity]
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L213)

<a id="mapping_data_integration.MappingDataIntegrationNode.entities_from_radar_marker"></a>

#### entities\_from\_radar\_marker

```python
def entities_from_radar_marker() -> List[Entity]
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L251)

<a id="mapping_data_integration.MappingDataIntegrationNode.lanemarkings_callback"></a>

#### lanemarkings\_callback

```python
def lanemarkings_callback(data: MapMsg)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L290)

<a id="mapping_data_integration.MappingDataIntegrationNode.entities_from_lidar"></a>

#### entities\_from\_lidar

```python
def entities_from_lidar() -> List[Entity]
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L294)

<a id="mapping_data_integration.MappingDataIntegrationNode.create_entities_from_clusters"></a>

#### create\_entities\_from\_clusters

```python
def create_entities_from_clusters(sensortype="") -> List[Entity]
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L342)

<a id="mapping_data_integration.MappingDataIntegrationNode.create_hero_entity"></a>

#### create\_hero\_entity

```python
def create_hero_entity() -> Optional[Car]
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L461)

<a id="mapping_data_integration.MappingDataIntegrationNode.publish_new_map_handler"></a>

#### publish\_new\_map\_handler

```python
def publish_new_map_handler(timer_event=None)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L472)

<a id="mapping_data_integration.MappingDataIntegrationNode.publish_new_map"></a>

#### publish\_new\_map

```python
def publish_new_map(timer_event=None)
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L478)

<a id="mapping_data_integration.MappingDataIntegrationNode.get_current_map_filters"></a>

#### get\_current\_map\_filters

```python
def get_current_map_filters() -> List[MapFilter]
```

[[view_source]](/code/mapping/src/mapping_data_integration.py#L539)

