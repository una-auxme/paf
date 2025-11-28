<!-- markdownlint-disable -->
# Markers documentation

## Table of Contents

* [mapping\_common.markers](#mapping_common.markers)
  * [debug\_marker](#mapping_common.markers.debug_marker)
  * [debug\_marker\_array](#mapping_common.markers.debug_marker_array)

<a id="mapping_common.markers"></a>

# mapping\_common.markers

Contains functions to easily create debug markers that can be visualized in RViz

**[API documentation](/doc/mapping/generated/mapping_common/markers.md)**

Overview of the main components:
- debug_marker(): Creates a ROS Marker based on different objects
- debug_marker_array(): Creates a ROS MarkerArray
  based on list of ROS Markers

<a id="mapping_common.markers.debug_marker"></a>

#### debug\_marker

```python
def debug_marker(base: Any,
                 frame_id: Optional[str] = "hero",
                 position_z: Optional[float] = None,
                 transform: Optional[Transform2D] = None,
                 offset: Optional[Vector2] = None,
                 color: Optional[Tuple[float, float, float, float]] = None,
                 scale_z: Optional[float] = None) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/markers.py#L28)

Creates a ROS Marker based on *base*

**Arguments**:

- `base` _Any_ - Currently supported: Entity, Shape2D, shapely.Polygon,
  shapely.LineString, Marker, str, Point2, [Point2, Point2] as Arrow
- `frame_id` _Optional[str], optional_ - Defaults to "hero".
- `position_z` _Optional[float], optional_ - Defaults to None.
  If None, the z position of base will be used.
- `transform` _Optional[Transform2D], optional_ - Defaults to None.
  If None, the transform of base will be used.
- `offset` _Optional[Vector2], optional_ - Offset Vector.
  Added to the position of the marker. Defaults to None.
  color (Optional[Tuple[float, float, float, float]], optional):
  (r, g, b, a) color tuple. Defaults to (0.5, 0.5, 0.5, 0.5).
- `scale_z` _Optional[float], optional_ - Defaults to None.
  If None, the scale of base will be used.
  If the scale.z of base is also 0.0,
  the scale will be set to 1.0 (and 0.3 for str)
  

**Raises**:

- `TypeError` - If the type of base is unsupported
  

**Returns**:

- `Marker` - Marker

<a id="mapping_common.markers.debug_marker_array"></a>

#### debug\_marker\_array

```python
def debug_marker_array(namespace: str,
                       markers: List[Marker],
                       timestamp: TimeMsg,
                       lifetime: Optional[DurationMsg] = None) -> MarkerArray
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/markers.py#L146)

Builds a ROS MarkerArray based on *markers*

**Arguments**:

- `namespace` _str_ - Namespace of the markers
- `markers` _List[Marker]_ - markers
- `timestamp` _builtin_interfaces.msg.Time_ - Timestamp of all markers.
- `lifetime` _Optional[builtin_interfaces.msg.Duration], optional_ - Marker lifetime.
  Defaults to 0.5.
  

**Returns**:

- `MarkerArray` - MarkerArray

