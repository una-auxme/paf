<!-- markdownlint-disable -->
# Table of Contents

* [mapping\_common.markers](#mapping_common.markers)
  * [debug\_marker](#mapping_common.markers.debug_marker)
  * [debug\_marker\_array](#mapping_common.markers.debug_marker_array)

<a id="mapping_common.markers"></a>

# mapping\_common.markers

<a id="mapping_common.markers.debug_marker"></a>

#### debug\_marker(base: Any, frame\_id: Optional[str] = "hero", position\_z: Optional[float] = None, transform: Optional[Transform2D] = None, offset: Optional[Vector2] = None, color: Optional[Tuple[float, float, float, float]] = None, scale\_z: Optional[float] = None)

```python
def debug_marker(base: Any,
                 frame_id: Optional[str] = "hero",
                 position_z: Optional[float] = None,
                 transform: Optional[Transform2D] = None,
                 offset: Optional[Vector2] = None,
                 color: Optional[Tuple[float, float, float, float]] = None,
                 scale_z: Optional[float] = None) -> Marker
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/markers.py#L15)

Creates a marker based on *base*

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

  Marker

<a id="mapping_common.markers.debug_marker_array"></a>

#### debug\_marker\_array(namespace: str, markers: List[Marker], timestamp: Optional[rospy.Time] = None, lifetime: Optional[rospy.Duration] = None)

```python
def debug_marker_array(
        namespace: str,
        markers: List[Marker],
        timestamp: Optional[rospy.Time] = None,
        lifetime: Optional[rospy.Duration] = None) -> MarkerArray
```

[[view_source]](/doc/mapping/../../code/mapping/ext_modules/mapping_common/markers.py#L130)

Builds a MArkerArray based on *markers*

**Arguments**:

- `namespace` _str_ - Namespace of the markers
  markers (List[Marker])
- `timestamp` _Optional[rospy.Time], optional_ - Timestamp of all markers.
  Defaults to None. If None, the current ros time will be used
- `lifetime` _Optional[rospy.Duration], optional_ - Marker lifetime.
  Defaults to 0.5.
  

**Returns**:

  MarkerArray

