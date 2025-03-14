# mapping_common.markers module

- [mapping_common.markers module]()
  - [`debug_marker()`](#mapping_common.markers.debug_marker)
  - [`debug_marker_array()`](#mapping_common.markers.debug_marker_array)

<a id="mapping_common.markers.debug_marker"></a>

### mapping_common.markers.debug_marker(base, frame_id='hero', position_z=None, transform=None, offset=None, color=None, scale_z=None)

Creates a marker based on *base*

* **Return type:**
  `Marker`

Args:
: base (Any): Currently supported: Entity, Shape2D, shapely.Polygon,
  : shapely.LineString, Marker, str, Point2, [Point2, Point2] as Arrow
  <br/>
  frame_id (Optional[str], optional): Defaults to “hero”.
  position_z (Optional[float], optional): Defaults to None.
  <br/>
  > If None, the z position of base will be used.
  <br/>
  transform (Optional[Transform2D], optional): Defaults to None.
  : If None, the transform of base will be used.
  <br/>
  offset (Optional[Vector2], optional): Offset Vector.
  : Added to the position of the marker. Defaults to None.
  <br/>
  color (Optional[Tuple[float, float, float, float]], optional):
  : (r, g, b, a) color tuple. Defaults to (0.5, 0.5, 0.5, 0.5).
  <br/>
  scale_z (Optional[float], optional): Defaults to None.
  : If None, the scale of base will be used.
    : If the scale.z of base is also 0.0,
      the scale will be set to 1.0 (and 0.3 for str)

Raises:
: TypeError: If the type of base is unsupported

Returns:
: Marker

<a id="mapping_common.markers.debug_marker_array"></a>

### mapping_common.markers.debug_marker_array(namespace, markers, timestamp=None, lifetime=None)

Builds a MArkerArray based on *markers*

* **Return type:**
  `MarkerArray`

Args:
: namespace (str): Namespace of the markers
  markers (List[Marker])
  timestamp (Optional[rospy.Time], optional): Timestamp of all markers.
  <br/>
  > Defaults to None. If None, the current ros time will be used
  <br/>
  lifetime (Optional[rospy.Duration], optional): Marker lifetime.
  : Defaults to 0.5.

Returns:
: MarkerArray
