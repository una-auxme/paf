# Traffic Light Detection

## Vision Node

For each analyzed image, it is checked whether an object with the ID=9 (traffic light) is detected.
If that is the case, `process_traffic_lights()` is called which applies the bounding box of the predicition to cut out the found object (e.g. traffic light).

Only if the object is within `min_x`, `max_x` and `min_prob` (probability), it will be published to `"/paf/{self.role_name}/{self.side}/segmented_traffic_light"`.

## TrafficLightNode

The `traffic_light_node.py` file is part of a larger system that handles traffic light detection. It contains a class `TrafficLightNode` that extends from `CompatibleNode`.

This class is responsible for setting up the traffic light detection system and handling the incoming camera images.

### Initialization

The `TrafficLightNode` class is initialized with a name and additional keyword arguments. During initialization, it sets up the following:

- A `CvBridge` instance for converting between ROS image messages and OpenCV images.
- The role name and side, which are parameters that can be set externally.
- A `TrafficLightInference` instance for performing traffic light detection.

### Methods

#### `setup_camera_subscriptions()`

This method sets up a subscription to the camera images. It subscribes to the topic `"/paf/{self.role_name}/{self.side}/segmented_traffic_light"` and calls the `handle_camera_image` method whenever a new image message is received.

#### `setup_traffic_light_publishers()`

This method sets up a publisher for the traffic light state. It publishes to the topic `"/paf/{self.role_name}/{self.side}/traffic_light_state"` in the format of `TrafficLightState.msg` which uses an int8-based enum for the traffic light state.

#### `handle_camera_image(image)`

This method is called whenever a new image message is received. It performs traffic light detection by using `traffic_light_inference.py` on the image and publishes the result.
The result is a `TrafficLightState` message where the state is set to the detected traffic light state (1 for green, 2 for red, 4 for yellow, 0 for unknown).
