# Traffic Light Detection

## Vision Node

For each analyzed image, it is checked whether an object with the ID=9 (traffic light) is detected.
If that is the case, `process_traffic_lights()` is called which applies the bounding box of the predicition to cut out the found object (e.g. traffic light).

Only if the object is in the upper half of the image and `min_prob` (probability, currently set to 30%) as well as the height being at least 1.5x of the width, it will be published to `"/paf/{self.role_name}/{self.side}/segmented_traffic_light"`.

## TrafficLightNode

The `traffic_light_node.py` file handles traffic light detection itself, meaning that it does the actual recognition. It contains a class `TrafficLightNode` that extends from `CompatibleNode`.

This class is responsible for setting up the traffic light detection system and handling the incoming camera images.

### Attributes

- `bridge`: An instance of CvBridge for converting between ROS image messages and OpenCV images.
- `role_name`: The role name of the node, default is "hero".
- `side`: The side of the node, default is "Center".
- `classifier`: An instance of TrafficLightInference for traffic light detection (CNN module).
- `last_info_time`: The time of the last information received.
- `last_state`: The last state of the traffic light.
- `traffic_light_publisher`: A publisher for traffic light state messages.
- `traffic_light_distance_publisher`: A publisher for traffic light distance messages.

### Methods

- `__init__(self, name, **kwargs)`: Initializes the node, sets up publishers and subscribers, and starts a thread for auto invalidation of traffic light state.
- `setup_camera_subscriptions(self)`: Sets up a subscription to the segmented traffic light image topic.
- `setup_traffic_light_publishers(self)`: Sets up publishers for traffic light state and distance.
- `auto_invalidate_state(self)`: Runs in a separate thread and invalidates the traffic light state if no new information has been received for 2 seconds.
- `handle_camera_image(self, image)`: Callback for the image subscription. Converts the image to RGB, infers the traffic light state, and publishes the state and distance if the state has changed.
- `run(self)`: Spins the node to handle callbacks.

### Functions

- `get_light_mask(image)`: Returns a binary mask where the pixels within the hue, saturation, and value bounds for red, yellow, and green are white, and all other pixels are black.
- `is_front(image)`: Returns `True` if the largest contour in the light mask has an aspect ratio within the range of a square (therefore a circle), and `False` otherwise.

### Usage

This script is intended to be used as a ROS node in the Carla ROS system.

## Filtering of images

### 1. Vision Node

Objects, which are detected as traffic light by the RTDETR-L model (or others), must fulfill the following criterias to be published:

- At least a 30% (0.30) certainty/probablity of the classification model.
- More than 1.5x as tall (height) as it is wide (width).
- Above 360px (upper half of the 1280x720 image).

### 2. Traffic Light Node

Objects, which are published by the Vision Node, are further filtered by the following criterias:

- Classification probabilities of "Unknown" and "Side" are either both below 1e-10 or one of both are below 1e-15.
- "Side" is treated as "Unknown".
- Valid states (Red, Green, Yellow) must be present at least twice in a row to be actually published.
- A state decays (state=0; "Unknown") after 2 seconds if there is no new info in the meantime.
- Filter out side-facing traffic lights by analysing their light shape (oval vs. round) using HSV + contour analysis.
