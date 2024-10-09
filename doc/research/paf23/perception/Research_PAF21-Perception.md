# Sprint 0: Research Samuel Kühnel

## PAF 21-2

### Perception

### Obstacle detection

- Detect objects via semantic lidar sensor
- Provides x and y coordinates, as well as distance value
- Additional information on position change in a time interval → Calculation of speed possible
- Detects the object and returns either the value "Vehicle" or "Pedestrian"

### TrafficLightDetection

![diagramm.png](https://github.com/ll7/paf21-2/raw/main/docs/imgs/trafficlightdetection_diagram.jpg)

- **FusionCamera** saves images from **RGBCamera** and **DepthCamera** with timestamp and then synchronizes with **SegmentationCamera**
- Neural network based on [ResNet18](https://pytorch.org/hub/pytorch_vision_resnet/) (predefined PyTorch network)
- Generally only traffic lights up to 100m distance
- Canny algorithm to filter contours

### Problems and solutions

- Red background distorts traffic light phase detection → **Solution**: Narrow section of the traffic light image for phase detection
- Yellow painted traffic lights distort traffic light phase detection → **Solution**: Filter out red and green sections beforehand using masks and convert remaining image to grayscale and add masks again.
- **Problem without solution**: European traffic lights can sometimes not be recognized at the stop line.

## Resumee

### Perception

- Status quo: LIDAR Sensor and Big Neural Network
- Possible focus: Using smaller (pretrained) models to improve overall performance fast
- Taking known issues into account

### Planning

- Currently decision tree for evaluating the current position
- Trying out different heuristics → already given as repo
