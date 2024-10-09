# Basic research perception

The perception is responsible for the efficient conversion of raw sensor and map data
into a useful environment representation that can be used by the planning for further processing.

This includes the classification and localization of relevant entities in traffic and also the preparation
of this data to enable a fast processing of this data in the planning layer.

## Interfaces

### Input

Output of the various implemented sensors plus additional HD map.

- **GNSS**: GPS sensor returning geo location data.
  - Output: [carla.GNSSMeasurement](https://carla.readthedocs.io/en/0.9.13/python_api/#carlagnssmeasurement)
    - altitude (float, meters): Height regarding ground level
    - latitude (float, degrees): North/South value of a point on the map
    - longitude (float degrees): West/East value of a point on the map
- **IMU**: 6-axis Inertial Measurement Unit
  - Output: [carla.IMUMeasurement](https://carla.readthedocs.io/en/0.9.13/python_api#carlaimumeasurement)
    - accelerometer (carla.Vector3D, m/s^2): Linear acceleration
    - compass (float, radians): Orientation with regard to the North ([0.0, -1.0, 0.0] in Unreal Engine).
    - gyroscope (carla.Vector3D, rad/s): Angular velocity
- **LIDAR**: Velodyne 64 LIDAR
  - Output: [carla.LidarMeasurement](https://carla.readthedocs.io/en/0.9.13/python_api#carlalidarmeasurement)
    - channels (int): Number of lasers shot
    - horizontal_angle (float, radians): Horizontal angle the LIDAR is rotated at the time of measurement
    - raw_data (bytes): Received list of 4D points. Each point consists of [x,y,z] coordiantes plus the intensity computed for that point
- **RADAR**: Long-range RADAR (up to 100 meters)
  - Output: [carla.RadarMeasurement](https://carla.readthedocs.io/en/0.9.13/python_api#carlaradarmeasurement)
    - raw_data (bytes): [carla.RadarDetection](https://carla.readthedocs.io/en/0.9.13/python_api/#carla.RadarDetection) Array
      - altitude (float, radians): Altitude angle of the detection
      - azimuth (float, radians): Horizontal angle of the detection
      - depth (float, meters): Distance from the sensor to the detection position
      - velocity (float, m/s): The velocity of the detected object towards the sensor
- **RGB Camera**: Regular camera that captures images
  - Output: [carla.Image](https://carla.readthedocs.io/en/0.9.13/python_api/#carlaimage)
    - fov (float, degrees): Horizontal field of view of the camera
    - height (int): Image height in pixels
    - width (int): Image width in pixels
    - raw_data (bytes)
- **Speedometer**: Pseudo-sensor that provides an approximation of the linear velocity
  - Output: (float, m/s)?
- (**OpenDRIVE map**: Pseudo-sensor that exposes the HD map in OpenDrive format parsed as a string)

### Output

Output of various sensors combined with detected objects (and predictions)

- GNSS
- IMU
- Speedometer
- (OpenDRIVE map)
- Traffic Lights (distance, state, direction)
- Vehicles (distance, speed, direction, lane)
- Pedestrians (distance, speed, crossing street?)
- Crosswalks (distance, Pedestrians waiting?)
- Obstacles (doors, poles)
- Lanes (direction, number, blocked)
- Intersection (lanes, directions)

## Environment

### What objects have to be detected?

- Traffic Lights
- Signs e.g. stop-sign
- Vehicles (cars, trucks, bicycles)
- Lanes + e.g. curbside, vegetation
- Intersections
- Crosswalks
- Pedestrians
- Poles, Fences, other obstacles

### Special case traffic light (PAF21-1)

- Traffic lights have yellow casings in some maps, which could cause the detection network to classify its
state always as orange
- On maps with european layout: It may happen that the traffic light cannot be detected, because it is too near
so the cameras won't "see" them

## Algorithms for classification/localization

- Object localization/classification:
  - CNN (HRNet, ResNet18 (PAF21-1), ...) + classifier (e.g. Support-Vector-Machines)
  - [YOLOv5](https://docs.ultralytics.com) (fastest, accurate, finished model for object detection, nice documentation)
  - [SSD](https://arxiv.org/pdf/1512.02325.pdf) (also finished model for object detection)
  - [AlexNet](https://en.wikipedia.org/wiki/AlexNet) (could be used for intersection-detection)
- Image segmentation
  - Pixel-oriented (not useful)
  - [Edge-oriented](https://www.tu-chemnitz.de/informatik/KI/edu/biver/ss2013/bild12_4_2.pdf)
  - [Region-oriented](https://www.tu-chemnitz.de/informatik/KI/edu/biver/ss2013/bild12_4_1.pdf)
  - Cluster-based (not useful)
  - Model-based (computationally expensive -> not useful)
  - [RefineNet](https://arxiv.org/pdf/1611.06612.pdf) (finished CNN for image segmentation, also used [here](https://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf))

## Prediction

For route planing it is necessary to predict further behaviour of some entities to avoid collisions.

- Prediction of lane switching vehicles on basis of indicator
- Prediction of crossing pedestrians when walking in the direction of a crosswalk or on the street
between parking vehicles

## Map data

Todo: Find information about map data

## Limitations of the sensors and perception

### LIDAR

- No range limits found in documentation
- Hidden objects can't be detected
- Colors can't be detected
- Objects can't be classified

### RADAR

- No range limits found in documentation (default range is 100m, so probably several hundreds of meters)
- Hidden objects can't be detected
- Colors can't be detected
- Objects can't be classified

### Camera

- No direct range limitations. But for long range detection, a high resolution is required,
 which results in a small FOV to reduce computation time
- Hidden objects can be partially detected (e.g. a pedestrian's head protruding over a car)
- Colors can be detected (e.g. traffic lights)
- Image segmentation possible

## Training data

The best would be to generate training data in carla simulator.
Using pre-build real-world datasets would be less effort, but the nets accuracy would be lower.

## Classification of situations

Should be part of the planning layer in my opinion. Perception should contain only
evaluation of sensor data and detection of objects.

## Combination of 2D camera data and 3D RADAR/LIDAR data

Useful and necessary to combine information about the kind of detected object (e.g. other vehicle) and its position (distance, relative speed etc.).

The output should be an image containing the classified objects plus their related RADAR/LIDAR data

Todo: Find algorithms for fusion
