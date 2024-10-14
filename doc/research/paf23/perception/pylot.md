# Pylot - Perception

**Summary:** This page contains the research into the perception component of pylot.

- [Pylot - Perception](#pylot---perception)
  - [Detection](#detection)
    - [Obstacle detection](#obstacle-detection)
    - [Traffic light detection](#traffic-light-detection)
    - [Lane detection](#lane-detection)
  - [Obstacle Tracking](#obstacle-tracking)
  - [Depth Estimation](#depth-estimation)
  - [Segmentation](#segmentation)
  - [Lidar](#lidar)

## [Detection](https://pylot.readthedocs.io/en/latest/perception.detection.html)

### Obstacle detection

Pylot provides two options for obstacle detection:

1. Obstacle detection operator that can use any model that adheres to the Tensorflow `object detection model zoo`
   - By default, three models that were trained on 1080p CARLA images (`faster-rcnn`, `ssd-mobilenet-fpn-640`, and `ssdlit-mobilenet-v2`) are provided
   - Models that have been trained on other data sets can be easily plugged in
2. Operator that can infer any of the EfficientDet models (not trained on CARLA data, but on the COCO dataset)

### Traffic light detection

Uses `Faster RCNN weight` (trained on 1080p CARLA images)

### Lane detection

Uses the `Lanenet` model ([repo](https://github.com/MaybeShewill-CV/lanenet-lane-detection)) or canny edge detector.

---

## [Obstacle Tracking](https://pylot.readthedocs.io/en/latest/perception.tracking.html)

For tracking obstacles across frames.
Uses the `DaSiamRPN` model ([repo](https://github.com/foolwood/DaSiamRPN)) to serially track multiple obstacles.

---

## [Depth Estimation](https://pylot.readthedocs.io/en/latest/perception.depth_estimation.html)

Uses stereo cameras to estimate depth with the `AnyNet` model ([repo](https://github.com/mileyan/AnyNet)).
Configurable camera distance between left and right.

---

## [Segmentation](https://pylot.readthedocs.io/en/latest/perception.segmentation.html)

Different approaches, such as using the `DRN` model ([repo](https://github.com/ICGog/drn)) for segmenting camera images.
No model with training on CARLA data available, output of segmentation component not used in Pylot right now.

---

## [Lidar](https://github.com/erdos-project/pylot/blob/master/pylot/perception/point_cloud.py)

Pylot contains a few helpful function for handling point clouds from the Lidar sensor.
It can for example merge two point clouds or map the points to a camera image.
