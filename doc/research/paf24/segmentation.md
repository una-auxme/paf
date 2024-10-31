# Segmentation

**Summary:** This page contains the research into the segmentation component of Autoware.

- [Segmentation](#Segmentation)
  - [Already implemended solutions](#1.-Already-implemented-solutions)
  - [Implemented but dropped](#2Implemented-but-dropped)
  - [Carla Sensors](#3-Carla-Sensors)
  - [Follow up Question](#4-Follow-up-Question)

## Already implemented solutions

https://github.com/una-auxme/paf/blob/c3011ee70039e199e106c54aa162a8f52be241a6/code/perception/launch/perception.launch?plain=1#L59-L61

probably trained with the generated dataset:
https://github.com/una-auxme/paf/blob/8e8f9a1a03ae09d5ac763c1a11b398fc1ce144b0/code/perception/src/dataset_generator.py#L109-L110

## Implemented but dropped:
https://github.com/una-auxme/paf/blob/8c968fb5c6c44c15b2733c5a181c496eb9b244be/doc/perception/efficientps.md#efficientps

## Carla Sensors:
https://carla.readthedocs.io/en/0.8.4/cameras_and_sensors/
```
camera = carla.sensor.Camera('MyCamera', PostProcessing='SemanticSegmentation')
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(camera)
```
![Alt text](https://github.com/una-auxme/paf/blob/368-visionnode-and-segmentation/doc/assets/perception/Carla_Segmentation_Sensor.png)

there is another solution implemented by the carla simulator:
https://carla.readthedocs.io/en/0.9.14/ref_sensors/#semantic-segmentation-camera:~:text=the%20object%20it.-,Semantic%20segmentation%20camera,-Blueprint%3A%20sensor

for more context:
the pedestrian walks will be labeld as roadlines

## Follow up Question:
Why the last group used bounding boxes and not the segmentation model is it to slow or maybe not reliable?
