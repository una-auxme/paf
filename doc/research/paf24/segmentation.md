Already implemented solutions:

https://github.com/una-auxme/paf/blob/c3011ee70039e199e106c54aa162a8f52be241a6/code/perception/launch/perception.launch?plain=1#L59-L61

Carla Sensor:
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


# Questions:
Is the already implemented solution using the "Sementic Sensor"?

How to convert the Carla Sementic Sensor into our code? (https://github.com/una-auxme/paf/blob/main/code/agent/src/agent/agent.py)

How are crosswalks, signs and traffic lights classiefied in the output from Carla's semantic sensor?(Create output in the simulation with sementic sensor)

How the yolo8x-seg and deeplabv3_resnet101 was trained?
