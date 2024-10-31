
# Segmentation

**Summary:** This page contains the research into the segmentation component of Autoware.

- [Segmentation](#Segmentation)
  - [Already implemented solutions](#already-implemented-solutions)
  - [Implemented but dropped](#implemented-but-dropped)
  - [Carla Sensors](#carla-sensors)
  - [Follow-up Question](#follow-up-question)

## Already implemented solutions

https://github.com/una-auxme/paf/blob/c3011ee70039e199e106c54aa162a8f52be241a6/code/perception/launch/perception.launch?plain=1#L59-L61

probably trained with the generated dataset:
https://github.com/una-auxme/paf/blob/8e8f9a1a03ae09d5ac763c1a11b398fc1ce144b0/code/perception/src/dataset_generator.py#L109-L110

## Implemented but dropped:

https://github.com/una-auxme/paf/blob/8c968fb5c6c44c15b2733c5a181c496eb9b244be/doc/perception/efficientps.md#efficientps

## Carla Sensors:

![Alt text](https://carla.readthedocs.io/en/0.9.14/img/ref_sensors_semantic.jpg)
![Alt text](https://carla.readthedocs.io/en/0.9.14/img/tuto_sem.jpg)

for more context:
the pedestrian walks will be labled as roadlines

example:

```

# --------------
# Add a new semantic segmentation camera to my ego
# --------------
sem_cam = None
sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
sem_bp.set_attribute("image_size_x",str(1920))
sem_bp.set_attribute("image_size_y",str(1080))
sem_bp.set_attribute("fov",str(105))
sem_location = carla.Location(2,0,1)
sem_rotation = carla.Rotation(0,180,0)
sem_transform = carla.Transform(sem_location,sem_rotation)
sem_cam = world.spawn_actor(sem_bp,sem_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
# This time, a color converter is applied to the image, to get the semantic segmentation view
sem_cam.listen(lambda image: image.save_to_disk('tutorial/new_sem_output/%.6d.jpg' % image.frame,carla.ColorConverter.CityScapesPalette))

```

For more information:
https://carla.readthedocs.io/en/0.9.14/ref_sensors/#semantic-segmentation-camera:~:text=the%20object%20it.-,Semantic%20segmentation%20camera,-Blueprint%3A%20sensor

https://carla.readthedocs.io/en/0.9.14/tuto_G_retrieve_data/#semantic-segmentation-camera:~:text=on%20the%20right.-,Semantic%20segmentation%20camera,-The%20semantic%20segmentation

## Follow up Question:

Why the last group used bounding boxes and not the segmentation model is it to slow or maybe not reliable?
