# Autoware Perception

## 1.Architecture

![image](https://github.com/una-auxme/paf/assets/102369315/6b3fb964-e650-442a-a674-8e0471d931a9)

Focus on:

- Object-Detection
- Object-Tracking
- Movement-Predictioon
- TrafficLight-Detection
- TrafficLight-Classification

## 2.Detection Mechanisms

Autowares perception is very complex and uses a variety of mechnaism to gather as much information as possible about the surroundings of the car.

![image](https://github.com/una-auxme/paf/assets/102369315/23f9699e-85c7-44c6-b9fa-a603dc73afcf)

For the perception Autoware mainly uses the following Sensors:

- LIDAR
- RGB-Camera
- Radar

Autoware has a giant repo of detection mechanism which implemtents the following methods and way more:

- TrafficLight-Detection
- LIDAR Segmentations
- Occupancy Grids
- Radar Object-Tracking
- Single-Shot-Detectors
- ....

Some more detailed docs can be found [here](https://autowarefoundation.github.io/autoware.universe/main/perception/traffic_light_classifier/).

## 3. Conclusion

Overall Autoware is very complex and not well enough documented to grasp every single concept in detail on the first look. Since they implemented a lot of different mechanisms,
Autoware can be considered as a good reference if youre looking for ideas.
