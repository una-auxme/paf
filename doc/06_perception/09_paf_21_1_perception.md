# Paf_21_1 - Perception

## 1. Architecture

![image](https://github.com/una-auxme/paf23/assets/102369315/07328c78-83d7-425c-802e-8cc49430e6c1)

**Key Features **

- Object-Detection
- TrafficLight-Detection

## 2. Sensors

- Position
- Velocity
- RGB-Image
- Depth-Image
- Semantic-Image

## 3. Object-Detection

- Objects are detected using semantic and depth images
- First they use masks and filters on the semantic image to identify pedestrians and vehicles
- Then they use the depth image and the position and velocity data to calculate the relative positions of each object

## 4. TrafficLight-Detection

- They use the semantic image and apply masks and filters to find traffic lights
- Then they use this information anlognside with the depth image to cut out a specific patch of the image
- After that they cut the same patch from the RGB Image (32x32x3) which is than classified by a simple Convolutional Neural Network (CNN)
- They use the classes Backside, Green, Yellow and Red

## 5. Conclusion

- Some parts of their systems, like the CNN, are probably usefull to us
- Since their entire perception is based on semantic and depth images it will be vewry hard for us to adopt their concepts
- We would have to build our own semantic and depth cameras which is a very complex task
- All in all the concepts of paf_21_1 regarding the perception aren´t to usefull for our project
