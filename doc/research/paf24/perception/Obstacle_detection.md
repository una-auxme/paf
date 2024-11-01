# Research: Why does the car hit obstacles

## Summary of current obstacle detection in perception:
- [Object detection](#Object-detection)
- [Distance calculation](#Distance-calculation)
- [Publishing of Outputs](#Publishing-of-Outputs)

For more details, please refer to the current documentation.

## Object detection:
Active sensors:
- Center camera
- LIDAR

Inactive sensors:
- Cameras: right, left, back
- Radar

## Distance calculation
- The vision_node receives depth-images from the lidar_distance_node for the specified camera angle

- According to the distance-to-objects documentation, the LIDAR sensor's 3D values are projected onto a 2D image that matches the exact dimensions of the camera image. 
Numpy formulas are used to reconstruct the depth image, with distances in meters represented as pixel values (grayscale image). 
If no distance is found in the depth image, “infinity” is returned for this bounding box.

- The LIDAR sensor may flicker; a higher spin rate of the LIDAR provieds fully reconstructed depth images but with lower resolution for the LIDAR points

## Publishing of Outputs:
- Class_index
- Min_X
- Min_Abs_y

## Dependencies of detected and hit obstacles:

- Are all obstacles detected? No
- Does successful object detection depend on:
    - Speed of the car? No
    - Size of obstacles? No
    - Type of obstacle? No
    - Dynamic/static obstacle? No

## Important aspects found during research:

- Sometimes objects are not detected (sometimes e.g.: cyclists, construction site signs, open car doors)
- Most object that are hit are detected
- Sometimes distance calculation is incorrect (distance is set to infinity just before the collision)
- Distance in x is alway > ~0.5 (is that a problem?), sometimes distance to a collided car is greater than the distance to a parked car on the side of the road
- Significant issues in dead-end situations: -> 1.2 seconds of reversing -> the car often immediately collides with the obstacle again
