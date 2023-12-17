# Getting the Distance to Objects

Using the vision node and the lidar distance node we can calculate the distance of detected objects.
We can solve this problem from two directions mapping either pixel into the 3D-World or mapping 3D-LidarPoints into Pixel.

This file will first discuss a simple Minimum Distance Publisher and will than explain the mapping of 3D-Points into 2D.

## Minimum Distance

This part of the node analysis the LIDAR-Data in a small corridor right in front of the vehicle.
This corridor is similar to what is explained in 03_lidar_distance_utility.md

We retrieve the (X, Y, Z) coordinates of every LIDAR-Point in this corridor.
We than check for the nearest cluster of Points, where there are at least 5 LIDAR-Points within a Range of 0.2 Meters.
That way we dont accidently classify a sensor error or a tiny object as an obstacle.

We publish a 3D-Graph of the analysis on the topic: "/paf/hero/Center/min_dist_image"

## Converting 3D-Points into 2D-Camera-Space

As said before we can try this from both directions. I chose to do this starting with the LIDAR-Data.
There are two reaons for this.

**1. Resolution**
The resolution of the LIDAR-Sensor is not as high as the Camera Resolution.
This means, that not every Pixel in the Camera-Image will have a corresponding Data-Point in the LIDAR-Pointcloud.

**2. Math**
The literature for Camera-Calibration proposes a way to calculate Pixel from a 3D-World Coordinate System.
You can only partially invert this formula, since it involves calculation the invers matrix for a matrix that isnt square.

I found ways online, that seemed to solve this issue though.

### Concept

![3d_2d_porjection](../00_assets/3d_2d_projection.png)

The goal is to calculate the projection of point P and find its Pixl-Coordinates (u,v) on the Image-Plain.
To do this you need a couple of thins:

1. Camera-Calibration
   1. Width
   2. Height
   3. Field-of-View
2. LIDAR-Sensor-Position
   1. Origin of 3D-World-Coordinates
3. Camera-Sensor
   1. Position
   2. Orientation

The formula for this projection proposed by the literature looks like this:

![3d_2d_formula](../00_assets/3d_2d_formula.png)

To get the camera-intrinsic matrix we need the width, height and fov of the image produced by the camera.
Luckily we cn easly get these values from the sensor configuration in (agent.py)

In our case we use the following configuration: Width: 300, Height: 200, FOV: 100

The Intrinsic Matrix is calculated within the code using these values and a piece of code from pyLot (insert link)

Next up we need the extrinsic Camera-Matrix. We can set this Matrix to the Identity-Matrix, if both LIDAR and Camera are in the exact same position (e.g (0,0, 0)) in the world.

### Purpose

**Why is this usefull?**

Using this projection we can reconstruct the camera image using the lidar points. The result is a 2D-Image that has the exact same Dimensions as the Camera-Output.
We can now use this Image in Combination with the Object-Detection to find the distance for every Object.

U can imageine putting both images on top of each other.

### Implementation

All that is left now is the calculation of the Pixel-Coordinates for every LIDAR-Point in front of the vehicle and some nice Visualization.

LIDAR-Points that are not within the field of view of the camera would be projected to (u, v)-Points that dont match the Camera-Image
(for example: (-100, 23))

To visualize the reconstruced Image we create a cv2 Image where the RGB-Values a mapped to a rainbox-color scheme according to the distance at each pixel.

insert image

## Next Steps

- Combine Object-Detection with LIDAR-Reconstruction to get a depth image
- Provide List of Objects with distance in a publisher
