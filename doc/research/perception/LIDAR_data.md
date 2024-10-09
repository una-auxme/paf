# LIDAR-Data

This File discusses where the LIDAR-Data comes from, how its processed and how we could possibly use it.

## Origin

LIDAR-Data comes in Pointclouds from a specific LIDAR-Topic.

`rospy.Subscriber(rospy.get_param('~source_topic', "/carla/hero/LIDAR"),
                         PointCloud2, self.callback)`

Read more about the LIDAR-Sensor [here](https://github.com/una-auxme/paf/blob/main/doc/perception/lidar_distance_utility.md)

## Processing

The goal is to identify Objects and their distance. Therefor we need to calculate distances from the pointcloud data.
To do this the lidar-distance node first converts pointcloud data to an array, which contains cartesian coordinates.

`paf-agent-1            |  (76.12445   , -1.6572031e+01, 13.737187  , 0.7287409 )`

`paf-agent-1            |  (71.9434    , -1.8718828e+01, 13.107929  , 0.7393809 )`

`paf-agent-1            |  (-0.3482422 , -1.6367188e-02, -0.20128906, 0.99839103)`

`paf-agent-1            |  (-0.3486328 , -1.4062500e-02, -0.20152344, 0.99838954)`

`paf-agent-1            |  (-0.35070312, -2.3828126e-03, -0.2025    , 0.99838144)`

The first three values of each row correspon to x, y, z.

x - the X Cartesian coordinate of a point (float32)

y - the Y Cartesian coordinate of a point (float32)

z - the Z Cartesian coordinate of a point (float32)

It wasn´t specified anywhere, what the 4th values represents. My best guess is some sort of intensity.

## Distance Calculation

The distance to a point is calculated by the euclidian distance to (0,0,0) for every point in the point cloud.

`distances = np.array(
            [np.linalg.norm(c - [0, 0, 0]) for c in coordinates_xyz])`

They then publish the minimum and maximum distance.

## Open questions

1. Currently we have no specified unit.
    **Is a relative distance enough?**

2. How do we translate cartesian coordinates to pixel in the Image and vice-versa?

   We should be able to set the bounding box of the LIDAR-PointCloud2 to the camera-calibration.
