# ClusteredPointsArray.msg

# Standard ROS message header
std_msgs/Header header

# Array of 3D points
float64[] clusterPointsArray

# Index array for points in clusters
int32[] indexArray

# Array of motion properties for each cluster
Motion2D[] motionArray

# Object class for each cluster
int32[] object_class