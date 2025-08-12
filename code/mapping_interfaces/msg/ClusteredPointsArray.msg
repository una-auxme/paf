# ClusteredPointsArray.msg

# Standard ROS message header
std_msgs/Header header

# Array of 3D points
float64[] cluster_points_array

# Index array for points in clusters
int32[] index_array

# Array of motion properties for each cluster
Motion2D[] motion_array

# Object class for each cluster
int32[] object_class
