cmake_minimum_required(VERSION 3.0.2)
project(sim)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  rospy
  roscpp
  tf
)

catkin_python_setup()


add_message_files(
   FILES
   VehicleCtrl.msg
   VehicleInfo.msg
   Entity.msg
   Entities.msg
   Vector2D.msg
   VisPath.msg
   MultiPath.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)




catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy tf
)

