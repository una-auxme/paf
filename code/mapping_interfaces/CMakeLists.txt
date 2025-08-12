cmake_minimum_required(VERSION 3.8)
project(mapping_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include any interface dependencies here:
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# Add all your interface files here:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Entity.msg"
  "msg/Flags.msg"
  "msg/Map.msg"
  "msg/Motion2D.msg"
  "msg/Shape2D.msg"
  "msg/TrackingInfo.msg"
  "msg/Transform2D.msg"
  "msg/TypeCar.msg"
  "msg/TypeLanemarking.msg"
  "msg/TypeTrafficLight.msg"
  "msg/TypeStopMark.msg"
  "msg/ClusteredPointsArray.msg"

  "srv/UpdateStopMarks.srv"

  DEPENDENCIES # Add packages that above messages depend on
  std_msgs
  unique_identifier_msgs
  geometry_msgs
  sensor_msgs
)

ament_package()
