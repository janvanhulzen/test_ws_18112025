cmake_minimum_required(VERSION 3.10)
project(my_robot_world)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)   # PlanningSceneInterface
find_package(geometric_shapes REQUIRED)                # createMeshFromResource
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(add_world_objects src/add_world_objects.cpp)
ament_target_dependencies(add_world_objects
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  shape_msgs
  tf2_geometry_msgs
  geometry_msgs
)

# If you have headers, uncomment:
# target_include_directories(add_world_objects PUBLIC include)

install(TARGETS add_world_objects
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install runtime assets so you can use package:// URIs
install(DIRECTORY
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
