cmake_minimum_required(VERSION 3.8)
project(my_robot_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(limit_switch_watcher_node src/limit_switch_watcher_node.cpp)
ament_target_dependencies(limit_switch_watcher_node
  rclcpp std_msgs std_srvs controller_manager_msgs)

# Link Phidgets C library
target_link_libraries(limit_switch_watcher_node phidget22)

install(TARGETS limit_switch_watcher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
