cmake_minimum_required(VERSION 3.14)
project(my_robot_hardware LANGUAGES CXX)

# ---- C++ standard & warnings ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# cmake_minimum_required(VERSION 3.8)
# project(my_robot_hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# ---- ROS 2 deps  for arm ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# This brings in include dirs + -lphidget22 automatically

# ---- Library (the hardware_interface plugin) ----
add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/arm_hardware_interface.cpp
)

# Public headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

#   rclcpp rclcpp_lifecycle hardware_interface pluginlib for arm


ament_target_dependencies(${PROJECT_NAME}
PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)




# ---- Plugin export (XML must exist at package top-level) ----
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)

# ---- Install ----
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- Phidgets fallback: find library and headers manually ---
find_library(PHIDGET22_LIB phidget22
  HINTS /usr/lib /usr/lib64 /usr/local/lib /opt/homebrew/lib
  REQUIRED
)
find_path(PHIDGET22_INCLUDE phidget22.h
  HINTS /usr/include/libphidget22 /usr/local/include/libphidget22 /opt/homebrew/include/libphidget22
  REQUIRED
)

target_include_directories(${PROJECT_NAME} PUBLIC ${PHIDGET22_INCLUDE})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PHIDGET22_LIB})

# ---- ament exports ----

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
