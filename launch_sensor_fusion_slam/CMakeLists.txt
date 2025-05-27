cmake_minimum_required(VERSION 3.5)
project(launch_sensor_fusion_slam)

# Enable C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create localisation_test node
add_executable(localisation_test src/localisation_test.cpp)
ament_target_dependencies(localisation_test rclcpp sensor_msgs nav_msgs std_msgs)

# Create lidar_test node
add_executable(lidar_test src/lidar_test.cpp)
ament_target_dependencies(lidar_test rclcpp sensor_msgs std_msgs)

# Install executables
install(TARGETS
  localisation_test
  lidar_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
