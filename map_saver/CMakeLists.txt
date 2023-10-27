cmake_minimum_required(VERSION 3.5)
project(map_saver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

# Create an executable
add_executable(map_saver src/map_saver.cpp)

# Add dependencies to the executable
ament_target_dependencies(map_saver
  rclcpp
  geometry_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# Install the executable
install(TARGETS
  map_saver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

