cmake_minimum_required(VERSION 3.5)
project(drone_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(as2_motion_reference_handlers REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

set(dependencies
  rclcpp 
  pcl_conversions
  pcl_ros  
  geometry_msgs 
  sensor_msgs
  message_filters
  as2_core
  as2_msgs
  as2_motion_reference_handlers
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(drone_nav_node src/drone_nav.cpp)

ament_target_dependencies(drone_nav_node
  ${dependencies}
)

target_link_libraries(drone_nav_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  drone_nav_node
  DESTINATION share/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

