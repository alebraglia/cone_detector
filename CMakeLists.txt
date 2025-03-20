cmake_minimum_required(VERSION 3.5)
project(cone_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(cone_detector_node src/cone_detector_node.cpp)
ament_target_dependencies(cone_detector_node rclcpp sensor_msgs std_msgs pcl_conversions)
target_link_libraries(cone_detector_node ${PCL_LIBRARIES})

install(TARGETS
  cone_detector_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
