cmake_minimum_required(VERSION 3.10)
project(bringup_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(sonic_filter_node src/sonic_filter_node.cpp)
ament_target_dependencies(sonic_filter_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

install(TARGETS sonic_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES Sonic-Movie-PNG.png
  DESTINATION share/${PROJECT_NAME})

ament_package()
