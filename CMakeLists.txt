cmake_minimum_required(VERSION 3.5)
project(image_proc_chain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/io.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  image_transport
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
