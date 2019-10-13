cmake_minimum_required(VERSION 3.5)
project(image_proc_chain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SwitchProcessorType.srv"
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME}_core
  src/${PROJECT_NAME}/chain_piece.cpp
  src/${PROJECT_NAME}/image_processors.cpp
  src/${PROJECT_NAME}/switchable_image_processor.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  cv_bridge
  image_transport
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
)


add_executable(${PROJECT_NAME}_chain_piece_node
  src/${PROJECT_NAME}/chain_piece_node.cpp
)
target_link_libraries(${PROJECT_NAME}_chain_piece_node
  ${PROJECT_NAME}_core
)

# install libraries
install(TARGETS
    ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executables
install(TARGETS
    ${PROJECT_NAME}_chain_piece_node
    DESTINATION lib/${PROJECT_NAME}
)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(${PROJECT_NAME}_core
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
