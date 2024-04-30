cmake_minimum_required(VERSION 3.5)
project(image_proc_chain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_proc_chain_msgs REQUIRED)


include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/chain_piece.cpp
  src/${PROJECT_NAME}/image_processors.cpp
  src/${PROJECT_NAME}/switchable_image_processor.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  image_transport
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  image_proc_chain_msgs
)
rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  "image_proc_chain::ChainPiece")

add_executable(${PROJECT_NAME}_component_container
  src/${PROJECT_NAME}/component_container.cpp
  src/${PROJECT_NAME}/component_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}_component_container
  rclcpp
  rclcpp_components
  image_proc_chain_msgs
)


# install libraries
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_component_container
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install executables
install(TARGETS
    ${PROJECT_NAME}_component_container
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install other files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY include/
  DESTINATION include
)

# get_default_rmw_implementation(rmw_implementation)
# find_package("${rmw_implementation}" REQUIRED)
# get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
# foreach(typesupport_impl ${typesupport_impls})
#   rosidl_target_interfaces(${PROJECT_NAME}_core
#     ${PROJECT_NAME} ${typesupport_impl}
#   )
# endforeach()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
