cmake_minimum_required(VERSION 3.10)
project(armor_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorDetection.msg"
  "msg/ArmorDetections.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include)

add_executable(armor_vision_node
  src/armor_vision_node.cpp
  src/detector.cpp
  src/tracker.cpp
  src/param_utils.cpp
)

target_include_directories(armor_vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(armor_vision_node
  rclcpp sensor_msgs image_transport cv_bridge geometry_msgs rcl_interfaces)

target_link_libraries(armor_vision_node ${OpenCV_LIBS})

rosidl_target_interfaces(armor_vision_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS armor_vision_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
