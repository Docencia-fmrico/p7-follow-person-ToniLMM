cmake_minimum_required(VERSION 3.8)
project(followperson)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yolo_msgs REQUIRED)

set(dependencies
  rclcpp
  kobuki_ros_interfaces
  geometry_msgs
  rclcpp_cascade_lifecycle
  tf2_ros
  vision_msgs
  nav_msgs
  tf2_geometry_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  yolo_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/followperson/PerceptionNode.cpp
  src/followperson/ControlNode.cpp
  src/followperson/MainNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(followperson_node src/followperson_main.cpp)
ament_target_dependencies(followperson_node ${dependencies})
target_link_libraries(followperson_node ${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  followperson_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
