cmake_minimum_required(VERSION 3.8)
project(turtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtleCommand.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_executable(turtle_controller_node
  src/turtle_controller_node.cpp
  src/turtle_controller_gui.cpp
)

target_link_libraries(turtle_controller_node
  Qt5::Widgets
)

ament_target_dependencies(turtle_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

rosidl_target_interfaces(turtle_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  turtle_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
