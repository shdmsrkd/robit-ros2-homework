cmake_minimum_required(VERSION 3.8)
project(turtlesim_draw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

add_executable(draw_node src/draw_node.cpp)
ament_target_dependencies(draw_node
  rclcpp
  geometry_msgs
  turtlesim
)

install(TARGETS
  draw_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/turtlesim_draw
  DESTINATION include/turtlesim_draw
)

ament_package()
