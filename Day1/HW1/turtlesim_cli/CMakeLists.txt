cmake_minimum_required(VERSION 3.8)
project(turtlesim_cli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(turtlesim_controller
  src/turtlesim_controller.cpp
)
ament_target_dependencies(turtlesim_controller
  rclcpp
  std_msgs
  geometry_msgs
  turtlesim
  std_srvs
)

install(TARGETS
  turtlesim_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()
