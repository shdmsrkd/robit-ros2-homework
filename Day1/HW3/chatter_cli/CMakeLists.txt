cmake_minimum_required(VERSION 3.8)
project(chatter_cli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# talker 노드
add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker
  rclcpp
  std_msgs
)

# listener 노드
add_executable(listener src/listener_node.cpp)
ament_target_dependencies(listener
  rclcpp
  std_msgs
)

# 설치 규칙
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
