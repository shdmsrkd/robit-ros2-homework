cmake_minimum_required(VERSION 3.5)
project(ros2_topic_qt_ui VERSION 0.1 LANGUAGES CXX)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# UI 파일 검색 경로 설정
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

# UI 파일 설정
set(PROJECT_UI_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/mainwindowdesign.ui
)

# 소스 파일 설정
set(PROJECT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp
)

# 헤더 파일 설정
set(PROJECT_HEADERS
    include/${PROJECT_NAME}/main_window.hpp
    include/${PROJECT_NAME}/qnode.hpp
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${PROJECT_HEADERS}
    ${PROJECT_UI_FILES}
)

# Include 디렉토리 설정
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# 의존성 설정
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
)

# Qt 라이브러리 링크
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Core
)

# 설치 설정
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    include/
    ui/
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
