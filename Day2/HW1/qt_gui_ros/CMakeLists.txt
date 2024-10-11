cmake_minimum_required(VERSION 3.5)
project(qt_gui_ros VERSION 0.1 LANGUAGES CXX)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake의 자동 UIC, MOC, RCC 기능 활성화
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# UI 파일 검색 경로 설정
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

# 파일 경로 설정
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
set(UI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)

# 프로젝트 파일 설정
set(SOURCES
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/main_window.cpp
    ${SOURCE_DIR}/qnode.cpp
)

set(HEADERS
    ${INCLUDE_DIR}/main_window.hpp
    ${INCLUDE_DIR}/qnode.hpp
)

set(UI_FILES
    ${UI_DIR}/mainwindow.ui
)

set(QRC_FILES
    ${RESOURCES_DIR}/images.qrc
)

# include 디렉토리 설정
include_directories(
    ${INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${UI_FILES}
    ${QRC_FILES}
)

# 의존성 및 라이브러리 설정
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    rcl_interfaces
    Boost
    turtlesim
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Network
)

# UI 파일 복사
configure_file(${UI_DIR}/mainwindow.ui ${CMAKE_BINARY_DIR}/mainwindow.ui COPYONLY)

# 설치 경로 설정
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
