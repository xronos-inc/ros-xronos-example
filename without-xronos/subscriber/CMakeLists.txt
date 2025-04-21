cmake_minimum_required(VERSION 3.28)
project(ros_xronos_example_subscriber VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard is cached for visibility in external tools." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DEFAULT_BUILD_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
set    (CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LF_MAIN_TARGET ros_xronos_example_subscriber)

add_executable(${LF_MAIN_TARGET}
    src/subscriber.cpp
)

ament_target_dependencies(${LF_MAIN_TARGET} rclcpp std_msgs)
target_include_directories(${LF_MAIN_TARGET} PUBLIC
    "include"
)

if(MSVC)
  target_compile_options(${LF_MAIN_TARGET} PRIVATE /W4)
else()
  target_compile_options(${LF_MAIN_TARGET} PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS
  ${LF_MAIN_TARGET}
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
