cmake_minimum_required(VERSION 3.20)
project(snake_joy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snake_msgs REQUIRED)
find_package(snake_params REQUIRED)

include_directories(
        include
)

# Add executable.
set(HEADER_FILES
        include/snake_joy/snake_joy_node.h
        include/snake_joy/snake_joy.h
        )

set(SOURCE_FILES
        src/snake_joy_node.cpp
        src/snake_joy.cpp
        )

add_executable(${PROJECT_NAME}
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        std_msgs
        snake_msgs
        snake_params
        )

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
