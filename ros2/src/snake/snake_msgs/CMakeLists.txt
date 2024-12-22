cmake_minimum_required(VERSION 3.20)
project(snake_msgs)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
        msg/ServoExecute.msg
        msg/ServoFeedback.msg
        msg/SnakeHeart.msg
        )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        stereo_msgs
        )

#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_dependencies(rosidl_default_generators)
#ament_export_dependencies(rosidl_default_runtime)

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
