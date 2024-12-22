cmake_minimum_required(VERSION 3.5)
project(snake_sensor)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snake_params REQUIRED)

find_package(Boost REQUIRED)
find_package(FMT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
MESSAGE("OpenCV VERSION:")
MESSAGE(${OpenCV_VERSION})
MESSAGE("Eigen3 VERSION:")
MESSAGE(${Eigen3_VERSION})

include_directories(
        include

        ${PROJECT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}

        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Add executable.
set(HEADER_FILES
        include/snake_sensor/snake_sensor_stereo_segment_node.h
        include/snake_sensor/snake_sensor_stereo_segment.h
        )

set(SOURCE_FILES
        src/snake_sensor_stereo_segment_node.cpp
        src/snake_sensor_stereo_segment.cpp
        )

add_executable(${PROJECT_NAME}_usb_cam
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

ament_target_dependencies(${PROJECT_NAME}_usb_cam
        rclcpp
        cv_bridge
        image_transport
        std_msgs
        sensor_msgs
        snake_params
        )

target_link_libraries(${PROJECT_NAME}_usb_cam
        fmt::fmt
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${EIGEN3_LIBS}
        )

install(TARGETS ${PROJECT_NAME}_usb_cam DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
