cmake_minimum_required(VERSION 3.20)
project(snake_frame)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snake_msgs REQUIRED)
find_package(snake_params REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# System dependencies.
find_package(Boost REQUIRED)
find_package(FMT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
MESSAGE("Eigen3 VERSION:")
MESSAGE(${Eigen3_VERSION})

include_directories(
        include
        ${PROJECT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Add library.
set(HEADER_FILES
        include/snake_frame/snake_frame.h
        )

set(SOURCE_FILES
        src/snake_frame.cpp
        )

add_library(${PROJECT_NAME} SHARED
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        fmt::fmt
        ${Sophus_LIBRARIES}
        ${EIGEN3_LIBS}
        )

ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        )

ament_export_targets(
        ${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
        rclcpp
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Add executable.
set(HEADER_FILES
        include/snake_frame/snake_frame_node.h
        include/snake_frame/snake_frame_sp.h
        )

set(SOURCE_FILES
        src/snake_frame_node.cpp
        src/snake_frame_sp.cpp
        )

add_executable(${PROJECT_NAME}_sp
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

target_link_libraries(${PROJECT_NAME}_sp PUBLIC
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        fmt::fmt
        ${Sophus_LIBRARIES}
        ${EIGEN3_LIBS}
        )

ament_target_dependencies(${PROJECT_NAME}_sp PUBLIC
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
        snake_msgs
        snake_params
        tf2
        tf2_eigen
        tf2_msgs
        tf2_ros
        )

install(TARGETS ${PROJECT_NAME}_sp DESTINATION lib/${PROJECT_NAME})

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
