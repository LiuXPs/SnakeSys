cmake_minimum_required(VERSION 3.20)
project(snake_servo)

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
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snake_msgs REQUIRED)
find_package(snake_params REQUIRED)

include_directories(
        include
)

# Add feetech_servo library.
set(HEADER_FILES_FT
        include/snake_servo/snake_feetech_data.h
        include/snake_servo/snake_feetech_proto.h
        include/snake_servo/snake_feetech_servo.h
        )

set(SOURCE_FILES_FT
        src/snake_feetech_proto.cpp
        src/snake_feetech_servo.cpp
        )

add_library(feetech_servo SHARED
        ${HEADER_FILES_FT}
        ${SOURCE_FILES_FT}
        )

target_include_directories(feetech_servo PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(feetech_servo PUBLIC
        rclcpp
        serial
        )

ament_export_targets(
        feetech_servo HAS_LIBRARY_TARGET
)

ament_export_dependencies(
        rclcpp
        serial
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS feetech_servo
        EXPORT feetech_servo
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )

install(TARGETS feetech_servo DESTINATION lib/${PROJECT_NAME})

# Add feetech_servo example.
set(SOURCE_FILES_FT_EXAMPLE
        examples/snake_feetech_servo_example.cpp
        )

add_executable(feetech_servo_example
        ${SOURCE_FILES_FT_EXAMPLE}
        )

target_link_libraries(feetech_servo_example PUBLIC
        feetech_servo
        )

ament_target_dependencies(feetech_servo_example PUBLIC
        rclcpp
        serial
        )

install(TARGETS feetech_servo_example DESTINATION lib/${PROJECT_NAME})

# Add snake servo executable.
set(HEADER_FILES
        include/snake_servo/snake_servo_node.h
        include/snake_servo/snake_servo_ft.h
        )

set(SOURCE_FILES
        src/snake_servo_node.cpp
        src/snake_servo_ft.cpp
        )

add_executable(${PROJECT_NAME}_ft
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

target_link_libraries(${PROJECT_NAME}_ft
        feetech_servo
        )

ament_target_dependencies(${PROJECT_NAME}_ft
        rclcpp
        serial
        std_msgs
        snake_msgs
        snake_params
        )

install(TARGETS ${PROJECT_NAME}_ft DESTINATION lib/${PROJECT_NAME})

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
