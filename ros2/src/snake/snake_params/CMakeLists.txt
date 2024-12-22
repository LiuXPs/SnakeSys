cmake_minimum_required(VERSION 3.20)
project(snake_params)

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

include_directories(
        include
)

# Add library.
set(HEADER_FILES
        include/snake_params/snake_params_heart.h
        include/snake_params/snake_params_cpg_hopf.h
        include/snake_params/snake_params_slam.h
        include/snake_params/snake_params_frame.h
        include/snake_params/snake_params_servo.h
        include/snake_params/snake_params_sensor.h
        )

set(SOURCE_FILES
        src/snake_params_heart.cpp
        src/snake_params_cpg_hopf.cpp
        src/snake_params_slam.cpp
        src/snake_params_frame.cpp
        src/snake_params_servo.cpp
        src/snake_params_sensor.cpp
        )

add_library(${PROJECT_NAME} SHARED
        ${HEADER_FILES}
        ${SOURCE_FILES}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
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
