cmake_minimum_required(VERSION 3.20)
project(snake_cpg)

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
find_package(snake_msgs REQUIRED)
find_package(snake_params REQUIRED)
find_package(snake_servo REQUIRED)

find_package(GSL REQUIRED)
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
        ${GSL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Add executable.
set(HEADER_FILES_HOPF
        include/snake_cpg/snake_cpg_hopf_node.h
        include/snake_cpg/snake_cpg_hopf.h
        )

set(SOURCE_FILES_HOPF
        src/snake_cpg_hopf_node.cpp
        src/snake_cpg_hopf.cpp
        )

add_executable(${PROJECT_NAME}_hopf
        ${HEADER_FILES_HOPF}
        ${SOURCE_FILES_HOPF}
        )

ament_target_dependencies(${PROJECT_NAME}_hopf
        rclcpp
        std_msgs
        snake_msgs
        snake_params
        snake_servo
        )

target_link_libraries(${PROJECT_NAME}_hopf
        ${GSL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        fmt::fmt
        ${Sophus_LIBRARIES}
        ${EIGEN3_LIBS}
        )

install(TARGETS ${PROJECT_NAME}_hopf DESTINATION lib/${PROJECT_NAME})

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
