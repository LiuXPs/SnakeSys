cmake_minimum_required(VERSION 3.20)
project(snake_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SnakeSys/ORB_SLAM3.
set(ORB_SLAM3_DIR ${PROJECT_SOURCE_DIR}/../../../../ORB_SLAM3)
set(CMAKE_PREFIX_PATH /opt/Qt/6.7.0/gcc_64/lib/cmake)
set(VTK_DIR /usr/local/include/vtk-9.3)
set(PCL_DIR /usr/local/include/pcl-1.14)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(snake_msgs REQUIRED)
find_package(snake_params REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

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

MESSAGE("Boost VERSION:")
MESSAGE(${Boost_VERSION})

find_package(QT NAMES Qt6 REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

# Add include directories.
include_directories(
        include

        ${PROJECT_SOURCE_DIR} # Used to locate the .h files converted from the .ui files.
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}

        # third-party libraries.
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${VTK_DIR}
        ${PCL_DIR}
        #        ${PCL_INCLUDE_DIRS}

        # ORB_SLAM3
        ${ORB_SLAM3_DIR}
        ${ORB_SLAM3_DIR}/include
        ${ORB_SLAM3_DIR}/include/CameraModels
        ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

# Add executable.
set(LIBS
        # ORB_SLAM3
        ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
        ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so

        # 3rdparty
        fmt::fmt
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${EIGEN3_LIBS}
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
)

set(AMENT_LIBS
        rclcpp
        serial
        cv_bridge
        image_transport
        geometry_msgs
        std_msgs
        sensor_msgs
        stereo_msgs
        snake_msgs
        snake_params
        tf2
        tf2_eigen
        tf2_ros
        tf2_msgs
)

set(HEADER_FILES
        include/snake_slam/snake_slam_node.h
)

#set(SOURCE_FILES
#        src/snake_slam_node.cpp
#        )

# mono node
set(SOURCE_FILES_MONO
        src/snake_slam_node.cpp
        src/slam_mono.cpp
)
add_executable(${PROJECT_NAME}_mono
        ${HEADER_FILES}
        ${SOURCE_FILES_MONO}
)
target_link_libraries(${PROJECT_NAME}_mono
        ${LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_mono
        ${AMENT_LIBS}
)

# stereo node
set(SOURCE_FILES_STEREO
        src/snake_slam_node.cpp
        src/slam_stereo.cpp
)
add_executable(${PROJECT_NAME}_stereo
        ${HEADER_FILES}
        ${SOURCE_FILES_STEREO}
)
target_link_libraries(${PROJECT_NAME}_stereo
        ${LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_stereo
        ${AMENT_LIBS}
)

# RGB-D node
set(SOURCE_FILES_RGBD
        src/snake_slam_node.cpp
        src/slam_rgbd.cpp
)
add_executable(${PROJECT_NAME}_rgbd
        ${HEADER_FILES}
        ${SOURCE_FILES_RGBD}
)
target_link_libraries(${PROJECT_NAME}_rgbd
        ${LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_rgbd
        ${AMENT_LIBS}
)

# mono-imu node
set(SOURCE_FILES_MONO_IMU
        src/snake_slam_node.cpp
        src/slam_mono_imu.cpp
)
add_executable(${PROJECT_NAME}_mono_imu
        ${HEADER_FILES}
        ${SOURCE_FILES_MONO_IMU}
)
target_link_libraries(${PROJECT_NAME}_mono_imu
        ${LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_mono_imu
        ${AMENT_LIBS}
)

# stereo-imu node
set(SOURCE_FILES_STEREO_IMU
        src/snake_slam_node.cpp
        src/slam_stereo_imu.cpp
)
add_executable(${PROJECT_NAME}_stereo_imu
        ${HEADER_FILES}
        ${SOURCE_FILES_STEREO_IMU}
)
target_link_libraries(${PROJECT_NAME}_stereo_imu
        ${LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_stereo_imu
        ${AMENT_LIBS}
)

# RGB-D-imu node
set(SOURCE_FILES_RGBD_IMU
        src/snake_slam_node.cpp
        src/slam_rgbd_imu.cpp
)
add_executable(${PROJECT_NAME}_rgbd_imu
        ${HEADER_FILES}
        ${SOURCE_FILES_RGBD_IMU}
)
target_link_libraries(${PROJECT_NAME}_rgbd_imu
        ${LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_rgbd_imu
        ${AMENT_LIBS}
)

install(TARGETS ${PROJECT_NAME}_mono DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_stereo DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_rgbd DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_mono_imu DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_stereo_imu DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_rgbd_imu DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY vocabulary DESTINATION share/${PROJECT_NAME})

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
