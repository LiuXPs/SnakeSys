cmake_minimum_required(VERSION 3.20)
project(snake_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/ui) # If the .ui files and .h files are not in the same directory.

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
find_package(sensor_msgs REQUIRED)
find_package(snake_msgs REQUIRED)
find_package(snake_params REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

set(CMAKE_PREFIX_PATH /opt/Qt/6.7.0/gcc_64/lib/cmake)
#set(Qt6_DIR /opt/Qt/6.5.0/gcc_64/lib/cmake/Qt6)
set(VTK_DIR /usr/local/include/vtk-9.3)
#set(catkin_DIR /opt/ros/noetic/share/catkin/cmake)

find_package(Boost REQUIRED)
find_package(FMT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
MESSAGE("OpenCV VERSION:")
MESSAGE(${OpenCV_VERSION})
MESSAGE("Eigen3 VERSION:")
MESSAGE(${Eigen3_VERSION})

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS
        Core
        Gui
        PrintSupport
        Widgets
        OpenGLWidgets
        REQUIRED
)

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
)

# Add ui files.
set(UI_FILES
        ui/dialog_param_control.ui
        ui/dialog_custom_mode.ui
        ui/dialog_typical_mode.ui
        ui/dialog_arc_mode.ui
        ui/dialog_spiral_mode.ui
        ui/dialog_pid.ui
        ui/double_spinbox_and_slider.ui
        ui/spinbox_and_slider.ui
        ui/snake_gui.ui
)

# Add header files.
set(HEADER_FILES
        include/snake_gui/qcustomplot.h
        include/snake_gui/dialog_param_control.h
        include/snake_gui/dialog_custom_mode.h
        include/snake_gui/dialog_typical_mode.h
        include/snake_gui/dialog_arc_mode.h
        include/snake_gui/dialog_spiral_mode.h
        include/snake_gui/dialog_pid.h
        include/snake_gui/double_spinbox_and_slider.h
        include/snake_gui/spinbox_and_slider.h
        include/snake_gui/struct_vtk_frame.h
        include/snake_gui/thread_callback.h
        include/snake_gui/thread_frame.h
        include/snake_gui/thread_pid.h
        include/snake_gui/snake_gui_node.h
        include/snake_gui/snake_gui.h
)

# Add source files.
set(SOURCE_FILES
        src/qcustomplot.cpp
        src/dialog_param_control.cpp
        src/dialog_custom_mode.cpp
        src/dialog_typical_mode.cpp
        src/dialog_arc_mode.cpp
        src/dialog_spiral_mode.cpp
        src/dialog_pid.cpp
        src/doublespinbox_and_slider.cpp
        src/spinbox_and_slider.cpp
        src/thread_callback.cpp
        src/thread_frame.cpp
        src/thread_pid.cpp
        src/snake_gui_node.cpp
        src/snake_gui.cpp
        src/main.cpp
)

# Add resource files.
set(RESOURCE_FILES
        resource/image.qrc
)

if (${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(${PROJECT_NAME}
            MANUAL_FINALIZATION
            ${HEADER_FILES}
            ${SOURCE_FILES}
            ${UI_FILES}
            ${RESOURCE_FILES}
    )

    # Define target properties for Android with Qt 6 as:
    #    set_property(TARGET snake_gui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
    #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
    # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else ()
    add_executable(${PROJECT_NAME}
            ${HEADER_FILES}
            ${SOURCE_FILES}
            ${UI_FILES}
            ${RESOURCE_FILES}
    )
endif ()

if (QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(${PROJECT_NAME})
endif ()

target_link_libraries(${PROJECT_NAME} PUBLIC
        # 3rdparty
        fmt::fmt
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${VTK_LIBRARIES}
        ${EIGEN3_LIBS}
        # Qt
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::PrintSupport
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::OpenGLWidgets
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        serial
        std_msgs
        sensor_msgs
        snake_msgs
        snake_params
        tf2
        tf2_eigen
        tf2_ros
        tf2_msgs
)

## Generate MOC files.
#qt6_wrap_cpp(MOC_FILES
#        ${HEADER_FILES}
#        )
#
## Generate UI headers.
#qt6_wrap_ui(UI_HEADERS
#        ${UI_FILES}
#        )
#
# Create executable.
#add_executable(${PROJECT_NAME}
#        ${SOURCES}
#        ${MOC_FILES}
#        ${UI_HEADERS}
#        ${RESOURCE_FILES}
#        )

#add_executable(${PROJECT_NAME}
#        ${HEADER_FILES}
#        ${SOURCE_FILES}
#        ${UI_FILES}
#        ${RESOURCE_FILES}
#        )

# Install ui files.
install(FILES ${UI_FILES} DESTINATION share/${PROJECT_NAME}/ui)

# Install include files.
install(DIRECTORY include/ DESTINATION include)

# Install executable.
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

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
