/*
========================================================================================================================

    ┌────────────────────────────────────────────────────────┐                                       .::!!!!!!!:.
    │  mmmm                #              mmmm               │      .!!!!!:.                        .:!!!!!!!!!!!!
    │ #"   " m mm    mmm   #   m   mmm   #"   " m   m   mmm  │      ~~~~!!!!!!.                 .:!!!!!!!!!UWWW$$$
    │ "#mmm  #"  #  "   #  # m"   #"  #  "#mmm  "m m"  #   " │          :$$NWX!!:           .:!!!!!!XUWW$$$$$$$$$P
    │     "# #   #  m"""#  #"#    #""""      "#  #m#    """m │          $$$$$##WX!:      .<!!!!UW$$$$"  $$$$$$$$#
    │ "mmm#" #   #  "mm"#  #  "m  "#mm"  "mmm#"  "#    "mmm" │          $$$$$  $$$UX   :!!UW$$$$$$$$$   4$$$$$*
    │                                            m"          │          ^$$$B  $$$$\     $$$$$$$$$$$$   d$$R"
    │                                           ""           │            "*$bd$$$$      '*$$$$$$$$$$$o+#"
    └────────────────────────────────────────────────────────┘                 """"          """""""

* SnakeSys: An Open-Source System for Snake Robots Research
* Copyright (C) 2020-2025 Xupeng Liu, Yong Zang and Zhiying Gao
*
* This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.

========================================================================================================================
*/

#ifndef THREAD_FRAME_H
#define THREAD_FRAME_H

#include "snake_gui/struct_vtk_frame.h"
#include "snake_gui/snake_gui_node.h"

#include <QObject>
#include <QTimer>
#include <QThread>

#include <iostream>
#include <vector>
#include <tuple>
#include <thread>
#include <utility>

#include <snake_params/snake_params_heart.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <vtkAxesActor.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkTextProperty.h>
#include <vtkCaptionActor2D.h>
#include <vtkCubeAxesActor.h>
#include <vtkOpenGLRenderer.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOutputWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

class ThreadFrame : public QObject {
Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ThreadFrame(std::shared_ptr<snake::SnakeGUINode> node,
                         std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
                         std::shared_ptr<snake::VTKFrame> vtk_frame,
                         QObject *parent = nullptr);

    ~ThreadFrame() override;

signals:

    void sigSetRenderWin();

    void sigRenderFrame();

public slots:

    void onCreateTimer();

    void onUpdateFrame();

private:
    void initFrame();

private:
    // 0:snake_world-->snake_base
    // 1:snake_base-->snake_camera_1
    // 2:snake_base-->snake_camera_2
    // 3:snake_base-->snake_imu
    // 4:snake_base-->link_1
    // 5:link_1-->link_2
    // 6:link_2-->...
    // n+3:link_n-1-->link_n.
    std::shared_ptr<std::vector<geometry_msgs::msg::TransformStamped>> tfs_msg_;
    std::shared_ptr<std::vector<std::tuple<std::string, std::string>>> tfs_frame_id_;

    std::shared_ptr<snake::SnakeGUINode> node_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

    std::shared_ptr<snake::VTKFrame> vtk_frame_;
    bool flag_re_win_;

    std::shared_ptr<QTimer> timer_ptr_;
};

#endif //THREAD_FRAME_H
