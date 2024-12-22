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

#ifndef SNAKE_GUI_H
#define SNAKE_GUI_H

//#define SNAKE_GUI_DEBUG_MODE

// Qt6
#include <QMainWindow>
#include <QtOpenGLWidgets/QtOpenGLWidgets>
#include <QtDebug>
#include <QDialog>
#include <QVector>
#include <QColor>
#include <QPushButton>
#include <QRadioButton>
#include <QAction>
#include <QPixmap>
#include <QIcon>
#include <QString>
#include <QThread>
#include <QMutex>
#include <QBrush>
#include <QtMath>
#include <QTimer>

// UI
#include "snake_gui/qcustomplot.h"
#include "snake_gui/dialog_param_control.h"
#include "snake_gui/dialog_custom_mode.h"
#include "snake_gui/dialog_typical_mode.h"
#include "snake_gui/dialog_arc_mode.h"
#include "snake_gui/dialog_spiral_mode.h"
#include "snake_gui/dialog_pid.h"
#include "snake_gui/struct_vtk_frame.h"
#include "snake_gui/thread_callback.h"
#include "snake_gui/thread_frame.h"
#include "snake_gui/snake_gui_node.h"

// VTK
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

// Eigen and Sophus.
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

// OpenCV

// PCL

// CPP
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <utility>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <snake_msgs/msg/snake_heart.hpp>
#include <snake_msgs/msg/servo_execute.hpp>
#include <snake_msgs/msg/servo_feedback.hpp>
#include <snake_params/snake_params_heart.h>
#include <snake_params/snake_params_cpg_hopf.h>

// Boost
#include <boost/thread/thread.hpp>

QT_BEGIN_NAMESPACE
namespace Ui {
    class SnakeGUI;
}
QT_END_NAMESPACE

class SnakeGUI : public QMainWindow {
Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit SnakeGUI(QWidget *parent = nullptr);

    ~SnakeGUI() override;

public:
    /*!
     * @ Calculate the interval between two Stamps.
     * @param header_1
     * @param header_2
     * @return second. header_2 - header_1
     */
    static double calTnterval(const std_msgs::msg::Header &header_1, const std_msgs::msg::Header &header_2);

signals:

    void sigReplotExecute(QCustomPlot::RefreshPriority rf_priority);

    void sigReplotFeedback(QCustomPlot::RefreshPriority rf_priority);

private slots:

    // GUI Actions.
    void onStartStop(bool checked);

    void onTorqueEnable(bool checked);

    void onParamControl();

    void onCustomMode();

    void onTypicalMode();

    void onArcMode();

    void onSpiralMode();

//    void onPID();

    // Snake frame.
    void onSetRenderWin();

    void onRenderFrame();

private: // GUI.
    void initNode(QObject *parent = nullptr);

    void initQCPJointAngle(QObject *parent = nullptr);

    void intiSnakeFrame(QObject *parent = nullptr);

    void cbExecute(const snake_msgs::msg::ServoExecute::SharedPtr msg_execute);

    void cbFeedback(const snake_msgs::msg::ServoFeedback::SharedPtr msg_feedback);

private: // GUI.
    std::shared_ptr<Ui::SnakeGUI> ui;

    std::shared_ptr<DialogParamControl> dlg_param_;
    std::shared_ptr<DialogCustomMode> dlg_custom_;
    std::shared_ptr<DialogTypicalMode> dlg_typical_;
    std::shared_ptr<DialogArcMode> dlg_arc_;
    std::shared_ptr<DialogSpiralMode> dlg_spiral_;
//    std::shared_ptr<DialogPID> dlg_pid_;
    std::shared_ptr<ThreadCallback> thread_callback_;

private: // ROS2.
    const std::string node_name = "snake_gui";
    std::shared_ptr<snake::SnakeGUINode> node_;

    rclcpp::QoS queue_size_ = 10;
    const std::string sub_execute_str_ = "servo_execute";
    const std::string sub_feedback_str_ = "servo_feedback";
    rclcpp::Subscription<snake_msgs::msg::ServoExecute>::SharedPtr sub_execute_;
    rclcpp::Subscription<snake_msgs::msg::ServoFeedback>::SharedPtr sub_feedback_;
    snake_msgs::msg::ServoExecute msg_execute_;
    snake_msgs::msg::ServoFeedback msg_feedback_;

    rclcpp::CallbackGroup::SharedPtr group_execute_;
    rclcpp::CallbackGroup::SharedPtr group_feedback_;

    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> option_execute_;
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> option_feedback_;

private: // Plot curve.
    std::shared_ptr<QThread> qcp_thread_execute_;
    std::shared_ptr<QThread> qcp_thread_feedback_;
    QMutex qcp_mutex_execute_;
    QMutex qcp_mutex_feedback_;

    std_msgs::msg::Header qcp_time_start_;
    const int qcp_data_num_ = 300; // Number of stored history.
    QVector<double> qcp_time_execute_;
    QVector<double> qcp_time_feedback_;
    QVector<QVector<double>> qcp_data_execute_;
    QVector<QVector<double>> qcp_data_feedback_;
    QVector<QColor> qcp_curve_color_;
    QVector<QCPScatterStyle::ScatterShape> qcp_curve_style_;

private: // Display frame.
    // tf2.
    std::shared_ptr<tf2_ros::TransformListener> vtk_tf2_listener_;
    std::shared_ptr<tf2_ros::Buffer> vtk_tf2_buffer_;

    std::shared_ptr<snake::VTKFrame> vtk_frame_;

    std::shared_ptr<ThreadFrame> vtk_thread_;
    std::shared_ptr<QThread> vtk_qthread_;
};

#endif //SNAKE_GUI_H
