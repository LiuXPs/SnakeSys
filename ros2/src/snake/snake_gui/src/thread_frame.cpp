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

#include "snake_gui/thread_frame.h"

ThreadFrame::ThreadFrame(std::shared_ptr<snake::SnakeGUINode> node,
                         std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
                         std::shared_ptr<snake::VTKFrame> vtk_frame,
                         QObject *parent) {
    tfs_msg_ = std::make_shared<std::vector<geometry_msgs::msg::TransformStamped>>();
    tfs_frame_id_ = std::make_shared<std::vector<std::tuple<std::string, std::string>>>();
    tfs_msg_->resize(0);
    tfs_frame_id_->resize(0);

    node_ = std::move(node);
    tf2_buffer_ = std::move(tf2_buffer);
    vtk_frame_ = std::move(vtk_frame);

    flag_re_win_ = false;

    timer_ptr_ = nullptr;

    onCreateTimer();
    initFrame();

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << " constructor frame thread id:" << QThread::currentThreadId();
#endif
}

ThreadFrame::~ThreadFrame() {
    timer_ptr_->stop();
    timer_ptr_->deleteLater();
}

void ThreadFrame::onCreateTimer() {
    timer_ptr_ = std::make_shared<QTimer>();
    int time_interval = 10;
    connect(timer_ptr_.get(), &QTimer::timeout, this, &ThreadFrame::onUpdateFrame, Qt::DirectConnection);
    timer_ptr_->start(time_interval);
}

void ThreadFrame::onUpdateFrame() {
    if (static_cast<int>(tfs_frame_id_->size()) != (node_->params_heart_->servo_idn_val + 4)) {
        initFrame();
    }

    for (int i = 0; i < static_cast<int>(tfs_frame_id_->size()); i++) {
        try {
            tfs_msg_->at(i) = tf2_buffer_->lookupTransform(std::get<0>(tfs_frame_id_->at(i)),
                                                           std::get<1>(tfs_frame_id_->at(i)),
                                                           tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
        }
    }

    std::unique_lock<QMutex> ul(vtk_frame_->vtk_mutex);
    for (int i = 0; i < static_cast<int>(tfs_msg_->size()); i++) {
        Eigen::Quaterniond q_pose;
        q_pose.w() = tfs_msg_->at(i).transform.rotation.w;
        q_pose.x() = tfs_msg_->at(i).transform.rotation.x;
        q_pose.y() = tfs_msg_->at(i).transform.rotation.y;
        q_pose.z() = tfs_msg_->at(i).transform.rotation.z;
        q_pose = q_pose.normalized(); // This step is very important.

        Eigen::Vector3d t_pose(tfs_msg_->at(i).transform.translation.x,
                               tfs_msg_->at(i).transform.translation.y,
                               tfs_msg_->at(i).transform.translation.z);

        Sophus::SE3<double> pose(q_pose.toRotationMatrix(), t_pose);

        Eigen::Matrix4d mat_pose = pose.matrix();
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                vtk_frame_->vtk_tfs_mat.at(i)->SetElement(row, col, mat_pose(row, col));
            }
        }
        vtk_frame_->vtk_tfs.at(i)->SetMatrix(vtk_frame_->vtk_tfs_mat.at(i));
        vtk_frame_->vtk_tfs.at(i)->Modified();
//        qDebug() << tfs_msg_->at(i).header.frame_id.c_str() << "\t" << tfs_msg_->at(i).child_frame_id.c_str();
    }

    if (flag_re_win_) {
        emit sigSetRenderWin();
        flag_re_win_ = false;
    }
    if (vtk_frame_->vtk_flag) {
        emit sigRenderFrame();
        vtk_frame_->vtk_flag = false;
    }

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << " update frame thread id:" << QThread::currentThreadId();
#endif
}

void ThreadFrame::initFrame() {
    std::unique_lock<QMutex> ul(vtk_frame_->vtk_mutex);

    int idn = node_->params_heart_->servo_idn_val;
    tfs_frame_id_->resize(idn + 4);
    tfs_msg_->resize(idn + 4);

    vtk_frame_->vtk_tfs.resize(idn + 4);
    vtk_frame_->vtk_tfs_mat.resize(idn + 4);
    vtk_frame_->vtk_axes_actors.resize(idn + 4);

    // 0:snake_world-->snake_base
    // 1:snake_base-->snake_camera_1
    // 2:snake_base-->snake_camera_2
    // 3:snake_base-->snake_imu
    // 4:snake_base-->link_1
    // 5:link_1-->link_2
    // 6:link_2-->...
    // n+3:link_n-1-->link_n.
    for (int i = 0; i < static_cast<int>(tfs_frame_id_->size()); i++) {
        if (i == 0) {
            tfs_frame_id_->at(i) = std::tuple<std::string, std::string>("snake_world", "snake_base");
        } else if (i == 1) {
            tfs_frame_id_->at(i) = std::tuple<std::string, std::string>("snake_base", "snake_camera_1");
        } else if (i == 2) {
            tfs_frame_id_->at(i) = std::tuple<std::string, std::string>("snake_base", "snake_camera_2");
        } else if (i == 3) {
            tfs_frame_id_->at(i) = std::tuple<std::string, std::string>("snake_base", "snake_imu");
        } else if (i == 4) {
            tfs_frame_id_->at(i) = std::tuple<std::string, std::string>("snake_base", "snake_link_1");
        } else {
            std::string str1 = "snake_link_" + std::to_string(i - 4);
            std::string str2 = "snake_link_" + std::to_string(i - 3);
            tfs_frame_id_->at(i) = std::tuple<std::string, std::string>(str1, str2);
        }
    }

    for (int i = 0; i < static_cast<int>(vtk_frame_->vtk_tfs.size()); i++) {
        vtk_frame_->vtk_tfs.at(i) = vtkSmartPointer<vtkTransform>::New();
        vtk_frame_->vtk_tfs.at(i)->PreMultiply();
        if (i == 0) {
            // TODO.
        } else if (i == 1 || i == 2 || i == 3 || i == 4) {
            vtk_frame_->vtk_tfs.at(i)->SetInput(vtk_frame_->vtk_tfs.at(0));
        } else {
            vtk_frame_->vtk_tfs.at(i)->SetInput(vtk_frame_->vtk_tfs.at(i - 1));
        }
    }

    for (auto &tf_mat: vtk_frame_->vtk_tfs_mat) {
        tf_mat = vtkSmartPointer<vtkMatrix4x4>::New();
    }

    for (int i = 0; i < static_cast<int>(vtk_frame_->vtk_axes_actors.size()); i++) {
        vtk_frame_->vtk_axes_actors.at(i) = vtkSmartPointer<vtkAxesActor>::New();

        int resolution = 50;
        double length_total = 0.1;
        double radius_cylinder = 0.02;

        vtk_frame_->vtk_axes_actors.at(i)->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
        vtk_frame_->vtk_axes_actors.at(i)->SetTipType(vtkAxesActor::CONE_TIP);
        vtk_frame_->vtk_axes_actors.at(i)->AxisLabelsOff(); // Don't show label font.
        vtk_frame_->vtk_axes_actors.at(i)->SetUserTransform(vtk_frame_->vtk_tfs.at(i));

        vtk_frame_->vtk_axes_actors.at(i)->SetConeResolution(resolution);
        vtk_frame_->vtk_axes_actors.at(i)->SetCylinderResolution(resolution);
        vtk_frame_->vtk_axes_actors.at(i)->SetSphereResolution(resolution);
        vtk_frame_->vtk_axes_actors.at(i)->SetTotalLength(length_total, length_total, length_total);
        vtk_frame_->vtk_axes_actors.at(i)->SetCylinderRadius(radius_cylinder);

        std::string str1 = "x" + std::to_string(i);
        std::string str2 = "y" + std::to_string(i);
        std::string str3 = "z" + std::to_string(i);
        vtk_frame_->vtk_axes_actors.at(i)->SetXAxisLabelText(str1.c_str());
        vtk_frame_->vtk_axes_actors.at(i)->SetYAxisLabelText(str2.c_str());
        vtk_frame_->vtk_axes_actors.at(i)->SetZAxisLabelText(str3.c_str());

        vtkCaptionActor2D *xAxisCaptionActor = vtk_frame_->vtk_axes_actors.at(i)->GetXAxisCaptionActor2D();
        vtkCaptionActor2D *yAxisCaptionActor = vtk_frame_->vtk_axes_actors.at(i)->GetYAxisCaptionActor2D();
        vtkCaptionActor2D *zAxisCaptionActor = vtk_frame_->vtk_axes_actors.at(i)->GetZAxisCaptionActor2D();

        // TODO.
        vtkTextProperty *captionTextProperty;
        captionTextProperty = xAxisCaptionActor->GetCaptionTextProperty();
        captionTextProperty->SetColor(1, 0, 0); // change X font color.
        captionTextProperty->SetFontSize(2);
        captionTextProperty = yAxisCaptionActor->GetCaptionTextProperty();
        captionTextProperty->SetColor(0, 1, 0); // change Y font color.
        captionTextProperty->SetFontSize(2);
        captionTextProperty = zAxisCaptionActor->GetCaptionTextProperty();
        captionTextProperty->SetColor(0, 0, 1); // change Z font color.
        captionTextProperty->SetFontSize(2);
    }

    {
        // Snake world axes.
        vtk_frame_->vtk_axes_world = vtkSmartPointer<vtkAxesActor>::New();
        int resolution = 50;
        double length_total = 1.0;
        double radius_cylinder = 0.01;
        double radius_sphere = 0.1;

        vtk_frame_->vtk_axes_world->SetShaftType(vtkAxesActor::LINE_SHAFT);
        vtk_frame_->vtk_axes_world->SetTipType(vtkAxesActor::SPHERE_TIP);
        vtk_frame_->vtk_axes_world->AxisLabelsOff();

        vtk_frame_->vtk_axes_world->SetConeResolution(resolution);
        vtk_frame_->vtk_axes_world->SetCylinderResolution(resolution);
        vtk_frame_->vtk_axes_world->SetSphereResolution(resolution);
        vtk_frame_->vtk_axes_world->SetTotalLength(length_total, length_total, length_total);
        vtk_frame_->vtk_axes_world->SetCylinderRadius(radius_cylinder);
        vtk_frame_->vtk_axes_world->SetSphereRadius(radius_sphere);

        std::string str1 = "snake_world_x";
        std::string str2 = "snake_world_y";
        std::string str3 = "snake_world_z";
        vtk_frame_->vtk_axes_world->SetXAxisLabelText(str1.c_str());
        vtk_frame_->vtk_axes_world->SetYAxisLabelText(str2.c_str());
        vtk_frame_->vtk_axes_world->SetZAxisLabelText(str3.c_str());

        vtkCaptionActor2D *xAxisCaptionActor = vtk_frame_->vtk_axes_world->GetXAxisCaptionActor2D();
        vtkCaptionActor2D *yAxisCaptionActor = vtk_frame_->vtk_axes_world->GetYAxisCaptionActor2D();
        vtkCaptionActor2D *zAxisCaptionActor = vtk_frame_->vtk_axes_world->GetZAxisCaptionActor2D();

        vtkTextProperty *captionTextProperty;
        captionTextProperty = xAxisCaptionActor->GetCaptionTextProperty();
        captionTextProperty->SetColor(1, 0, 0); // change X font color.
        captionTextProperty->SetFontSize(2);
        captionTextProperty = yAxisCaptionActor->GetCaptionTextProperty();
        captionTextProperty->SetColor(0, 1, 0); // change Y font color.
        captionTextProperty->SetFontSize(2);
        captionTextProperty = zAxisCaptionActor->GetCaptionTextProperty();
        captionTextProperty->SetColor(0, 0, 1); // change Z font color.
        captionTextProperty->SetFontSize(2);
    }

    vtk_frame_->vtk_cube_actor = vtkSmartPointer<vtkCubeAxesActor>::New();
    // font size.
    vtk_frame_->vtk_cube_actor->SetScreenSize(20);
    // VTK_GRID_LINES_ALL=0, VTK_GRID_LINES_CLOSEST=1, VTK_GRID_LINES_FURTHEST=2
    vtk_frame_->vtk_cube_actor->SetGridLineLocation(2);
    vtk_frame_->vtk_cube_actor->SetXAxisRange(-1, 1);
    vtk_frame_->vtk_cube_actor->SetYAxisRange(-1, 1);
    vtk_frame_->vtk_cube_actor->SetZAxisRange(-1, 1);
    vtk_frame_->vtk_cube_actor->SetDrawXGridlines(true);
    vtk_frame_->vtk_cube_actor->SetDrawYGridlines(true);
    vtk_frame_->vtk_cube_actor->SetDrawZGridlines(true);
    vtk_frame_->vtk_cube_actor->SetXAxisTickVisibility(true);
    vtk_frame_->vtk_cube_actor->SetYAxisTickVisibility(true);
    vtk_frame_->vtk_cube_actor->SetZAxisTickVisibility(true);
    vtk_frame_->vtk_cube_actor->SetXAxisMinorTickVisibility(true);
    vtk_frame_->vtk_cube_actor->SetYAxisMinorTickVisibility(true);
    vtk_frame_->vtk_cube_actor->SetZAxisMinorTickVisibility(true);

    vtk_frame_->vtk_render = vtkSmartPointer<vtkOpenGLRenderer>::New();
    // Paper color.
    vtk_frame_->vtk_render->SetBackground(0.15, 0.15, 0.15);
//    vtk_frame_->vtk_render->SetBackground(1.0, 1.0, 1.0);
    vtk_frame_->vtk_cube_actor->SetCamera(vtk_frame_->vtk_render->GetActiveCamera());
//    vtk_frame_->vtk_render->AddActor(vtk_frame_->vtk_cube_actor);

    for (const auto &ax_actor: vtk_frame_->vtk_axes_actors) {
        vtk_frame_->vtk_render->AddActor(ax_actor);
    }
    vtk_frame_->vtk_render->AddActor(vtk_frame_->vtk_axes_world);

    vtk_frame_->vtk_render_win = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtk_frame_->vtk_render_win->AddRenderer(vtk_frame_->vtk_render);

    vtk_frame_->vtk_style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    vtk_frame_->vtk_iren = vtkSmartPointer<vtkGenericRenderWindowInteractor>::New();
//    vtk_frame_->vtk_iren->SetRenderWindow(vtk_frame_->vtk_render_win);
    vtk_frame_->vtk_iren->SetInteractorStyle(vtk_frame_->vtk_style);
    vtk_frame_->vtk_iren->Initialize();
    vtk_frame_->vtk_iren->Start();

    flag_re_win_ = true;

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << " init frame thread id:" << QThread::currentThreadId();
#endif
}
