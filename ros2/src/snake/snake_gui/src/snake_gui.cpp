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

#include "snake_gui/snake_gui.h"
#include "snake_gui/ui_snake_gui.h"

SnakeGUI::SnakeGUI(QWidget *parent) : QMainWindow(parent) {
    ui = std::make_shared<Ui::SnakeGUI>();
    node_ = std::make_shared<snake::SnakeGUINode>(node_name);
    ui->setupUi(this);

    // Paper color.
    // setStyleSheet("background-color: rgb(255, 255, 255);color: rgb(0, 0, 0);");

    initNode(parent);
    initQCPJointAngle(parent);
    intiSnakeFrame(parent);

    dlg_param_ = std::make_shared<DialogParamControl>(parent);
    dlg_custom_ = std::make_shared<DialogCustomMode>(parent);
    dlg_typical_ = std::make_shared<DialogTypicalMode>(parent);
    dlg_arc_ = std::make_shared<DialogArcMode>(parent);
    dlg_spiral_ = std::make_shared<DialogSpiralMode>(parent);
//    dlg_pid_ = std::make_shared<DialogPID>(parent);

//    dlg_param_->setStyleSheet("QDialog{background-color: rgb(255, 255, 255)}");
//    dlg_custom_->setStyleSheet("QDialog{background-color: rgb(255, 255, 255)}");
//    dlg_typical_->setStyleSheet("QDialog{background-color: rgb(255, 255, 255)}");
//    dlg_arc_->setStyleSheet("QDialog{background-color: rgb(255, 255, 255)}");
//    dlg_spiral_->setStyleSheet("QDialog{background-color: rgb(255, 255, 255)}");
//    dlg_pid_->setStyleSheet("QDialog{background-color: rgb(255, 255, 255)}");

    // Set modeless dialog.
    dlg_param_->setModal(false);
    dlg_custom_->setModal(false);
    dlg_typical_->setModal(false);
    dlg_arc_->setModal(false);
    dlg_spiral_->setModal(false);
//    dlg_pid_->setModal(false);

    // Associate client.
    dlg_param_->setClient(node_, node_->client_heart_, node_->client_cpg_hopf_);
    dlg_custom_->setClient(node_, node_->client_heart_, node_->client_cpg_hopf_);
    dlg_typical_->setClient(node_, node_->client_heart_, node_->client_cpg_hopf_);
    dlg_arc_->setClient(node_, node_->client_heart_, node_->client_cpg_hopf_);
    dlg_spiral_->setClient(node_, node_->client_heart_, node_->client_cpg_hopf_);
//    dlg_pid_->setClient(node_);

    // Control start and stop.
    connect(SnakeGUI::ui->action_start_stop, &QAction::triggered, this, &SnakeGUI::onStartStop);
    ui->action_start_stop->triggered(node_->params_cpg_hopf_->cpg_start_val);

    // Control torque enable.
    connect(SnakeGUI::ui->action_torque_enable, &QAction::triggered, this, &SnakeGUI::onTorqueEnable);
    ui->action_torque_enable->triggered(node_->params_cpg_hopf_->cpg_torque_val);

    // Control parameters.
    connect(SnakeGUI::ui->action_param_control, &QAction::triggered, this, &SnakeGUI::onParamControl);
    connect(SnakeGUI::ui->action_custom_mode, &QAction::triggered, this, &SnakeGUI::onCustomMode);
    connect(SnakeGUI::ui->action_typical_mode, &QAction::triggered, this, &SnakeGUI::onTypicalMode);
    connect(SnakeGUI::ui->action_arc_mode, &QAction::triggered, this, &SnakeGUI::onArcMode);
    connect(SnakeGUI::ui->action_spiral_mode, &QAction::triggered, this, &SnakeGUI::onSpiralMode);
//    connect(SnakeGUI::ui->action_pid, &QAction::triggered, this, &SnakeGUI::onPID);

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "SnakeGUI thread id: " << QThread::currentThreadId();
#endif
}

SnakeGUI::~SnakeGUI() {
    qcp_thread_execute_->quit();
    qcp_thread_execute_->wait();

    qcp_thread_feedback_->quit();
    qcp_thread_feedback_->wait();

    vtk_qthread_->quit();
    vtk_qthread_->wait();
}

double SnakeGUI::calTnterval(const std_msgs::msg::Header &header_1, const std_msgs::msg::Header &header_2) {
    double interval;
    interval = (header_2.stamp.sec - header_1.stamp.sec) +
               (header_2.stamp.nanosec * 1e-9 - header_1.stamp.nanosec * 1e-9);
    return interval;
}

void SnakeGUI::initNode(QObject *parent) {
    group_execute_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group_feedback_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    option_execute_ = rclcpp::SubscriptionOptions();
    option_feedback_ = rclcpp::SubscriptionOptions();
    option_execute_.callback_group = group_execute_;
    option_feedback_.callback_group = group_feedback_;

    auto bind_execute = std::bind(&SnakeGUI::cbExecute, this, std::placeholders::_1);
    sub_execute_ = node_->create_subscription<snake_msgs::msg::ServoExecute>(sub_execute_str_,
                                                                             queue_size_,
                                                                             bind_execute,
                                                                             option_execute_);

    auto bind_feedback = std::bind(&SnakeGUI::cbFeedback, this, std::placeholders::_1);
    sub_feedback_ = node_->create_subscription<snake_msgs::msg::ServoFeedback>(sub_feedback_str_,
                                                                               queue_size_,
                                                                               bind_feedback,
                                                                               option_feedback_);

    thread_callback_ = std::make_shared<ThreadCallback>(nullptr);
    connect(thread_callback_.get(), &ThreadCallback::finished, thread_callback_.get(), &QObject::deleteLater);
    thread_callback_->setNode(node_);
    thread_callback_->start();

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "init node thread id:" << QThread::currentThreadId();
#endif
}

void SnakeGUI::initQCPJointAngle(QObject *parent) {
    qcp_curve_color_.append(QColor(0, 114, 189));
    qcp_curve_color_.append(QColor(217, 83, 25));
    qcp_curve_color_.append(QColor(237, 177, 32));
    qcp_curve_color_.append(QColor(126, 47, 142));
    qcp_curve_color_.append(QColor(119, 172, 48));
    qcp_curve_color_.append(QColor(77, 190, 238));
    qcp_curve_color_.append(QColor(162, 20, 47));
    qcp_curve_color_.append(QColor(255, 214, 10));
    qcp_curve_color_.append(QColor(101, 130, 253));
    qcp_curve_color_.append(QColor(255, 69, 58));
    qcp_curve_color_.append(QColor(0, 163, 163));
    qcp_curve_color_.append(QColor(203, 132, 93));

    qcp_curve_style_.append(QCPScatterStyle::ssCircle);
    qcp_curve_style_.append(QCPScatterStyle::ssDisc);
    qcp_curve_style_.append(QCPScatterStyle::ssSquare);
    qcp_curve_style_.append(QCPScatterStyle::ssDiamond);
    qcp_curve_style_.append(QCPScatterStyle::ssStar);
    qcp_curve_style_.append(QCPScatterStyle::ssTriangle);
    qcp_curve_style_.append(QCPScatterStyle::ssTriangleInverted);
    qcp_curve_style_.append(QCPScatterStyle::ssCrossSquare);
    qcp_curve_style_.append(QCPScatterStyle::ssCrossCircle);
    qcp_curve_style_.append(QCPScatterStyle::ssPlusSquare);
    qcp_curve_style_.append(QCPScatterStyle::ssPlusCircle);

//    qcp_thread_execute_ = std::make_shared<QThread>();
//    qcp_thread_feedback_ = std::make_shared<QThread>();
//    ui->qcp_execute_pos->moveToThread(qcp_thread_execute_.get());
//    ui->qcp_feedback_pos->moveToThread(qcp_thread_feedback_.get());
//    connect(qcp_thread_execute_.get(), &QThread::finished, ui->qcp_execute_pos, &QObject::deleteLater);
//    connect(qcp_thread_feedback_.get(), &QThread::finished, ui->qcp_feedback_pos, &QObject::deleteLater);

    connect(this, &SnakeGUI::sigReplotExecute, ui->qcp_execute_pos, &QCustomPlot::replot, Qt::AutoConnection);
    connect(this, &SnakeGUI::sigReplotFeedback, ui->qcp_feedback_pos, &QCustomPlot::replot, Qt::AutoConnection);

    ui->qcp_execute_pos->setOpenGl(true);
    ui->qcp_feedback_pos->setOpenGl(true);
    ui->qcp_execute_pos->legend->setFont(QFont("Helvetica", 12));
    ui->qcp_feedback_pos->legend->setFont(QFont("Helvetica", 12));
    ui->qcp_execute_pos->yAxis->setLabel("Execute position");
    ui->qcp_feedback_pos->yAxis->setLabel("Feedback position");
    // Paper color.
    ui->qcp_execute_pos->setBackground(QColor(40, 40, 40));
    ui->qcp_feedback_pos->setBackground(QColor(40, 40, 40));
//    ui->qcp_execute_pos->setBackground(QColor(255, 255, 255));
//    ui->qcp_feedback_pos->setBackground(QColor(255, 255, 255));

    ui->qcp_execute_pos->legend->setVisible(false);
    ui->qcp_execute_pos->xAxis->setVisible(true);
    ui->qcp_execute_pos->xAxis->setLabel("Time");
    // Paper color.
    ui->qcp_execute_pos->xAxis->setLabelColor(QColor(0, 160, 230));
    ui->qcp_execute_pos->xAxis->setTickLabelColor(Qt::white);
//    ui->qcp_execute_pos->xAxis->setLabelColor(QColor(0, 0, 0));
//    ui->qcp_execute_pos->xAxis->setTickLabelColor(Qt::black);
    // Paper color.
    ui->qcp_execute_pos->xAxis->setBasePen(QPen(QColor(32, 178, 170)));
    ui->qcp_execute_pos->xAxis->setTickPen(QPen(QColor(128, 0, 255)));
    ui->qcp_execute_pos->xAxis->setSubTickPen(QColor(255, 165, 0));
//    ui->qcp_execute_pos->xAxis->setBasePen(QPen(QColor(0, 0, 0)));
//    ui->qcp_execute_pos->xAxis->setTickPen(QPen(QColor(0, 0, 0)));
//    ui->qcp_execute_pos->xAxis->setSubTickPen(QColor(0, 0, 0));
    QFont x_axis_font = ui->qcp_execute_pos->xAxis->labelFont();
    x_axis_font.setPixelSize(16);
    x_axis_font.setFamily("Helvetica");
    ui->qcp_execute_pos->xAxis->setLabelFont(x_axis_font);
    QFont x_tick_font = ui->qcp_execute_pos->xAxis->tickLabelFont();
    x_tick_font.setPixelSize(16);
    x_tick_font.setFamily("Helvetica");
    ui->qcp_execute_pos->xAxis->setTickLabelFont(x_tick_font);

    ui->qcp_execute_pos->yAxis->setVisible(true);
    // Paper color.
    ui->qcp_execute_pos->yAxis->setLabelColor(QColor(0, 160, 230));
    ui->qcp_execute_pos->yAxis->setTickLabelColor(Qt::white);
//    ui->qcp_execute_pos->yAxis->setLabelColor(QColor(0, 0, 0));
//    ui->qcp_execute_pos->yAxis->setTickLabelColor(Qt::black);
    // Paper color.
    ui->qcp_execute_pos->yAxis->setBasePen(QPen(QColor(32, 178, 170)));
    ui->qcp_execute_pos->yAxis->setTickPen(QPen(QColor(128, 0, 255)));
    ui->qcp_execute_pos->yAxis->setSubTickPen(QColor(255, 165, 0));
//    ui->qcp_execute_pos->yAxis->setBasePen(QPen(QColor(0, 0, 0)));
//    ui->qcp_execute_pos->yAxis->setTickPen(QPen(QColor(0, 0, 0)));
//    ui->qcp_execute_pos->yAxis->setSubTickPen(QColor(0, 0, 0));
    QFont y_axis_font = ui->qcp_execute_pos->yAxis->labelFont();
    y_axis_font.setPixelSize(16);
    y_axis_font.setFamily("Helvetica");
    ui->qcp_execute_pos->yAxis->setLabelFont(y_axis_font);
    QFont y_tick_font = ui->qcp_execute_pos->yAxis->tickLabelFont();
    y_tick_font.setPixelSize(16);
    y_tick_font.setFamily("Helvetica");
    ui->qcp_execute_pos->yAxis->setTickLabelFont(y_tick_font);


    ui->qcp_feedback_pos->legend->setVisible(false);
    ui->qcp_feedback_pos->xAxis->setVisible(true);
    ui->qcp_feedback_pos->xAxis->setLabel("Time");
    // Paper color.
    ui->qcp_feedback_pos->xAxis->setLabelColor(QColor(0, 160, 230));
    ui->qcp_feedback_pos->xAxis->setTickLabelColor(Qt::white);
//    ui->qcp_feedback_pos->xAxis->setLabelColor(QColor(0, 0, 0));
//    ui->qcp_feedback_pos->xAxis->setTickLabelColor(Qt::black);
    // Paper color.
    ui->qcp_feedback_pos->xAxis->setBasePen(QPen(QColor(32, 178, 170)));
    ui->qcp_feedback_pos->xAxis->setTickPen(QPen(QColor(128, 0, 255)));
    ui->qcp_feedback_pos->xAxis->setSubTickPen(QColor(255, 165, 0));
//    ui->qcp_feedback_pos->xAxis->setBasePen(QPen(QColor(0, 0, 0)));
//    ui->qcp_feedback_pos->xAxis->setTickPen(QPen(QColor(0, 0, 0)));
//    ui->qcp_feedback_pos->xAxis->setSubTickPen(QColor(0, 0, 0));
    QFont x_axis_font2 = ui->qcp_feedback_pos->xAxis->labelFont();
    x_axis_font2.setPixelSize(16);
    x_axis_font2.setFamily("Helvetica");
    ui->qcp_feedback_pos->xAxis->setLabelFont(x_axis_font2);
    QFont x_tick_font2 = ui->qcp_feedback_pos->xAxis->tickLabelFont();
    x_tick_font2.setPixelSize(16);
    x_tick_font2.setFamily("Helvetica");
    ui->qcp_feedback_pos->xAxis->setTickLabelFont(x_tick_font2);

    ui->qcp_feedback_pos->yAxis->setVisible(true);
    // Paper color.
    ui->qcp_feedback_pos->yAxis->setLabelColor(QColor(0, 160, 230));
    ui->qcp_feedback_pos->yAxis->setTickLabelColor(Qt::white);
//    ui->qcp_feedback_pos->yAxis->setLabelColor(QColor(0, 0, 0));
//    ui->qcp_feedback_pos->yAxis->setTickLabelColor(Qt::black);
    // Paper color.
    ui->qcp_feedback_pos->yAxis->setBasePen(QPen(QColor(32, 178, 170)));
    ui->qcp_feedback_pos->yAxis->setTickPen(QPen(QColor(128, 0, 255)));
    ui->qcp_feedback_pos->yAxis->setSubTickPen(QColor(255, 165, 0));
//    ui->qcp_feedback_pos->yAxis->setBasePen(QPen(QColor(0, 0, 0)));
//    ui->qcp_feedback_pos->yAxis->setTickPen(QPen(QColor(0, 0, 0)));
//    ui->qcp_feedback_pos->yAxis->setSubTickPen(QColor(0, 0, 0));
    QFont y_axis_font2 = ui->qcp_feedback_pos->yAxis->labelFont();
    y_axis_font2.setPixelSize(16);
    y_axis_font2.setFamily("Helvetica");
    ui->qcp_feedback_pos->yAxis->setLabelFont(y_axis_font2);
    QFont y_tick_font2 = ui->qcp_feedback_pos->yAxis->tickLabelFont();
    y_tick_font2.setPixelSize(16);
    y_tick_font2.setFamily("Helvetica");
    ui->qcp_feedback_pos->yAxis->setTickLabelFont(y_tick_font2);

    node_->client_heart_->get_parameters(
            {
                    node_->params_heart_->heart_rate_str,
                    node_->params_heart_->servo_idn_str,
                    node_->params_heart_->link_length_str,
            },
            [this](const std::shared_future<std::vector<rclcpp::Parameter>> &params) {
                node_->params_heart_->heart_rate_val = static_cast<int>(params.get().at(0).as_int());
                node_->params_heart_->servo_idn_val = static_cast<int>(params.get().at(1).as_int());
                node_->params_heart_->link_length_val = static_cast<double>(params.get().at(2).as_double());
                RCLCPP_INFO(node_->get_logger(), "Get parameters successfully.");
            });

    qcp_time_start_.stamp = node_->now();

    qcp_time_execute_.clear();
    qcp_time_feedback_.clear();
    qcp_data_execute_.clear();
    qcp_data_feedback_.clear();
    qcp_data_execute_.resize(node_->params_heart_->servo_idn_val);
    qcp_data_feedback_.resize(node_->params_heart_->servo_idn_val);

    ui->qcp_execute_pos->clearGraphs();
    ui->qcp_feedback_pos->clearGraphs();
    for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
        ui->qcp_execute_pos->addGraph(ui->qcp_execute_pos->xAxis, ui->qcp_execute_pos->yAxis);
        ui->qcp_execute_pos->graph(i)->setPen(qcp_curve_color_.at(i % qcp_curve_color_.size()));
//        ui->qcp_execute_pos->graph(i)->setScatterStyle(qcp_curve_style_.at(i % qcp_curve_style_.size()));
        ui->qcp_execute_pos->graph(i)->setName(QString("execute_pos_%1").arg(i + 1));

        ui->qcp_feedback_pos->addGraph(ui->qcp_feedback_pos->xAxis, ui->qcp_feedback_pos->yAxis);
        ui->qcp_feedback_pos->graph(i)->setPen(qcp_curve_color_.at(i % qcp_curve_color_.size()));
//        ui->qcp_feedback_pos->graph(i)->setScatterStyle(qcp_curve_style_.at(i % qcp_curve_style_.size()));
        ui->qcp_feedback_pos->graph(i)->setName(QString("feedback_pos_%1").arg(i + 1));
    }

    ui->qcp_execute_pos->xAxis->rescale(true);
    ui->qcp_execute_pos->yAxis->rescale(true);
    ui->qcp_execute_pos->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->qcp_execute_pos->show();

    ui->qcp_feedback_pos->xAxis->rescale(true);
    ui->qcp_feedback_pos->yAxis->rescale(true);
    ui->qcp_feedback_pos->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->qcp_feedback_pos->show();

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "init QCP thread id: " << QThread::currentThreadId();
    qDebug() << "init QCP execute thread id: " << ui->qcp_execute_pos->thread()->currentThreadId();
    qDebug() << "init QCP feedback thread id: " << ui->qcp_feedback_pos->thread()->currentThreadId();
#endif
}

void SnakeGUI::intiSnakeFrame(QObject *parent) {
    vtkOutputWindow::SetGlobalWarningDisplay(1);

    vtk_tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    vtk_tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*vtk_tf2_buffer_);

    vtk_frame_ = std::make_shared<snake::VTKFrame>();
    vtk_frame_->vtk_flag = true;
    vtk_frame_->vtk_tfs.resize(0);
    vtk_frame_->vtk_tfs_mat.resize(0);
    vtk_frame_->vtk_axes_actors.resize(0);

    // Creat new thread to render slam window.
    vtk_thread_ = std::make_shared<ThreadFrame>(node_,
                                                vtk_tf2_buffer_,
                                                vtk_frame_);

    vtk_qthread_ = std::make_shared<QThread>();
    vtk_thread_->moveToThread(vtk_qthread_.get());

    // Create a timer when the thread starts.
//    connect(vtk_qthread_.get(), &QThread::started, vtk_thread_.get(), &ThreadFrame::onCreateTimer);
    connect(vtk_qthread_.get(), &QThread::finished, vtk_thread_.get(), &QObject::deleteLater);
    connect(vtk_thread_.get(), &ThreadFrame::sigSetRenderWin, this, &SnakeGUI::onSetRenderWin, Qt::QueuedConnection);
    connect(vtk_thread_.get(), &ThreadFrame::sigRenderFrame, this, &SnakeGUI::onRenderFrame, Qt::QueuedConnection);

    vtk_qthread_->start();

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "init frame thread id:" << QThread::currentThreadId();
#endif
}

void SnakeGUI::onStartStop(bool checked) {
    ui->action_start_stop->setChecked(checked);
    if (checked) {
        ui->action_start_stop->setText(QString("Running"));
        node_->params_cpg_hopf_->cpg_start_val = true;

        node_->client_cpg_hopf_->set_parameters(
                {
                        rclcpp::Parameter(node_->params_cpg_hopf_->cpg_start_str,
                                          node_->params_cpg_hopf_->cpg_start_val),
                },
                [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                    RCLCPP_INFO(node_->get_logger(), "Running.[%d]", params.get().size());
                });

        ui->action_torque_enable->triggered(true);
    } else {
        ui->action_start_stop->setText(QString("Stopped"));
        node_->params_cpg_hopf_->cpg_start_val = false;

        node_->client_cpg_hopf_->set_parameters(
                {
                        rclcpp::Parameter(node_->params_cpg_hopf_->cpg_start_str,
                                          node_->params_cpg_hopf_->cpg_start_val),
                },
                [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                    RCLCPP_INFO(node_->get_logger(), "Stopped.[%d]", params.get().size());
                });
    }
}

void SnakeGUI::onTorqueEnable(bool checked) {
    ui->action_torque_enable->setChecked(checked);
    if (checked) {
        ui->action_torque_enable->setText(QString("LoadedTorque"));
        node_->params_cpg_hopf_->cpg_torque_val = true;
    } else {
        ui->action_torque_enable->setText(QString("UnloadedTorque"));
        node_->params_cpg_hopf_->cpg_torque_val = false;

        node_->client_cpg_hopf_->set_parameters(
                {
                        rclcpp::Parameter(node_->params_cpg_hopf_->cpg_torque_str,
                                          node_->params_cpg_hopf_->cpg_torque_val),
                },
                [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                    RCLCPP_INFO(node_->get_logger(), "UnloadedTorque.[%d]", params.get().size());
                });

        ui->action_start_stop->triggered(false);
    }
}

void SnakeGUI::onParamControl() {
    if (dlg_param_->isHidden()) {
        dlg_param_->show();
    } else if (!dlg_param_->isActiveWindow()) {
        dlg_param_->activateWindow();
    }

    dlg_param_->getParam();
    dlg_param_->setBoxValue();
}

void SnakeGUI::onCustomMode() {
    if (dlg_custom_->isHidden()) {
        dlg_custom_->show();
    } else if (!dlg_custom_->isActiveWindow()) {
        dlg_custom_->activateWindow();
    }

    dlg_custom_->getParam();
    dlg_custom_->setBoxValue();
}

void SnakeGUI::onTypicalMode() {
    if (dlg_typical_->isHidden()) {
        dlg_typical_->show();
    } else if (!dlg_typical_->isActiveWindow()) {
        dlg_typical_->activateWindow();
    }

    dlg_typical_->getParam();
    dlg_typical_->setBoxValue();
}

void SnakeGUI::onArcMode() {
    if (dlg_arc_->isHidden()) {
        dlg_arc_->show();
    } else if (!dlg_arc_->isActiveWindow()) {
        dlg_arc_->activateWindow();
    }

    dlg_arc_->getParam();
    dlg_arc_->setBoxValue();
}

void SnakeGUI::onSpiralMode() {
    if (dlg_spiral_->isHidden()) {
        dlg_spiral_->show();
    } else if (!dlg_spiral_->isActiveWindow()) {
        dlg_spiral_->activateWindow();
    }

    dlg_spiral_->getParam();
    dlg_spiral_->setBoxValue();
}

//void SnakeGUI::onPID() {
//    if (dlg_pid_->isHidden()) {
//        dlg_pid_->show();
//    } else if (!dlg_pid_->isActiveWindow()) {
//        dlg_pid_->activateWindow();
//    }
//
//    dlg_pid_->getParam();
//    dlg_pid_->setBoxValue();
//}

void SnakeGUI::onSetRenderWin() {
    std::unique_lock<QMutex> ul(vtk_frame_->vtk_mutex);

    ui->snake_view->setRenderWindow(vtk_frame_->vtk_render_win);
    ui->snake_view->setEnableHiDPI(true);

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "snake_view thread id:" << ui->snake_view->thread()->currentThreadId();
#endif
}

void SnakeGUI::onRenderFrame() {
    std::unique_lock<QMutex> ul(vtk_frame_->vtk_mutex);

    vtk_frame_->vtk_render_win->Render();
    ui->snake_view->renderWindow();

    vtk_frame_->vtk_flag = true;

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "render thread id:" << QThread::currentThreadId();
#endif
}

void SnakeGUI::cbExecute(const snake_msgs::msg::ServoExecute::SharedPtr msg_execute) {
    qcp_mutex_execute_.lock();

    msg_execute_.header = msg_execute->header;
    msg_execute_.seq = msg_execute->seq;
    msg_execute_.id = msg_execute->id;
    msg_execute_.pos = msg_execute->pos;
    msg_execute_.speed = msg_execute->speed;
    msg_execute_.acc = msg_execute->acc;

#ifdef SNAKE_GUI_DEBUG_MODE
    RCLCPP_INFO(node_->get_logger(),
                "callbackExecute===>Seq:[%d]\tID[0] Pos:[%f]\tID[1] Pos:[%f]",
                msg_execute_.seq,
                msg_execute_.pos.at(0),
                msg_execute_.pos.at(1));
#endif

    node_->params_heart_->servo_idn_val = static_cast<int>(msg_execute_.id.size());
    if (qcp_data_execute_.size() != node_->params_heart_->servo_idn_val) {
        qcp_time_execute_.clear();
        qcp_data_execute_.clear();
        qcp_data_execute_.resize(node_->params_heart_->servo_idn_val);

        ui->qcp_execute_pos->clearGraphs();
        for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
            ui->qcp_execute_pos->addGraph(ui->qcp_execute_pos->xAxis, ui->qcp_execute_pos->yAxis);
            ui->qcp_execute_pos->graph(i)->setPen(qcp_curve_color_.at(i % qcp_curve_color_.size()));
//            ui->qcp_execute_pos->graph(i)->setScatterStyle(qcp_curve_style_.at(i % qcp_curve_style_.size()));
            ui->qcp_execute_pos->graph(i)->setName(QString("joint_pos_%1").arg(i + 1));
        }
    }

    // Update time.
    if (qcp_time_execute_.size() >= qcp_data_num_) {
        qcp_time_execute_.removeFirst();
        for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
            qcp_data_execute_[i].removeFirst();
        }
    }
    qcp_time_execute_.append(calTnterval(qcp_time_start_, msg_execute_.header));
    for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
        qcp_data_execute_[i].append(msg_execute_.pos[i] / M_PI * 180);
        ui->qcp_execute_pos->graph(i)->setData(qcp_time_execute_, qcp_data_execute_.at(i));
    }

    ui->qcp_execute_pos->xAxis->rescale(true);
    ui->qcp_execute_pos->yAxis->rescale(true);

    emit sigReplotExecute(QCustomPlot::rpQueuedReplot);
//    ui->qcp_execute_pos->replot(QCustomPlot::rpQueuedReplot);

    qcp_mutex_execute_.unlock();

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "execute thread qt id:" << QThread::currentThreadId();
    qDebug() << "execute thread qcp id:" << ui->qcp_execute_pos->thread()->currentThreadId();
#endif
}

void SnakeGUI::cbFeedback(const snake_msgs::msg::ServoFeedback::SharedPtr msg_feedback) {
    qcp_mutex_feedback_.lock();

    msg_feedback_.header = msg_feedback->header;
    msg_feedback_.seq = msg_feedback->seq;
    msg_feedback_.id = msg_feedback->id;
    msg_feedback_.position = msg_feedback->position;
    msg_feedback_.speed = msg_feedback->speed;
    msg_feedback_.load = msg_feedback->load;
    msg_feedback_.voltage = msg_feedback->voltage;
    msg_feedback_.temperature = msg_feedback->temperature;
    msg_feedback_.move = msg_feedback->move;
    msg_feedback_.current = msg_feedback->current;

#ifdef SNAKE_GUI_DEBUG_MODE
    RCLCPP_INFO(node_->get_logger(),
                "callbackTheta===>Seq:[%d]\tID[0] Pos:[%f]\tID[1] Pos:[%f]",
                msg_feedback_.seq,
                msg_feedback_.position.at(0),
                msg_feedback_.position.at(1));
#endif

    node_->params_heart_->servo_idn_val = static_cast<int>(msg_feedback_.id.size());
    if (qcp_data_feedback_.size() != node_->params_heart_->servo_idn_val) {
        qcp_time_feedback_.clear();
        qcp_data_feedback_.clear();
        qcp_data_feedback_.resize(node_->params_heart_->servo_idn_val);

        ui->qcp_feedback_pos->clearGraphs();
        for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
            ui->qcp_feedback_pos->addGraph(ui->qcp_feedback_pos->xAxis, ui->qcp_feedback_pos->yAxis);
            ui->qcp_feedback_pos->graph(i)->setPen(qcp_curve_color_.at(i % qcp_curve_color_.size()));
//            ui->qcp_feedback_pos->graph(i)->setScatterStyle(qcp_curve_style_.at(i % qcp_curve_style_.size()));
            ui->qcp_feedback_pos->graph(i)->setName(QString("joint_pos_%1").arg(i + 1));
        }
    }

    // Update time.
    if (qcp_time_feedback_.size() >= qcp_data_num_) {
        qcp_time_feedback_.removeFirst();
        for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
            qcp_data_feedback_[i].removeFirst();
        }
    }
    qcp_time_feedback_.append(calTnterval(qcp_time_start_, msg_feedback_.header));
    for (int i = 0; i < node_->params_heart_->servo_idn_val; i++) {
        qcp_data_feedback_[i].append(msg_feedback_.position[i] / M_PI * 180);
        ui->qcp_feedback_pos->graph(i)->setData(qcp_time_feedback_, qcp_data_feedback_.at(i));
    }

    ui->qcp_feedback_pos->xAxis->rescale(true);
    ui->qcp_feedback_pos->yAxis->rescale(true);

    emit sigReplotFeedback(QCustomPlot::rpQueuedReplot);
//    ui->qcp_feedback_pos->replot(QCustomPlot::rpQueuedReplot);

    qcp_mutex_feedback_.unlock();

#ifdef SNAKE_GUI_DEBUG_MODE
    qDebug() << "feedback thread qt id:" << QThread::currentThreadId();
    qDebug() << "feedback thread qcp id:" << ui->qcp_feedback_pos->thread()->currentThreadId();
#endif
}
