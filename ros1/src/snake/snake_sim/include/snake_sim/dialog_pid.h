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

#ifndef DIALOG_PID_H
#define DIALOG_PID_H

#include "snake_sim/thread_pid.h"
#include "snake_sim/thread_callback.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSlider>
#include <QThread>

#include <iostream>
#include <utility>

#include <ros/ros.h>

namespace Ui {
    class DialogPID;
}

class DialogPID : public QDialog {
Q_OBJECT

public:
    explicit DialogPID(QWidget *parent = nullptr);

    ~DialogPID() override;

private:
    std::shared_ptr<Ui::DialogPID> ui;
    ThreadCallback thread_callback_;

signals:

    void sigSetPID(double p, double i, double d);

public slots:

    void onSetProgress(int val);

public:
    void initNode();

    void getParam();

    void setParam();

    void getBoxValue();

    void setBoxValue();

private:
    double p_;
    double i_;
    double d_;

    const std::string str_p_ = "/gazebo_ros_control/pid_gains/snake_joint_1/p";
    const std::string str_i_ = "/gazebo_ros_control/pid_gains/snake_joint_1/i";
    const std::string str_d_ = "/gazebo_ros_control/pid_gains/snake_joint_1/d";

    std::shared_ptr<QThread> qthread_pid_;
    std::shared_ptr<ThreadPID> thread_pid_;
};

#endif //DIALOG_PID_H
