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

#ifndef THREAD_PID_H
#define THREAD_PID_H

#include <QObject>
#include <QMutexLocker>
#include <QThread>
#include <QDebug>

#include <iostream>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <vector>

class ThreadPID : public QObject {
Q_OBJECT

public:
    explicit ThreadPID(QObject *parent = nullptr);

    ~ThreadPID() override;

signals:

    void sigSetProgress(int val);

public slots:

    void onSetPID(double pid_p, double pid_i, double pid_d);

public:
    void setPIDFun(std::string comm_p, std::string comm_i, std::string comm_d);

private:
    int count_;
    const int joint_n_ = 28;
    const std::string comm_ = "rosrun dynamic_reconfigure dynparam set /gazebo_ros_control/pid_gains/snake_joint_";

    std::vector<std::string> command_p_;
    std::vector<std::string> command_i_;
    std::vector<std::string> command_d_;
    std::vector<std::shared_ptr<std::thread>> thread_pid_;
    std::mutex mutex_pid_;
};

#endif //THREAD_PID_H
