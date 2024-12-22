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

#include "snake_sim/thread_pid.h"

ThreadPID::ThreadPID(QObject *parent) : QObject(parent) {
    count_ = 0;
    emit sigSetProgress(count_);

    command_p_.resize(joint_n_);
    command_i_.resize(joint_n_);
    command_d_.resize(joint_n_);

    thread_pid_.resize(joint_n_);
}

ThreadPID::~ThreadPID() = default;

void ThreadPID::onSetPID(double pid_p, double pid_i, double pid_d) {

//    qDebug() << " pid root thread id:" << QThread::currentThreadId();

    count_ = 0;
    emit sigSetProgress(count_);

    for (int i = 0; i < joint_n_; i++) {
        command_p_.at(i) = comm_ + std::to_string(i + 1) + " p " + std::to_string(pid_p);
        command_i_.at(i) = comm_ + std::to_string(i + 1) + " i " + std::to_string(pid_i);
        command_d_.at(i) = comm_ + std::to_string(i + 1) + " d " + std::to_string(pid_d);

        thread_pid_.at(i) = std::make_shared<std::thread>(&ThreadPID::setPIDFun, this,
                                                          std::ref(command_p_.at(i)),
                                                          std::ref(command_i_.at(i)),
                                                          std::ref(command_d_.at(i)));
    }

    for (const auto &thread: thread_pid_) {
        thread->join();
    }
}

void ThreadPID::setPIDFun(std::string comm_p, std::string comm_i, std::string comm_d) {

//    qDebug() << " pid child thread id:" << QThread::currentThreadId();

    FILE *fp_p = popen(comm_p.c_str(), "w");
    FILE *fp_i = popen(comm_i.c_str(), "w");
    FILE *fp_d = popen(comm_d.c_str(), "w");

    pclose(fp_p);
    pclose(fp_i);
    pclose(fp_d);

    {
        std::unique_lock<std::mutex> ul(mutex_pid_);
        count_ += 1;
        emit sigSetProgress(count_);
    }
}
