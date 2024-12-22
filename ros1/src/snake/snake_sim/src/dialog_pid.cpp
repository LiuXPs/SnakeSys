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

#include "snake_sim/dialog_pid.h"
#include "snake_sim/ui_dialog_pid.h"

DialogPID::DialogPID(QWidget *parent) : QDialog(parent) {
    ui = std::make_shared<Ui::DialogPID>();
    ui->setupUi(this);

    this->setStyleSheet("QDialog{background-color: rgb(55, 55, 55)}");

    initNode();

    // First kind of connect function.
    connect(ui->doubleSpinBox_p, &QDoubleSpinBox::valueChanged, ui->horizontalSlider_p, &QSlider::setValue);
    connect(ui->horizontalSlider_p, &QSlider::valueChanged, ui->doubleSpinBox_p, &QDoubleSpinBox::setValue);

    connect(ui->doubleSpinBox_i, &QDoubleSpinBox::valueChanged, ui->horizontalSlider_i, &QSlider::setValue);
    connect(ui->horizontalSlider_i, &QSlider::valueChanged, ui->doubleSpinBox_i, &QDoubleSpinBox::setValue);

    connect(ui->doubleSpinBox_d, &QDoubleSpinBox::valueChanged, ui->horizontalSlider_d, &QSlider::setValue);
    connect(ui->horizontalSlider_d, &QSlider::valueChanged, ui->doubleSpinBox_d, &QDoubleSpinBox::setValue);

    connect(DialogPID::ui->doubleSpinBox_p, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                p_ = value;
            });
    connect(DialogPID::ui->doubleSpinBox_i, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                i_ = value;
            });
    connect(DialogPID::ui->doubleSpinBox_d, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                d_ = value;
            });

    // Second kind of connect function.
    connect(DialogPID::ui->buttonBox, &QDialogButtonBox::clicked,
            [this](QAbstractButton *button) {
                if (button == static_cast<QAbstractButton *>(ui->buttonBox->button(QDialogButtonBox::Ok))) {
//                    getBoxValue();
                    setParam();
                } else if (button == static_cast<QAbstractButton *>(ui->buttonBox->button(QDialogButtonBox::Cancel))) {
                    getParam();
                    setBoxValue();
                } else if (button == static_cast<QAbstractButton *>(ui->buttonBox->button(QDialogButtonBox::Apply))) {
//                    getBoxValue();
                    setParam();
                }
            });

    // Third kind of connect function.
    qthread_pid_ = std::make_shared<QThread>();
    thread_pid_ = std::make_shared<ThreadPID>();
    thread_pid_->moveToThread(qthread_pid_.get());
    connect(qthread_pid_.get(), &QThread::finished, thread_pid_.get(), &ThreadPID::deleteLater);
    connect(this, &DialogPID::sigSetPID, thread_pid_.get(), &ThreadPID::onSetPID);
    connect(thread_pid_.get(), &ThreadPID::sigSetProgress, this, &DialogPID::onSetProgress);
    qthread_pid_->start();

    ui->progressBar->setAlignment(Qt::AlignCenter);
    ui->progressBar->setTextVisible(true);
    ui->progressBar->setFixedHeight(20);
    ui->progressBar->setStyleSheet(
            "QProgressBar{color: rgb(0,0,0); border-radius: 10px; background: rgb(200, 200, 200);}"
            "QProgressBar::chunk{border-radius: 10px; background: rgb(52, 153, 216);}"
    );
}

DialogPID::~DialogPID() {
    qthread_pid_->quit();
    qthread_pid_->wait();
}

void DialogPID::initNode() {
    connect(&thread_callback_, &ThreadCallback::finished, &thread_callback_, &QObject::deleteLater);
    thread_callback_.start();
}

void DialogPID::onSetProgress(int val) {
    ui->progressBar->setValue(val);
}

void DialogPID::getParam() {
    ros::param::get(str_p_, p_);
    ros::param::get(str_i_, i_);
    ros::param::get(str_d_, d_);
}

void DialogPID::setParam() {
    emit sigSetPID(p_, i_, d_);
}

void DialogPID::getBoxValue() {
    p_ = ui->doubleSpinBox_p->value();
    i_ = ui->doubleSpinBox_i->value();
    d_ = ui->doubleSpinBox_d->value();
}

void DialogPID::setBoxValue() {
    ui->doubleSpinBox_p->setValue(p_);
    ui->doubleSpinBox_i->setValue(i_);
    ui->doubleSpinBox_d->setValue(d_);
}
