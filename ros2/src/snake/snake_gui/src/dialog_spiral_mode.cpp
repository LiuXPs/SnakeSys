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

#include "snake_gui/dialog_spiral_mode.h"
#include "snake_gui/ui_dialog_spiral_mode.h"

DialogSpiralMode::DialogSpiralMode(QWidget *parent) : QDialog(parent) {
    ui = std::make_shared<Ui::DialogSpiralMode>();
    ui->setupUi(this);

    // Paper color.
//    setStyleSheet("background-color: rgb(255, 255, 255);color: rgb(0, 0, 0);");

    params_heart_ = std::make_shared<snake::SnakeParamsHeart>();
    params_cpg_hopf_ = std::make_shared<snake::SnakeParamsCPGHopf>();

    // First kind of connect function.
    connect(DialogSpiralMode::ui->doubleSpinBox_spiral_radius, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_spiral_r_val = value;
            });
    connect(DialogSpiralMode::ui->doubleSpinBox_spiral_pitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_spiral_p_val = value;
            });

    // Second kind of connect function.
    connect(DialogSpiralMode::ui->buttonBox, &QDialogButtonBox::clicked,
            [this](QAbstractButton *button) {
                if (button == static_cast<QAbstractButton *>(ui->buttonBox->button(QDialogButtonBox::Ok))) {
                    setParam();
                } else if (button == static_cast<QAbstractButton *>(ui->buttonBox->button(QDialogButtonBox::Cancel))) {
                    getParam();
                } else if (button == static_cast<QAbstractButton *>(ui->buttonBox->button(QDialogButtonBox::Apply))) {
                    setParam();
                }
            });
}

DialogSpiralMode::~DialogSpiralMode() = default;

void DialogSpiralMode::setClient(std::shared_ptr<snake::SnakeGUINode> node,
                                 rclcpp::AsyncParametersClient::SharedPtr client_heart,
                                 rclcpp::AsyncParametersClient::SharedPtr client_cpg_hopf) {
    node_ = std::move(node);
    client_heart_ = std::move(client_heart);
    client_cpg_hopf_ = std::move(client_cpg_hopf);

    getParam();
    setBoxValue();
}

void DialogSpiralMode::getParam() {
    client_cpg_hopf_->get_parameters(
            {
                    params_cpg_hopf_->cpg_spiral_r_str,
                    params_cpg_hopf_->cpg_spiral_p_str,
            },
            [this](const std::shared_future<std::vector<rclcpp::Parameter>> &params) {
                params_cpg_hopf_->cpg_spiral_r_val = static_cast<double>(params.get().at(0).as_double());
                params_cpg_hopf_->cpg_spiral_p_val = static_cast<double>(params.get().at(1).as_double());

                RCLCPP_INFO(node_->get_logger(),
                            "Get spiral parameters successfully.[%d]", params.get().size());
            });
}

void DialogSpiralMode::setParam() {
    params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::SPIRAL_ROLLING_LOCOMOTION;
    params_cpg_hopf_->cpg_change_val = true;

    client_cpg_hopf_->set_parameters(
            {
                    rclcpp::Parameter(params_cpg_hopf_->cpg_spiral_r_str, params_cpg_hopf_->cpg_spiral_r_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_spiral_p_str, params_cpg_hopf_->cpg_spiral_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_motion_mode_str,
                                      static_cast<int>(params_cpg_hopf_->cpg_motion_mode_val)),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val),
            },
            [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                node_->params_cpg_hopf_->cpg_motion_mode_val = params_cpg_hopf_->cpg_motion_mode_val;
                node_->params_cpg_hopf_->cpg_change_val = params_cpg_hopf_->cpg_change_val;
                node_->params_cpg_hopf_->cpg_spiral_r_val = params_cpg_hopf_->cpg_spiral_r_val;
                node_->params_cpg_hopf_->cpg_spiral_p_val = params_cpg_hopf_->cpg_spiral_p_val;

                RCLCPP_INFO(node_->get_logger(),
                            "Set typical parameters successfully.[%d]", params.get().size());
            });
}

void DialogSpiralMode::getBoxValue() {
    params_cpg_hopf_->cpg_spiral_r_val = ui->doubleSpinBox_spiral_radius->value();
    params_cpg_hopf_->cpg_spiral_p_val = ui->doubleSpinBox_spiral_pitch->value();
}

void DialogSpiralMode::setBoxValue() {
    ui->doubleSpinBox_spiral_radius->setValue(params_cpg_hopf_->cpg_spiral_r_val);
    ui->doubleSpinBox_spiral_pitch->setValue(params_cpg_hopf_->cpg_spiral_p_val);
}
