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

#include "snake_gui/dialog_param_control.h"
#include "snake_gui/ui_dialog_param_control.h"

DialogParamControl::DialogParamControl(QWidget *parent) : QDialog(parent) {
    ui = std::make_shared<Ui::DialogParamControl>();
    ui->setupUi(this);

    // Paper color.
//    setStyleSheet("background-color: rgb(255, 255, 255);color: rgb(0, 0, 0);");

    params_heart_ = std::make_shared<snake::SnakeParamsHeart>();
    params_cpg_hopf_ = std::make_shared<snake::SnakeParamsCPGHopf>();

    // First kind of connect function.
    connect(DialogParamControl::ui->spinBox_rate, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                params_heart_->heart_rate_val = value;
            });
    connect(DialogParamControl::ui->spinBox_servo_idn, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                params_heart_->servo_idn_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_link_length, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_heart_->link_length_val = value;
            });

    // Second kind of connect function.
    connect(DialogParamControl::ui->spinBox_topo_mode, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                params_cpg_hopf_->cpg_topo_mode_val = static_cast<snake::SnakeParamsCPGHopf::TopologyMode>(value);
            });
    connect(DialogParamControl::ui->spinBox_weight_mode, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                params_cpg_hopf_->cpg_weight_mode_val = static_cast<snake::SnakeParamsCPGHopf::WeightMode>(value);
            });
    connect(DialogParamControl::ui->spinBox_couple_mode, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                params_cpg_hopf_->cpg_couple_mode_val = static_cast<snake::SnakeParamsCPGHopf::CoupleMode>(value);
            });
    connect(DialogParamControl::ui->spinBox_cpg_mode, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                params_cpg_hopf_->cpg_mode_val = static_cast<snake::SnakeParamsCPGHopf::CPGMode>(value);
            });
    connect(DialogParamControl::ui->doubleSpinBox_gauss_mu, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_gauss_mu_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_gauss_sigma, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_gauss_sigma_val = value;
            });

    // Third kind of connect function.
    connect(DialogParamControl::ui->doubleSpinBox_alpha_yaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_alpha_y_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_alpha_pitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_alpha_p_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_kg_yaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_kg_y_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_kg_pitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_kg_p_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_lambda_yaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_lambda_y_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_lambda_pitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_lambda_p_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_sigma_yaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_sigma_y_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_sigma_pitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_sigma_p_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_omega_yaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_omega_y_val = value;
            });
    connect(DialogParamControl::ui->doubleSpinBox_omega_pitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value) {
                params_cpg_hopf_->cpg_omega_p_val = value;
            });

    // Fourth kind of connect function.
    connect(DialogParamControl::ui->buttonBox, &QDialogButtonBox::clicked,
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

DialogParamControl::~DialogParamControl() = default;

void DialogParamControl::setClient(std::shared_ptr<snake::SnakeGUINode> node,
                                   rclcpp::AsyncParametersClient::SharedPtr client_heart,
                                   rclcpp::AsyncParametersClient::SharedPtr client_cpg_hopf) {
    node_ = std::move(node);
    client_heart_ = std::move(client_heart);
    client_cpg_hopf_ = std::move(client_cpg_hopf);

    getParam();
    setBoxValue();
}

void DialogParamControl::getParam() {
    client_heart_->get_parameters(
            {
                    params_heart_->heart_rate_str,
                    params_heart_->servo_idn_str,
                    params_heart_->link_length_str,
            },
            [this](const std::shared_future<std::vector<rclcpp::Parameter>> &params) {
                params_heart_->heart_rate_val = static_cast<int>(params.get().at(0).as_int());
                params_heart_->servo_idn_val = static_cast<int>(params.get().at(1).as_int());
                params_heart_->link_length_val = static_cast<double >(params.get().at(2).as_double());

                RCLCPP_INFO(node_->get_logger(),
                            "Get control parameters of heart successfully.[%d]", params.get().size());
            });

    client_cpg_hopf_->get_parameters(
            {
                    params_cpg_hopf_->cpg_topo_mode_str,
                    params_cpg_hopf_->cpg_weight_mode_str,
                    params_cpg_hopf_->cpg_couple_mode_str,
                    params_cpg_hopf_->cpg_mode_str,

                    params_cpg_hopf_->cpg_gauss_mu_str,
                    params_cpg_hopf_->cpg_gauss_sigma_str,
                    params_cpg_hopf_->cpg_alpha_y_str,
                    params_cpg_hopf_->cpg_alpha_p_str,
                    params_cpg_hopf_->cpg_kg_y_str,
                    params_cpg_hopf_->cpg_kg_p_str,
                    params_cpg_hopf_->cpg_lambda_y_str,
                    params_cpg_hopf_->cpg_lambda_p_str,
                    params_cpg_hopf_->cpg_sigma_y_str,
                    params_cpg_hopf_->cpg_sigma_p_str,
                    params_cpg_hopf_->cpg_omega_y_str,
                    params_cpg_hopf_->cpg_omega_p_str,
            },
            [this](const std::shared_future<std::vector<rclcpp::Parameter>> &params) {
                params_cpg_hopf_->cpg_topo_mode_val =
                        static_cast<snake::SnakeParamsCPGHopf::TopologyMode>(params.get().at(0).as_int());
                params_cpg_hopf_->cpg_weight_mode_val =
                        static_cast<snake::SnakeParamsCPGHopf::WeightMode>(params.get().at(1).as_int());
                params_cpg_hopf_->cpg_couple_mode_val =
                        static_cast<snake::SnakeParamsCPGHopf::CoupleMode>(params.get().at(2).as_int());
                params_cpg_hopf_->cpg_mode_val =
                        static_cast<snake::SnakeParamsCPGHopf::CPGMode>(params.get().at(3).as_int());

                params_cpg_hopf_->cpg_gauss_mu_val = static_cast<double>(params.get().at(4).as_double());
                params_cpg_hopf_->cpg_gauss_sigma_val = static_cast<double>(params.get().at(5).as_double());
                params_cpg_hopf_->cpg_alpha_y_val = static_cast<double>(params.get().at(6).as_double());
                params_cpg_hopf_->cpg_alpha_p_val = static_cast<double>(params.get().at(7).as_double());
                params_cpg_hopf_->cpg_kg_y_val = static_cast<double>(params.get().at(8).as_double());
                params_cpg_hopf_->cpg_kg_p_val = static_cast<double>(params.get().at(9).as_double());
                params_cpg_hopf_->cpg_lambda_y_val = static_cast<double>(params.get().at(10).as_double());
                params_cpg_hopf_->cpg_lambda_p_val = static_cast<double>(params.get().at(11).as_double());
                params_cpg_hopf_->cpg_sigma_y_val = static_cast<double>(params.get().at(12).as_double());
                params_cpg_hopf_->cpg_sigma_p_val = static_cast<double>(params.get().at(13).as_double());
                params_cpg_hopf_->cpg_omega_y_val = static_cast<double>(params.get().at(14).as_double());
                params_cpg_hopf_->cpg_omega_p_val = static_cast<double>(params.get().at(15).as_double());

                RCLCPP_INFO(node_->get_logger(),
                            "Get control parameters of CPG Hopf successfully.[%d]", params.get().size());
            });
}

void DialogParamControl::setParam() {
    // TODO.
    // Make the yaw and pitch oscillator have the same parameters.
    params_cpg_hopf_->cpg_alpha_p_val = params_cpg_hopf_->cpg_alpha_y_val;
    params_cpg_hopf_->cpg_kg_p_val = params_cpg_hopf_->cpg_kg_y_val;
    params_cpg_hopf_->cpg_lambda_p_val = params_cpg_hopf_->cpg_lambda_y_val;
    params_cpg_hopf_->cpg_sigma_p_val = params_cpg_hopf_->cpg_sigma_y_val;
    params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;

    // This step is necessary when the parameters change.
    params_cpg_hopf_->cpg_change_val = true;

    client_heart_->set_parameters(
            {
                    rclcpp::Parameter(params_heart_->heart_rate_str, params_heart_->heart_rate_val),
                    rclcpp::Parameter(params_heart_->servo_idn_str, params_heart_->servo_idn_val),
                    rclcpp::Parameter(params_heart_->link_length_str, params_heart_->link_length_val),
            },
            [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                node_->params_heart_->heart_rate_val = params_heart_->heart_rate_val;
                node_->params_heart_->servo_idn_val = params_heart_->servo_idn_val;
                node_->params_heart_->link_length_val = params_heart_->link_length_val;

                RCLCPP_INFO(node_->get_logger(),
                            "Set control parameters of heart successfully.[%d]", params.get().size());
            });

    client_cpg_hopf_->set_parameters(
            {
                    rclcpp::Parameter(params_cpg_hopf_->cpg_topo_mode_str,
                                      static_cast<int>(params_cpg_hopf_->cpg_topo_mode_val)),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_weight_mode_str,
                                      static_cast<int>(params_cpg_hopf_->cpg_weight_mode_val)),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_couple_mode_str,
                                      static_cast<int>(params_cpg_hopf_->cpg_couple_mode_val)),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_mode_str,
                                      static_cast<int>(params_cpg_hopf_->cpg_mode_val)),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_gauss_mu_str, params_cpg_hopf_->cpg_gauss_mu_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_gauss_sigma_str, params_cpg_hopf_->cpg_gauss_sigma_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_alpha_y_str, params_cpg_hopf_->cpg_alpha_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_alpha_p_str, params_cpg_hopf_->cpg_alpha_p_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_kg_y_str, params_cpg_hopf_->cpg_kg_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_kg_p_str, params_cpg_hopf_->cpg_kg_p_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_lambda_y_str, params_cpg_hopf_->cpg_lambda_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_lambda_p_str, params_cpg_hopf_->cpg_lambda_p_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_sigma_y_str, params_cpg_hopf_->cpg_sigma_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_sigma_p_str, params_cpg_hopf_->cpg_sigma_p_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_omega_y_str, params_cpg_hopf_->cpg_omega_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_omega_p_str, params_cpg_hopf_->cpg_omega_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val),
            },
            [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                node_->params_cpg_hopf_->cpg_topo_mode_val = params_cpg_hopf_->cpg_topo_mode_val;
                node_->params_cpg_hopf_->cpg_weight_mode_val = params_cpg_hopf_->cpg_weight_mode_val;
                node_->params_cpg_hopf_->cpg_couple_mode_val = params_cpg_hopf_->cpg_couple_mode_val;
                node_->params_cpg_hopf_->cpg_mode_val = params_cpg_hopf_->cpg_mode_val;

                node_->params_cpg_hopf_->cpg_gauss_mu_val = params_cpg_hopf_->cpg_gauss_mu_val;
                node_->params_cpg_hopf_->cpg_gauss_sigma_val = params_cpg_hopf_->cpg_gauss_sigma_val;
                node_->params_cpg_hopf_->cpg_alpha_y_val = params_cpg_hopf_->cpg_alpha_y_val;
                node_->params_cpg_hopf_->cpg_alpha_p_val = params_cpg_hopf_->cpg_alpha_p_val;
                node_->params_cpg_hopf_->cpg_kg_y_val = params_cpg_hopf_->cpg_kg_y_val;
                node_->params_cpg_hopf_->cpg_kg_p_val = params_cpg_hopf_->cpg_kg_p_val;
                node_->params_cpg_hopf_->cpg_lambda_y_val = params_cpg_hopf_->cpg_lambda_y_val;
                node_->params_cpg_hopf_->cpg_lambda_p_val = params_cpg_hopf_->cpg_lambda_p_val;
                node_->params_cpg_hopf_->cpg_sigma_y_val = params_cpg_hopf_->cpg_sigma_y_val;
                node_->params_cpg_hopf_->cpg_sigma_p_val = params_cpg_hopf_->cpg_sigma_p_val;
                node_->params_cpg_hopf_->cpg_omega_y_val = params_cpg_hopf_->cpg_omega_y_val;
                node_->params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_p_val;

                node_->params_cpg_hopf_->cpg_change_val = params_cpg_hopf_->cpg_change_val;

                RCLCPP_INFO(node_->get_logger(),
                            "Set control parameters of CPG Hopf successfully.[%d]", params.get().size());
            }
    );
}

void DialogParamControl::getBoxValue() {
    params_heart_->heart_rate_val = ui->spinBox_rate->value();
    params_heart_->servo_idn_val = ui->spinBox_servo_idn->value();
    params_heart_->link_length_val = ui->doubleSpinBox_link_length->value();

    params_cpg_hopf_->cpg_topo_mode_val =
            static_cast<snake::SnakeParamsCPGHopf::TopologyMode>(ui->spinBox_topo_mode->value());
    params_cpg_hopf_->cpg_weight_mode_val =
            static_cast<snake::SnakeParamsCPGHopf::WeightMode>(ui->spinBox_weight_mode->value());
    params_cpg_hopf_->cpg_couple_mode_val =
            static_cast<snake::SnakeParamsCPGHopf::CoupleMode>(ui->spinBox_couple_mode->value());
    params_cpg_hopf_->cpg_mode_val =
            static_cast<snake::SnakeParamsCPGHopf::CPGMode>(ui->spinBox_cpg_mode->value());

    params_cpg_hopf_->cpg_gauss_mu_val = ui->doubleSpinBox_gauss_mu->value();
    params_cpg_hopf_->cpg_gauss_sigma_val = ui->doubleSpinBox_gauss_sigma->value();
    params_cpg_hopf_->cpg_alpha_y_val = ui->doubleSpinBox_alpha_yaw->value();
    params_cpg_hopf_->cpg_alpha_p_val = ui->doubleSpinBox_alpha_pitch->value();
    params_cpg_hopf_->cpg_kg_y_val = ui->doubleSpinBox_kg_yaw->value();
    params_cpg_hopf_->cpg_kg_p_val = ui->doubleSpinBox_kg_pitch->value();
    params_cpg_hopf_->cpg_lambda_y_val = ui->doubleSpinBox_lambda_yaw->value();
    params_cpg_hopf_->cpg_lambda_p_val = ui->doubleSpinBox_lambda_pitch->value();
    params_cpg_hopf_->cpg_sigma_y_val = ui->doubleSpinBox_sigma_yaw->value();
    params_cpg_hopf_->cpg_sigma_p_val = ui->doubleSpinBox_sigma_pitch->value();
    params_cpg_hopf_->cpg_omega_y_val = ui->doubleSpinBox_omega_yaw->value();
    params_cpg_hopf_->cpg_omega_p_val = ui->doubleSpinBox_omega_pitch->value();

    // TODO.
    // Make the yaw and pitch oscillator have the same parameters.
    params_cpg_hopf_->cpg_alpha_p_val = params_cpg_hopf_->cpg_alpha_y_val;
    params_cpg_hopf_->cpg_kg_p_val = params_cpg_hopf_->cpg_kg_y_val;
    params_cpg_hopf_->cpg_lambda_p_val = params_cpg_hopf_->cpg_lambda_y_val;
    params_cpg_hopf_->cpg_sigma_p_val = params_cpg_hopf_->cpg_sigma_y_val;
    params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;
}

void DialogParamControl::setBoxValue() {
    // TODO.
    // Make the yaw and pitch oscillator have the same parameters.
    params_cpg_hopf_->cpg_alpha_p_val = params_cpg_hopf_->cpg_alpha_y_val;
    params_cpg_hopf_->cpg_kg_p_val = params_cpg_hopf_->cpg_kg_y_val;
    params_cpg_hopf_->cpg_lambda_p_val = params_cpg_hopf_->cpg_lambda_y_val;
    params_cpg_hopf_->cpg_sigma_p_val = params_cpg_hopf_->cpg_sigma_y_val;
    params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;

    ui->spinBox_rate->setValue(params_heart_->heart_rate_val);
    ui->spinBox_servo_idn->setValue(params_heart_->servo_idn_val);
    ui->doubleSpinBox_link_length->setValue(params_heart_->link_length_val);

    ui->spinBox_topo_mode->setValue(static_cast<int>(params_cpg_hopf_->cpg_topo_mode_val));
    ui->spinBox_weight_mode->setValue(static_cast<int>(params_cpg_hopf_->cpg_weight_mode_val));
    ui->spinBox_couple_mode->setValue(static_cast<int>(params_cpg_hopf_->cpg_couple_mode_val));
    ui->spinBox_cpg_mode->setValue(static_cast<int>(params_cpg_hopf_->cpg_mode_val));

    ui->doubleSpinBox_gauss_mu->setValue(params_cpg_hopf_->cpg_gauss_mu_val);
    ui->doubleSpinBox_gauss_sigma->setValue(params_cpg_hopf_->cpg_gauss_sigma_val);
    ui->doubleSpinBox_alpha_yaw->setValue(params_cpg_hopf_->cpg_alpha_y_val);
    ui->doubleSpinBox_alpha_pitch->setValue(params_cpg_hopf_->cpg_alpha_p_val);
    ui->doubleSpinBox_kg_yaw->setValue(params_cpg_hopf_->cpg_kg_y_val);
    ui->doubleSpinBox_kg_pitch->setValue(params_cpg_hopf_->cpg_kg_p_val);
    ui->doubleSpinBox_lambda_yaw->setValue(params_cpg_hopf_->cpg_lambda_y_val);
    ui->doubleSpinBox_lambda_pitch->setValue(params_cpg_hopf_->cpg_lambda_p_val);
    ui->doubleSpinBox_sigma_yaw->setValue(params_cpg_hopf_->cpg_sigma_y_val);
    ui->doubleSpinBox_sigma_pitch->setValue(params_cpg_hopf_->cpg_sigma_p_val);
    ui->doubleSpinBox_omega_yaw->setValue(params_cpg_hopf_->cpg_omega_y_val);
    ui->doubleSpinBox_omega_pitch->setValue(params_cpg_hopf_->cpg_omega_p_val);
}
