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

#include "snake_gui/snake_gui_node.h"

namespace snake {
    SnakeGUINode::SnakeGUINode(std::string name) : rclcpp::Node(name) {
        params_heart_ = std::make_shared<snake::SnakeParamsHeart>();
        params_cpg_hopf_ = std::make_shared<snake::SnakeParamsCPGHopf>();

        client_heart_ = std::make_shared<rclcpp::AsyncParametersClient>(this, node_heart_str_);
        client_cpg_hopf_ = std::make_shared<rclcpp::AsyncParametersClient>(this, node_cpg_hopf_str_);

        while (!client_heart_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                exit(1);
            }
            RCLCPP_ERROR(get_logger(), "Service not available, waiting again ...");
        }
        while (!client_cpg_hopf_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                exit(1);
            }
            RCLCPP_ERROR(get_logger(), "Service not available, waiting again ...");
        }

        results_get_heart_ = client_heart_->get_parameters(
                {
                        params_heart_->heart_rate_str,
                        params_heart_->servo_idn_str,
                        params_heart_->link_length_str,
                });
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), results_get_heart_) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            params_heart_->heart_rate_val = static_cast<int>(results_get_heart_.get().at(0).as_int());
            params_heart_->servo_idn_val = static_cast<int>(results_get_heart_.get().at(1).as_int());
            params_heart_->link_length_val = static_cast<double>(results_get_heart_.get().at(2).as_double());
            RCLCPP_INFO(get_logger(), "Get all parameters of heart successfully.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get parameters of heart.");
            exit(1);
        }

        results_get_cpg_hopf_ = client_cpg_hopf_->get_parameters(
                {
                        params_cpg_hopf_->cpg_topo_mode_str,
                        params_cpg_hopf_->cpg_weight_mode_str,
                        params_cpg_hopf_->cpg_motion_mode_str,
                        params_cpg_hopf_->cpg_couple_mode_str,
                        params_cpg_hopf_->cpg_mode_str,

                        params_cpg_hopf_->cpg_alpha_y_str,
                        params_cpg_hopf_->cpg_alpha_p_str,
                        params_cpg_hopf_->cpg_kg_y_str,
                        params_cpg_hopf_->cpg_kg_p_str,
                        params_cpg_hopf_->cpg_lambda_y_str,
                        params_cpg_hopf_->cpg_lambda_p_str,
                        params_cpg_hopf_->cpg_sigma_y_str,
                        params_cpg_hopf_->cpg_sigma_p_str,
                        params_cpg_hopf_->cpg_uvc_y_str,
                        params_cpg_hopf_->cpg_uvc_p_str,
                        params_cpg_hopf_->cpg_rho_y_str,
                        params_cpg_hopf_->cpg_rho_p_str,
                        params_cpg_hopf_->cpg_omega_y_str,
                        params_cpg_hopf_->cpg_omega_p_str,
                        params_cpg_hopf_->cpg_phi_y_str,
                        params_cpg_hopf_->cpg_phi_p_str,
                        params_cpg_hopf_->cpg_phi_yp_str,

                        params_cpg_hopf_->cpg_gauss_mu_str,
                        params_cpg_hopf_->cpg_gauss_sigma_str,
                        params_cpg_hopf_->cpg_wave_kn_str,
                        params_cpg_hopf_->cpg_wave_ay_str,
                        params_cpg_hopf_->cpg_wave_ap_str,
                        params_cpg_hopf_->cpg_arc_r_str,
                        params_cpg_hopf_->cpg_spiral_r_str,
                        params_cpg_hopf_->cpg_spiral_p_str,

                        params_cpg_hopf_->cpg_start_str,
                        params_cpg_hopf_->cpg_change_str,
                        params_cpg_hopf_->cpg_torque_str,
                });
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), results_get_cpg_hopf_) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            params_cpg_hopf_->cpg_topo_mode_val =
                    static_cast<snake::SnakeParamsCPGHopf::TopologyMode>(results_get_cpg_hopf_.get().at(0).as_int());
            params_cpg_hopf_->cpg_weight_mode_val =
                    static_cast<snake::SnakeParamsCPGHopf::WeightMode>(results_get_cpg_hopf_.get().at(1).as_int());
            params_cpg_hopf_->cpg_motion_mode_val =
                    static_cast<snake::SnakeParamsCPGHopf::MotionMode>(results_get_cpg_hopf_.get().at(2).as_int());
            params_cpg_hopf_->cpg_couple_mode_val =
                    static_cast<snake::SnakeParamsCPGHopf::CoupleMode>(results_get_cpg_hopf_.get().at(3).as_int());
            params_cpg_hopf_->cpg_mode_val =
                    static_cast<snake::SnakeParamsCPGHopf::CPGMode>(results_get_cpg_hopf_.get().at(4).as_int());

            params_cpg_hopf_->cpg_alpha_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(5).as_double());
            params_cpg_hopf_->cpg_alpha_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(6).as_double());
            params_cpg_hopf_->cpg_kg_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(7).as_double());
            params_cpg_hopf_->cpg_kg_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(8).as_double());
            params_cpg_hopf_->cpg_lambda_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(9).as_double());
            params_cpg_hopf_->cpg_lambda_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(10).as_double());
            params_cpg_hopf_->cpg_sigma_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(11).as_double());
            params_cpg_hopf_->cpg_sigma_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(12).as_double());
            params_cpg_hopf_->cpg_uvc_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(13).as_double());
            params_cpg_hopf_->cpg_uvc_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(14).as_double());
            params_cpg_hopf_->cpg_rho_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(15).as_double());
            params_cpg_hopf_->cpg_rho_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(16).as_double());
            params_cpg_hopf_->cpg_omega_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(17).as_double());
            params_cpg_hopf_->cpg_omega_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(18).as_double());
            params_cpg_hopf_->cpg_phi_y_val = static_cast<double>(results_get_cpg_hopf_.get().at(19).as_double());
            params_cpg_hopf_->cpg_phi_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(20).as_double());
            params_cpg_hopf_->cpg_phi_yp_val = static_cast<double>(results_get_cpg_hopf_.get().at(21).as_double());

            params_cpg_hopf_->cpg_gauss_mu_val = static_cast<double>(results_get_cpg_hopf_.get().at(22).as_double());
            params_cpg_hopf_->cpg_gauss_sigma_val = static_cast<double>(results_get_cpg_hopf_.get().at(23).as_double());
            params_cpg_hopf_->cpg_wave_kn_val = static_cast<double>(results_get_cpg_hopf_.get().at(24).as_double());
            params_cpg_hopf_->cpg_wave_ay_val = static_cast<double>(results_get_cpg_hopf_.get().at(25).as_double());
            params_cpg_hopf_->cpg_wave_ap_val = static_cast<double>(results_get_cpg_hopf_.get().at(26).as_double());
            params_cpg_hopf_->cpg_arc_r_val = static_cast<double>(results_get_cpg_hopf_.get().at(27).as_double());
            params_cpg_hopf_->cpg_spiral_r_val = static_cast<double>(results_get_cpg_hopf_.get().at(28).as_double());
            params_cpg_hopf_->cpg_spiral_p_val = static_cast<double>(results_get_cpg_hopf_.get().at(29).as_double());

            params_cpg_hopf_->cpg_start_val = static_cast<bool>(results_get_cpg_hopf_.get().at(30).as_bool());
            params_cpg_hopf_->cpg_change_val = static_cast<bool>(results_get_cpg_hopf_.get().at(31).as_bool());
            params_cpg_hopf_->cpg_torque_val = static_cast<bool>(results_get_cpg_hopf_.get().at(32).as_bool());
            RCLCPP_INFO(get_logger(), "Get all parameters of CPG Hopf successfully.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to get parameters of CPG Hopf.");
            exit(1);
        }
    }

    SnakeGUINode::~SnakeGUINode() = default;
}
