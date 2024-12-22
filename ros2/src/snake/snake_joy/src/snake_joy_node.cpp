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

#include "snake_joy/snake_joy_node.h"

SnakeJoyNode::SnakeJoyNode(std::string name) : rclcpp::Node(name) {
    params_cpg_hopf_ = std::make_shared<snake::SnakeParamsCPGHopf>();

    params_.clear();
    params_.emplace_back(params_cpg_hopf_->cpg_motion_mode_str, params_cpg_hopf_->cpg_motion_mode_val);
    params_.emplace_back(params_cpg_hopf_->cpg_omega_y_str, params_cpg_hopf_->cpg_omega_y_val);
    params_.emplace_back(params_cpg_hopf_->cpg_omega_p_str, params_cpg_hopf_->cpg_omega_p_val);
    params_.emplace_back(params_cpg_hopf_->cpg_uvc_y_str, params_cpg_hopf_->cpg_uvc_y_val);
    params_.emplace_back(params_cpg_hopf_->cpg_uvc_p_str, params_cpg_hopf_->cpg_uvc_p_val);
    params_.emplace_back(params_cpg_hopf_->cpg_wave_kn_str, params_cpg_hopf_->cpg_wave_kn_val);
    params_.emplace_back(params_cpg_hopf_->cpg_wave_ay_str, params_cpg_hopf_->cpg_wave_ay_val);
    params_.emplace_back(params_cpg_hopf_->cpg_wave_ap_str, params_cpg_hopf_->cpg_wave_ap_val);
    params_.emplace_back(params_cpg_hopf_->cpg_arc_r_str, params_cpg_hopf_->cpg_arc_r_val);
    params_.emplace_back(params_cpg_hopf_->cpg_spiral_r_str, params_cpg_hopf_->cpg_spiral_r_val);
    params_.emplace_back(params_cpg_hopf_->cpg_spiral_p_str, params_cpg_hopf_->cpg_spiral_p_val);
    params_.emplace_back(params_cpg_hopf_->cpg_start_str, params_cpg_hopf_->cpg_start_val);
    params_.emplace_back(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val);
    params_.emplace_back(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val);

    client_param_ = std::make_shared<rclcpp::AsyncParametersClient>(this, node_cpg_hopf_str_);
    while (!client_param_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        RCLCPP_ERROR(get_logger(), "Service not available, waiting again ...");
    }
    get_params_value_ = client_param_->get_parameters(
            {
                    params_.at(0).get_name(),
                    params_.at(1).get_name(),
                    params_.at(2).get_name(),
                    params_.at(3).get_name(),
                    params_.at(4).get_name(),
                    params_.at(5).get_name(),
                    params_.at(6).get_name(),
                    params_.at(7).get_name(),
                    params_.at(8).get_name(),
                    params_.at(9).get_name(),
                    params_.at(10).get_name(),
                    params_.at(11).get_name(),
                    params_.at(12).get_name(),
                    params_.at(13).get_name(),
            });
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_params_value_) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        params_cpg_hopf_->cpg_motion_mode_val =
                static_cast<snake::SnakeParamsCPGHopf::MotionMode>(get_params_value_.get().at(0).as_int());
        params_cpg_hopf_->cpg_omega_y_val = static_cast<double>(get_params_value_.get().at(1).as_double());
        params_cpg_hopf_->cpg_omega_p_val = static_cast<double>(get_params_value_.get().at(2).as_double());
        params_cpg_hopf_->cpg_uvc_y_val = static_cast<double>(get_params_value_.get().at(3).as_double());
        params_cpg_hopf_->cpg_uvc_p_val = static_cast<double>(get_params_value_.get().at(4).as_double());
        params_cpg_hopf_->cpg_wave_kn_val = static_cast<double>(get_params_value_.get().at(5).as_double());
        params_cpg_hopf_->cpg_wave_ay_val = static_cast<double>(get_params_value_.get().at(6).as_double());
        params_cpg_hopf_->cpg_wave_ap_val = static_cast<double>(get_params_value_.get().at(7).as_double());
        params_cpg_hopf_->cpg_arc_r_val = static_cast<double>(get_params_value_.get().at(8).as_double());
        params_cpg_hopf_->cpg_spiral_r_val = static_cast<double>(get_params_value_.get().at(9).as_double());
        params_cpg_hopf_->cpg_spiral_p_val = static_cast<double>(get_params_value_.get().at(10).as_double());
        params_cpg_hopf_->cpg_start_val = static_cast<bool>(get_params_value_.get().at(11).as_bool());
        params_cpg_hopf_->cpg_change_val = static_cast<bool>(get_params_value_.get().at(12).as_bool());
        params_cpg_hopf_->cpg_torque_val = static_cast<bool>(get_params_value_.get().at(13).as_bool());

        RCLCPP_INFO(get_logger(), "Get parameters successfully.");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get the parameter value.");
        exit(1);
    }

    auto bind_callback = std::bind(&SnakeJoyNode::cbJoy, this, std::placeholders::_1);
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(sub_joy_str_,
                                                                queue_size_,
                                                                bind_callback);

    getParam();
}

SnakeJoyNode::~SnakeJoyNode() {

}

void SnakeJoyNode::getParam() {
    client_param_->get_parameters(
            {
                    params_.at(0).get_name(),
                    params_.at(1).get_name(),
                    params_.at(2).get_name(),
                    params_.at(3).get_name(),
                    params_.at(4).get_name(),
                    params_.at(5).get_name(),
                    params_.at(6).get_name(),
                    params_.at(7).get_name(),
                    params_.at(8).get_name(),
                    params_.at(9).get_name(),
                    params_.at(10).get_name(),
                    params_.at(11).get_name(),
                    params_.at(12).get_name(),
                    params_.at(13).get_name(),
            },
            [this](const std::shared_future<std::vector<rclcpp::Parameter>> &params) {
                params_cpg_hopf_->cpg_motion_mode_val =
                        static_cast<snake::SnakeParamsCPGHopf::MotionMode>(params.get().at(0).as_int());
                params_cpg_hopf_->cpg_omega_y_val = static_cast<double>(params.get().at(1).as_double());
                params_cpg_hopf_->cpg_omega_p_val = static_cast<double>(params.get().at(2).as_double());
                params_cpg_hopf_->cpg_uvc_y_val = static_cast<double>(params.get().at(3).as_double());
                params_cpg_hopf_->cpg_uvc_p_val = static_cast<double>(params.get().at(4).as_double());
                params_cpg_hopf_->cpg_wave_kn_val = static_cast<double>(params.get().at(5).as_double());
                params_cpg_hopf_->cpg_wave_ay_val = static_cast<double>(params.get().at(6).as_double());
                params_cpg_hopf_->cpg_wave_ap_val = static_cast<double>(params.get().at(7).as_double());
                params_cpg_hopf_->cpg_arc_r_val = static_cast<double>(params.get().at(8).as_double());
                params_cpg_hopf_->cpg_spiral_r_val = static_cast<double>(params.get().at(9).as_double());
                params_cpg_hopf_->cpg_spiral_p_val = static_cast<double>(params.get().at(10).as_double());
                params_cpg_hopf_->cpg_start_val = static_cast<bool>(params.get().at(11).as_bool());
                params_cpg_hopf_->cpg_change_val = static_cast<bool>(params.get().at(12).as_bool());
                params_cpg_hopf_->cpg_torque_val = static_cast<bool>(params.get().at(13).as_bool());

                RCLCPP_INFO(get_logger(), "Get parameters successfully.");
            });
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_params_value_) ==
        rclcpp::FutureReturnCode::SUCCESS) {

    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get the parameter value.");
        exit(1);
    }
}

void SnakeJoyNode::setParam() {
    params_cpg_hopf_->cpg_change_val = true;

    params_.at(0) = rclcpp::Parameter(
            params_cpg_hopf_->cpg_motion_mode_str, static_cast<int>(params_cpg_hopf_->cpg_motion_mode_val));
    params_.at(1) = rclcpp::Parameter(params_cpg_hopf_->cpg_omega_y_str, params_cpg_hopf_->cpg_omega_y_val);
    params_.at(2) = rclcpp::Parameter(params_cpg_hopf_->cpg_omega_p_str, params_cpg_hopf_->cpg_omega_p_val);
    params_.at(3) = rclcpp::Parameter(params_cpg_hopf_->cpg_uvc_y_str, params_cpg_hopf_->cpg_uvc_y_val);
    params_.at(4) = rclcpp::Parameter(params_cpg_hopf_->cpg_uvc_p_str, params_cpg_hopf_->cpg_uvc_p_val);
    params_.at(5) = rclcpp::Parameter(params_cpg_hopf_->cpg_wave_kn_str, params_cpg_hopf_->cpg_wave_kn_val);
    params_.at(6) = rclcpp::Parameter(params_cpg_hopf_->cpg_wave_ay_str, params_cpg_hopf_->cpg_wave_ay_val);
    params_.at(7) = rclcpp::Parameter(params_cpg_hopf_->cpg_wave_ap_str, params_cpg_hopf_->cpg_wave_ap_val);
    params_.at(8) = rclcpp::Parameter(params_cpg_hopf_->cpg_arc_r_str, params_cpg_hopf_->cpg_arc_r_val);
    params_.at(9) = rclcpp::Parameter(params_cpg_hopf_->cpg_spiral_r_str, params_cpg_hopf_->cpg_spiral_r_val);
    params_.at(10) = rclcpp::Parameter(params_cpg_hopf_->cpg_spiral_p_str, params_cpg_hopf_->cpg_spiral_p_val);
    params_.at(11) = rclcpp::Parameter(params_cpg_hopf_->cpg_start_str, params_cpg_hopf_->cpg_start_val);
    params_.at(12) = rclcpp::Parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val);
    params_.at(13) = rclcpp::Parameter(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val);

    set_params_value_ = client_param_->set_parameters(
            params_,
            [this](const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> &params) {
                RCLCPP_INFO(get_logger(), "Set parameters successfully. [%d]", params.get().size());
            });
}

void SnakeJoyNode::cbJoy(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    msg_joy_.header = joy_msg->header;
    msg_joy_.axes = joy_msg->axes;
    msg_joy_.buttons = joy_msg->buttons;

    // Start and stop.
    if (msg_joy_.buttons.at(9)) {
        if (params_cpg_hopf_->cpg_start_val) {
            params_cpg_hopf_->cpg_start_val = false;
            RCLCPP_INFO(get_logger(), "stop");
        } else {
            params_cpg_hopf_->cpg_start_val = true;
            RCLCPP_INFO(get_logger(), "start");
        }
        setParam();
    }

    // Motion mode.
    if (msg_joy_.buttons.at(0)) {
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::CUSTOM_MODE_LOCOMOTION;
        RCLCPP_INFO(get_logger(), "sport mode: %d", params_cpg_hopf_->cpg_motion_mode_val);
        setParam();
    } else if (msg_joy_.buttons.at(1)) {
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::SIDE_WINDING_LOCOMOTION;
        RCLCPP_INFO(get_logger(), "sport mode: %d", params_cpg_hopf_->cpg_motion_mode_val);
        setParam();
    } else if (msg_joy_.buttons.at(2)) {
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::ARC_ROLLING_LOCOMOTION;
        RCLCPP_INFO(get_logger(), "sport mode: %d", params_cpg_hopf_->cpg_motion_mode_val);
        setParam();
    } else if (msg_joy_.buttons.at(3)) {
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::SPIRAL_ROLLING_LOCOMOTION;
        RCLCPP_INFO(get_logger(), "sport mode: %d", params_cpg_hopf_->cpg_motion_mode_val);
        setParam();
    }

    // Acceleration and deceleration.
    if (msg_joy_.buttons.at(6)) {
        if (params_cpg_hopf_->cpg_omega_y_val >= 0) {
            params_cpg_hopf_->cpg_omega_y_val += 0.1;
            if (params_cpg_hopf_->cpg_omega_y_val > 0.4) {
                params_cpg_hopf_->cpg_omega_y_val = 0.4;
            }
        } else {
            params_cpg_hopf_->cpg_omega_y_val -= 0.1;
            if (params_cpg_hopf_->cpg_omega_y_val < -0.4) {
                params_cpg_hopf_->cpg_omega_y_val = -0.4;
            }
        }
        params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;
        RCLCPP_INFO(get_logger(), "acceleration: [%f]", params_cpg_hopf_->cpg_omega_y_val);
        setParam();
    } else if (msg_joy_.buttons.at(7)) {
        if (params_cpg_hopf_->cpg_omega_y_val >= 0) {
            params_cpg_hopf_->cpg_omega_y_val -= 0.1;
            if (params_cpg_hopf_->cpg_omega_y_val < 0.0) {
                params_cpg_hopf_->cpg_omega_y_val = 0.0;
            }
        } else {
            params_cpg_hopf_->cpg_omega_y_val += 0.1;
            if (params_cpg_hopf_->cpg_omega_y_val > -0.0) {
                params_cpg_hopf_->cpg_omega_y_val = -0.0;
            }
        }
        params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;
        RCLCPP_INFO(get_logger(), "deceleration: [%f]", params_cpg_hopf_->cpg_omega_y_val);
        setParam();
    }

    // Forward and backward.
    if (msg_joy_.axes.at(1) > 0) {
        if (params_cpg_hopf_->cpg_omega_y_val <= 0) {
            params_cpg_hopf_->cpg_omega_y_val = -1 * params_cpg_hopf_->cpg_omega_y_val;
        }
        params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;
        RCLCPP_INFO(get_logger(), "forward");
        setParam();
    } else if (msg_joy_.axes.at(1) < 0) {
        if (params_cpg_hopf_->cpg_omega_y_val >= 0) {
            params_cpg_hopf_->cpg_omega_y_val = -1 * params_cpg_hopf_->cpg_omega_y_val;
        }
        params_cpg_hopf_->cpg_omega_p_val = params_cpg_hopf_->cpg_omega_y_val;
        RCLCPP_INFO(get_logger(), "backward");
        setParam();
    }

    // Control param.
    if (params_cpg_hopf_->cpg_motion_mode_val == snake::SnakeParamsCPGHopf::MotionMode::CUSTOM_MODE_LOCOMOTION) {
        // TODO.
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::CUSTOM_MODE_LOCOMOTION;
    } else if (params_cpg_hopf_->cpg_motion_mode_val ==
               snake::SnakeParamsCPGHopf::MotionMode::SIDE_WINDING_LOCOMOTION) {
        // Left and right.
        if (msg_joy_.axes.at(0) > 0) {
            params_cpg_hopf_->cpg_uvc_y_val += 5;
            if (params_cpg_hopf_->cpg_uvc_y_val > 10) {
                params_cpg_hopf_->cpg_uvc_y_val = 10;
            }
            RCLCPP_INFO(get_logger(), "left: [%f]", params_cpg_hopf_->cpg_uvc_y_val);
            setParam();
        } else if (msg_joy_.axes.at(0) < 0) {
            params_cpg_hopf_->cpg_uvc_y_val -= 5;
            if (params_cpg_hopf_->cpg_uvc_y_val < -10) {
                params_cpg_hopf_->cpg_uvc_y_val = -10;
            }
            RCLCPP_INFO(get_logger(), "right: [%f]", params_cpg_hopf_->cpg_uvc_y_val);
            setParam();
        }
        // Enlarged and reduce wave angle.
        if (msg_joy_.buttons.at(4)) {
            params_cpg_hopf_->cpg_wave_ay_val += 5;
            if (params_cpg_hopf_->cpg_wave_ay_val > 50) {
                params_cpg_hopf_->cpg_wave_ay_val = 50;
            }
            RCLCPP_INFO(get_logger(), "enlarged wave angle: [%f]", params_cpg_hopf_->cpg_wave_ay_val);
            setParam();
        } else if (msg_joy_.buttons.at(5)) {
            params_cpg_hopf_->cpg_wave_ay_val -= 5;
            if (params_cpg_hopf_->cpg_wave_ay_val <= 0.1) {
                params_cpg_hopf_->cpg_wave_ay_val = 0.1;
            }
            RCLCPP_INFO(get_logger(), "reduce wave angle: [%f]", params_cpg_hopf_->cpg_wave_ay_val);
            setParam();
        }
    } else if (params_cpg_hopf_->cpg_motion_mode_val == snake::SnakeParamsCPGHopf::MotionMode::ARC_ROLLING_LOCOMOTION) {
        // TODO.
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::ARC_ROLLING_LOCOMOTION;
    } else if (params_cpg_hopf_->cpg_motion_mode_val ==
               snake::SnakeParamsCPGHopf::MotionMode::SPIRAL_ROLLING_LOCOMOTION) {
        // TODO.
        params_cpg_hopf_->cpg_motion_mode_val = snake::SnakeParamsCPGHopf::MotionMode::SPIRAL_ROLLING_LOCOMOTION;
    }
}
