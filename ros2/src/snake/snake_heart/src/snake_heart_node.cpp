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

#include "snake_heart/snake_heart_node.h"

SnakeHeartNode::SnakeHeartNode(std::string name) : rclcpp::Node(name) {
    params_heart_.heart_rate_val = 10;
    params_heart_.servo_idn_val = 28;
    params_heart_.link_length_val = 0.01;
    seq_val_ = 0;

    declare_parameter<int>(params_heart_.heart_rate_str, params_heart_.heart_rate_val);
    declare_parameter<int>(params_heart_.servo_idn_str, params_heart_.servo_idn_val);
    declare_parameter<double>(params_heart_.link_length_str, params_heart_.link_length_val);
    get_parameter(params_heart_.heart_rate_str, params_heart_.heart_rate_val);
    get_parameter(params_heart_.servo_idn_str, params_heart_.servo_idn_val);
    get_parameter(params_heart_.link_length_str, params_heart_.link_length_val);
    get_parameter(params_heart_.use_sim_time_str, params_heart_.use_sim_time_val);
    std::cout << params_heart_ << std::endl;

    publisher_ = create_publisher<snake_msgs::msg::SnakeHeart>(topic_name_, queue_size_);

    time_last_ = now();
    time_now_ = now();
    timer_rate_ = -1;
    respond_heart();
}

SnakeHeartNode::~SnakeHeartNode() {
}

void SnakeHeartNode::respond_heart() {
    seq_val_ += 1;
    get_parameter(params_heart_.heart_rate_str, params_heart_.heart_rate_val);
    get_parameter(params_heart_.servo_idn_str, params_heart_.servo_idn_val);
    get_parameter(params_heart_.link_length_str, params_heart_.link_length_val);
    get_parameter(params_heart_.use_sim_time_str, params_heart_.use_sim_time_val);

    heart_msg_.seq = seq_val_;
    heart_msg_.heart = params_heart_.heart_rate_val;
    heart_msg_.idn = uint8_t(params_heart_.servo_idn_val);
    heart_msg_.header.frame_id = frame_id_str_;

    if (timer_rate_ != params_heart_.heart_rate_val) {
        timer_rate_ = params_heart_.heart_rate_val;

        interval_ = std::make_shared<rclcpp::Duration>(static_cast<int>(1000.0 / timer_rate_) * 1ms);
        timer_ = create_wall_timer(static_cast<int>(1000.0 / timer_rate_) * 1ms,
                                   std::bind(&SnakeHeartNode::respond_heart, this));
    }

    time_now_ = now();
    if ((time_now_ - time_last_) >= *interval_) {
        heart_msg_.header.stamp = now();
        time_last_ = heart_msg_.header.stamp;
        publisher_->publish(heart_msg_);

        RCLCPP_INFO(get_logger(),
                    "Heart ===> Seq:[%d]\tHeart Rate:[%d]\tServo IDn:[%d]",
                    heart_msg_.seq, heart_msg_.heart,
                    heart_msg_.idn);
    }
}
