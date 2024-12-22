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

#ifndef SNAKE_JOY_NODE_H
#define SNAKE_JOY_NODE_H

#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <snake_params/snake_params_cpg_hopf.h>

class SnakeJoyNode : public rclcpp::Node {
public:
    SnakeJoyNode() = delete;

    explicit SnakeJoyNode(std::string name);

    ~SnakeJoyNode() override;

private:
    void getParam();

    void setParam();

    void cbJoy(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

private: // ROS2.
    rclcpp::QoS queue_size_ = 10;

    const std::string sub_joy_str_ = "joy";
    sensor_msgs::msg::Joy msg_joy_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

    const std::string node_cpg_hopf_str_ = "snake_cpg_hopf";
    std::shared_ptr<snake::SnakeParamsCPGHopf> params_cpg_hopf_;

    rclcpp::AsyncParametersClient::SharedPtr client_param_;
    std::shared_future<std::vector<rclcpp::Parameter>> get_params_value_;
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_params_value_;
    std::vector<std::string> get_params_name_;
    std::vector<rclcpp::Parameter> params_;
};

#endif //SNAKE_JOY_NODE_H
