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

#ifndef SNAKE_HEART_NODE_H
#define SNAKE_HEART_NODE_H

#include <iostream>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/string.hpp>
#include <snake_msgs/msg/snake_heart.hpp>
#include <snake_params/snake_params_heart.h>

using namespace std::chrono_literals;

class SnakeHeartNode : public rclcpp::Node {
public:
    SnakeHeartNode() = delete;

    explicit SnakeHeartNode(std::string name);

    ~SnakeHeartNode() override;

private:
    // node param.
    const std::string topic_name_ = "snake_heart";
    rclcpp::QoS queue_size_ = 10;
    std::uint32_t seq_val_;

    snake_msgs::msg::SnakeHeart heart_msg_;
    const std::string frame_id_str_ = "snake_heart_frame";
    rclcpp::Publisher<snake_msgs::msg::SnakeHeart>::SharedPtr publisher_;

    // Timer.
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Duration> interval_;
    rclcpp::Time time_last_;
    rclcpp::Time time_now_;
    int timer_rate_;

    snake::SnakeParamsHeart params_heart_;

private:
    void respond_heart();
};

#endif// SNAKE_HEART_NODE_H
