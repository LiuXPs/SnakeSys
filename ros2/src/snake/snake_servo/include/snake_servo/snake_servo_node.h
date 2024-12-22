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

#ifndef SNAKE_SERVO_NODE_H
#define SNAKE_SERVO_NODE_H

#define ENABLE_SERVO

#include <iostream>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <snake_servo/snake_feetech_servo.h>
#include <snake_msgs/msg/snake_heart.hpp>
#include <snake_msgs/msg/servo_feedback.hpp>
#include <snake_msgs/msg/servo_execute.hpp>
#include <snake_params/snake_params_servo.h>

class SnakeServoNode : public rclcpp::Node {
public:
    SnakeServoNode() = delete;

    SnakeServoNode(const SnakeServoNode &msp) = delete;

    explicit SnakeServoNode(std::string name);

    ~SnakeServoNode() override;

    SnakeServoNode &operator=(const SnakeServoNode &msp);

private:
    void cbHeart(const snake_msgs::msg::SnakeHeart::SharedPtr msg_heart);

    void cbExecute(const snake_msgs::msg::ServoExecute::SharedPtr msg_execute);

public:
    snake::FeeTechServo servo;

private:
    const double max_speed = 0.154; // unit:0.154sec/60degree. Measured value of FeeTech official website.
    double max_rmp;
    double unit_velocity;
    const double max_torque = 85; // unit:kg.cm. Measured value of FeeTech official website.
    double unit_torque;

private:
    snake::u8 joints_idn_;
    snake::u8 *joints_id_;
    snake::s16 *position_;
    snake::u16 *speed_;
    snake::u8 *acc_;
    int *position_fb_;
    int *speed_fb_;
    int *load_fb_;
    int *voltage_fb_;
    int *temper_fb_;
    int *move_fb_;
    int *current_fb_;

    const snake::s16 servo_pos_max_ = 3071;
    const snake::s16 servo_pos_min_ = 1023;

    rclcpp::QoS queue_size_ = 10;
    uint32_t seq_val_;
    const std::string frame_id_str_ = "servo_feedback_frame";

    snake::SnakeParamsServo params_servo_ = snake::SnakeParamsServo();

    rclcpp::Subscription<snake_msgs::msg::SnakeHeart>::SharedPtr sub_heart_;
    rclcpp::Subscription<snake_msgs::msg::ServoExecute>::SharedPtr sub_execute_;
    rclcpp::Publisher<snake_msgs::msg::ServoFeedback>::SharedPtr pub_feedback_;

    const std::string sub_heart_str_ = "snake_heart";
    const std::string sub_execute_str_ = "servo_execute";
    const std::string pub_feedback_str_ = "servo_feedback";
    snake_msgs::msg::SnakeHeart msg_heart_;
    snake_msgs::msg::ServoExecute msg_execute_;
    snake_msgs::msg::ServoFeedback msg_feedback_;
};

#endif //SNAKE_SERVO_NODE_H
