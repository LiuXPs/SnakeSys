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

#ifndef SNAKE_SIM_NODE_H
#define SNAKE_SIM_NODE_H

#include <iostream>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <snake_msgs/msg/snake_heart.hpp>
#include <snake_msgs/msg/servo_execute.hpp>
#include <snake_msgs/msg/servo_feedback.hpp>
#include <snake_params/snake_params_heart.h>
#include <snake_params/snake_params_cpg_hopf.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

class SnakeSimNode : public rclcpp::Node {
public:
    SnakeSimNode() = delete;

    explicit SnakeSimNode(std::string name);

    ~SnakeSimNode() override;

private:
    void resetJointName(int num);

    void cbHeart(const snake_msgs::msg::SnakeHeart::SharedPtr msg_heart);

    void cbExecute(const snake_msgs::msg::ServoExecute::SharedPtr msg_execute);

    void cbJointStates(const sensor_msgs::msg::JointState::SharedPtr msg_joints);

    void cbLinkStates(const gazebo_msgs::msg::LinkStates::SharedPtr msg_links);

private:
    uint32_t seq_val_;
    rclcpp::QoS queue_size_ = 10;
    const std::string frame_id_str_ = "simulate_feedback_frame";

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf2_broadcaster_;
    geometry_msgs::msg::TransformStamped msg_tf_snake_base_;
    const std::string snake_world_frame_id_ = "snake_world";
    const std::string snake_base_frame_id_ = "snake_base";
    const std::string snake_dummy_frame_name_ = "snake_robot::dummy_link";
    std::vector<std::string> joints_name_;

    const std::string sub_heart_str_ = "snake_heart";
    const std::string sub_execute_str_ = "servo_execute";
    const std::string sub_joints_str_ = "/joint_states";
    const std::string sub_links_str_ = "/gazebo/link_states";
    const std::string pub_feedback_str_ = "servo_feedback";

    // ROS2.
//    const std::string pub_position_str_ = "/position_controllers/commands";
    // ROS1.
    const std::string pub_position_str_ = "/position_controllers/command";

    snake_msgs::msg::SnakeHeart msg_heart_;
    snake_msgs::msg::ServoExecute msg_execute_;
    sensor_msgs::msg::JointState msg_joints_;
    gazebo_msgs::msg::LinkStates msg_links_;
    snake_msgs::msg::ServoFeedback msg_feedback_;
    std_msgs::msg::Float64MultiArray msg_position_;

    rclcpp::Subscription<snake_msgs::msg::SnakeHeart>::SharedPtr sub_heart_;
    rclcpp::Subscription<snake_msgs::msg::ServoExecute>::SharedPtr sub_execute_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub_links_;
    rclcpp::Publisher<snake_msgs::msg::ServoFeedback>::SharedPtr pub_feedback_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_position_;
};

#endif //SNAKE_SIM_NODE_H
