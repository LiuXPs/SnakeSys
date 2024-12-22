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

#include "snake_sim/snake_sim_node.h"

SnakeSimNode::SnakeSimNode(std::string name) : rclcpp::Node(name) {
    seq_val_ = 0;
    joints_name_.resize(0);
    tf2_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    auto bind_heart = std::bind(&SnakeSimNode::cbHeart, this, std::placeholders::_1);
    sub_heart_ = this->create_subscription<snake_msgs::msg::SnakeHeart>(sub_heart_str_,
                                                                        queue_size_,
                                                                        bind_heart);

    auto bind_execute = std::bind(&SnakeSimNode::cbExecute, this, std::placeholders::_1);
    sub_execute_ = this->create_subscription<snake_msgs::msg::ServoExecute>(sub_execute_str_,
                                                                            queue_size_,
                                                                            bind_execute);

    auto bind_joints = std::bind(&SnakeSimNode::cbJointStates, this, std::placeholders::_1);
    sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(sub_joints_str_,
                                                                          queue_size_,
                                                                          bind_joints);

    auto bind_links = std::bind(&SnakeSimNode::cbLinkStates, this, std::placeholders::_1);
    sub_links_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(sub_links_str_,
                                                                         queue_size_,
                                                                         bind_links);

    pub_feedback_ = this->create_publisher<snake_msgs::msg::ServoFeedback>(pub_feedback_str_,
                                                                           queue_size_);

    pub_position_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pub_position_str_,
                                                                             queue_size_);
}

SnakeSimNode::~SnakeSimNode() {

}

void SnakeSimNode::resetJointName(int num) {
    joints_name_.resize(num);
    for (int i = 0; i < num; i++) {
        joints_name_.at(i) = "snake_joint_" + std::to_string(i + 1);
    }
}

void SnakeSimNode::cbHeart(const snake_msgs::msg::SnakeHeart::SharedPtr msg_heart) {
    msg_heart_.header = msg_heart->header;
    msg_heart_.seq = msg_heart->seq;
    msg_heart_.heart = msg_heart->heart;
    msg_heart_.idn = msg_heart->idn;

    if (msg_feedback_.id.size() != msg_heart_.idn) {
        msg_feedback_.id.resize(msg_heart_.idn);
        msg_feedback_.position.resize(msg_heart_.idn);
        msg_feedback_.speed.resize(msg_heart_.idn);
        msg_feedback_.load.resize(msg_heart_.idn);
        msg_feedback_.voltage.resize(msg_heart_.idn);
        msg_feedback_.temperature.resize(msg_heart_.idn);
        msg_feedback_.move.resize(msg_heart_.idn);
        msg_feedback_.current.resize(msg_heart_.idn);

        for (int i = 0; i < static_cast<int>(msg_feedback_.id.size()); i++) {
            msg_feedback_.id.at(i) = i + 1;
            msg_feedback_.position.at(i) = 0.0;
            msg_feedback_.speed.at(i) = 0.0;
            msg_feedback_.load.at(i) = 0.0;
            msg_feedback_.voltage.at(i) = 0.0;
            msg_feedback_.temperature.at(i) = 0.0;
            msg_feedback_.move.at(i) = 0.0;
            msg_feedback_.current.at(i) = 0.0;
        }

        msg_feedback_.header.stamp = now();
        msg_feedback_.header.frame_id = frame_id_str_;
    }

    seq_val_ += 1;
    msg_feedback_.seq = seq_val_;
    msg_feedback_.header.stamp = now();
    msg_feedback_.header.frame_id = frame_id_str_;

    pub_feedback_->publish(msg_feedback_);

    RCLCPP_INFO(get_logger(), "SnakeSim Feedback ===> Seq:[%d]\tServo IDn:[%d]\tID[0] Pos:[%f]\tID[1] Pos:[%f]",
                msg_feedback_.seq,
                static_cast<int>(msg_feedback_.position.size()),
                msg_feedback_.position.at(0),
                msg_feedback_.position.at(1));
}

void SnakeSimNode::cbExecute(const snake_msgs::msg::ServoExecute::SharedPtr msg_execute) {
    msg_execute_.header = msg_execute->header;
    msg_execute_.seq = msg_execute->seq;
    msg_execute_.id = msg_execute->id;
    msg_execute_.pos = msg_execute->pos;
    msg_execute_.speed = msg_execute->speed;
    msg_execute_.acc = msg_execute->acc;
    msg_execute_.torque = msg_execute->torque;

    if (msg_position_.data.size() != msg_execute_.pos.size()) {
        msg_position_.data.resize(msg_execute_.pos.size());
    }

    for (int i = 0; i < static_cast<int>(msg_position_.data.size()); i++) {
        msg_position_.data.at(i) = msg_execute_.pos.at(i);
    }

    pub_position_->publish(msg_position_);

    RCLCPP_INFO(get_logger(), "SnakeSim Execute ===> Seq:[%d]\tServo IDn:[%d]\tID[0] Pos:[%f]\tID[1] Pos[1]:[%f]",
                msg_execute_.seq,
                static_cast<int>(msg_execute_.pos.size()),
                msg_execute_.pos.at(0),
                msg_execute_.pos.at(1));
}

void SnakeSimNode::cbJointStates(const sensor_msgs::msg::JointState::SharedPtr msg_joints) {
    msg_joints_.header = msg_joints->header;
    msg_joints_.name = msg_joints->name;
    msg_joints_.position = msg_joints->position;
    msg_joints_.velocity = msg_joints->velocity;
    msg_joints_.effort = msg_joints->effort;

    if (msg_feedback_.id.size() != msg_joints_.name.size()) {
        msg_feedback_.id.resize(msg_joints_.name.size());
        msg_feedback_.position.resize(msg_joints_.name.size());
        msg_feedback_.speed.resize(msg_joints_.name.size());
        msg_feedback_.load.resize(msg_joints_.name.size());
        msg_feedback_.voltage.resize(msg_joints_.name.size());
        msg_feedback_.temperature.resize(msg_joints_.name.size());
        msg_feedback_.move.resize(msg_joints_.name.size());
        msg_feedback_.current.resize(msg_joints_.name.size());
    }
    if (joints_name_.size() != msg_joints_.name.size()) {
        resetJointName(static_cast<int>(msg_joints_.name.size()));
    }

    msg_feedback_.header = msg_joints_.header;

    for (int i = 0; i < static_cast<int>(joints_name_.size()); i++) {
        auto it = std::find(msg_joints_.name.begin(), msg_joints_.name.end(), joints_name_.at(i));
        if (it != msg_joints_.name.end()) {
            int index = static_cast<int>(std::distance(msg_joints_.name.begin(), it));

            msg_feedback_.position.at(i) = msg_joints_.position.at(index);
            msg_feedback_.speed.at(i) = msg_joints_.velocity.at(index);
            msg_feedback_.load.at(i) = msg_joints_.effort.at(index);
        } else {
            std::cout << "joint name not found:\t" << joints_name_.at(i) << std::endl;
            exit(1);
        }
    }

    RCLCPP_INFO(get_logger(), "SnakeSim States ===> Link num:[%d]", msg_joints_.name.size());
}

void SnakeSimNode::cbLinkStates(const gazebo_msgs::msg::LinkStates::SharedPtr msg_links) {
    msg_links_.name = msg_links->name;
    msg_links_.pose = msg_links->pose;
    msg_links_.twist = msg_links->twist;

    auto it = std::find(msg_links_.name.begin(), msg_links_.name.end(), snake_dummy_frame_name_);
    if (it != msg_links_.name.end()) {
        int index = static_cast<int>(std::distance(msg_links_.name.begin(), it));

        msg_tf_snake_base_.transform.rotation.x = msg_links_.pose.at(index).orientation.x;
        msg_tf_snake_base_.transform.rotation.y = msg_links_.pose.at(index).orientation.y;
        msg_tf_snake_base_.transform.rotation.z = msg_links_.pose.at(index).orientation.z;
        msg_tf_snake_base_.transform.rotation.w = msg_links_.pose.at(index).orientation.w;
        msg_tf_snake_base_.transform.translation.x = msg_links_.pose.at(index).position.x;
        msg_tf_snake_base_.transform.translation.y = msg_links_.pose.at(index).position.y;
        msg_tf_snake_base_.transform.translation.z = msg_links_.pose.at(index).position.z;

        msg_tf_snake_base_.header.stamp = now();
        msg_tf_snake_base_.header.frame_id = snake_world_frame_id_;
        msg_tf_snake_base_.child_frame_id = snake_base_frame_id_;

        tf2_broadcaster_->sendTransform(msg_tf_snake_base_);

        RCLCPP_INFO(get_logger(), "SnakeSim Links ===> Link name:[%s]\tX:[%f]\tY:[%f]\tZ:[%f]",
                    msg_links_.name.at(index).c_str(),
                    msg_tf_snake_base_.transform.translation.x,
                    msg_tf_snake_base_.transform.translation.y,
                    msg_tf_snake_base_.transform.translation.z);
    } else {
        std::cout << "this link name not found:\t" << snake_dummy_frame_name_ << std::endl;
        exit(1);
    }
}
