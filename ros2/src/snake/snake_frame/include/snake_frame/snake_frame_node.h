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

#ifndef SNAKE_FRAME_NODE_H
#define SNAKE_FRAME_NODE_H

#define FROM_FEEDBACK
#define SLAM_FRAME

#include <iostream>
#include <string>
#include <vector>

#include "snake_frame/snake_frame.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <snake_msgs/msg/servo_execute.hpp>
#include <snake_msgs/msg/servo_feedback.hpp>
#include <snake_params/snake_params_heart.h>
#include <snake_params/snake_params_frame.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

class SnakeFrameNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SnakeFrameNode() = delete;

    explicit SnakeFrameNode(std::string name);

    ~SnakeFrameNode() override;

private:
    void calculateTheta();

    void loadConstFrame();

    void resetFrameParam();

    void pubConstFrame();

    void pubRelFrame();

#ifdef FROM_FEEDBACK

    void cbTheta(const snake_msgs::msg::ServoFeedback::SharedPtr msg_theta);

#else

    void cbTheta(const snake_msgs::msg::ServoExecute::SharedPtr msg_theta);

#endif

private: // ROS2.
    rclcpp::QoS queue_size_ = 10;
    uint32_t seq_val_;

    const std::string node_heart_str_ = "snake_heart";
    snake::SnakeParamsHeart params_heart_ = snake::SnakeParamsHeart();
    snake::SnakeParamsFrame params_frame_ = snake::SnakeParamsFrame();

    rclcpp::AsyncParametersClient::SharedPtr client_get_;
    std::shared_future<std::vector<rclcpp::Parameter>> results_;

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf2_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

#ifdef FROM_FEEDBACK
    rclcpp::Subscription<snake_msgs::msg::ServoFeedback>::SharedPtr sub_theta_;
    const std::string sub_theta_str_ = "servo_feedback"; // Topic name.
    snake_msgs::msg::ServoFeedback msg_theta_;
#else
    rclcpp::Subscription<snake_msgs::msg::ServoExecute>::SharedPtr sub_theta_;
    const std::string sub_theta_str_ = "servo_execute"; // Topic name.
    snake_msgs::msg::ServoExecute msg_theta_;
#endif

private: // Frame.
    cv::FileStorage fs_;
    const std::string params_path_ = "./src/snake/snake_frame/config/";
//    const std::string params_name_ = "frame_d455.xml";
    const std::string params_name_ = "frame_zed2i.xml";
//    const std::string params_name_ = "frame_stereo.xml";

    snake::SnakeFrame snake_frame_;
    std::vector<double> alpha_;
    std::vector<double> a_;
    std::vector<double> theta_;
    std::vector<double> d_;

    // Snake frame.
    Sophus::SE3<double> FRAME_BASE_CAM1_; // Obtained from calibration.
    Sophus::SE3<double> FRAME_CAM1_CAM2_; // Obtained from calibration.
    Sophus::SE3<double> FRAME_CAM1_IMU_; // Obtained from calibration.
    Sophus::SE3<double> FRAME_CAM1_BASE_;
    Sophus::SE3<double> FRAME_BASE_CAM2_;
    Sophus::SE3<double> FRAME_BASE_IMU_;
    Sophus::SE3<double> frame_world_base_;

    std::vector<Sophus::SE3<double>> frame_link_rel_;
    std::vector<Sophus::SE3<double>> frame_link_abs_;

    // tf2.
    // snake_world-->snake_base.
    // snake_base-->snake_camera_1
    // snake_base-->snake_camera_2.
    // snake_base-->snake_imu.
    // snake_base-->link_1-->link_2-->...-->link_n-1-->link_n.
    geometry_msgs::msg::TransformStamped msg_tfs_base_;
    geometry_msgs::msg::TransformStamped msg_tfs_cam1_;
    geometry_msgs::msg::TransformStamped msg_tfs_cam2_;
    geometry_msgs::msg::TransformStamped msg_tfs_imu_;
    std::vector<geometry_msgs::msg::TransformStamped> msg_tfs_links_;
    std::vector<std::string> frame_id_links_child_;
    const std::string frame_id_base_parent_ = "snake_world";
    const std::string frame_id_base_child_ = "snake_base";
    const std::string frame_id_cam1_parent_ = "snake_base";
    const std::string frame_id_cam1_child_ = "snake_camera_1";
    const std::string frame_id_cam2_parent_ = "snake_base";
    const std::string frame_id_cam2_child_ = "snake_camera_2";
    const std::string frame_id_imu_parent_ = "snake_base";
    const std::string frame_id_imu_child_ = "snake_imu";
};

#endif //SNAKE_FRAME_NODE_H
