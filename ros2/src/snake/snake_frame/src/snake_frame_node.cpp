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

#include "snake_frame/snake_frame_node.h"

SnakeFrameNode::SnakeFrameNode(std::string name) : rclcpp::Node(name) {
    seq_val_ = 0;
    tf2_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

#ifdef SLAM_FRAME
    // TODO.
#else
    client_get_ = std::make_shared<rclcpp::AsyncParametersClient>(this, node_heart_str_);
    while (!client_get_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            exit(1);
        }
        RCLCPP_ERROR(get_logger(), "Service unavailable ... ");
    }
#endif

    loadConstFrame();
    pubConstFrame();

#ifdef SLAM_FRAME
    // TODO.
    params_heart_.heart_rate_val = 50;
    params_heart_.servo_idn_val = 28;
    params_heart_.link_length_val = 0.16;
#else
    // Snake frame.
    results_ = client_get_->get_parameters({params_heart_.heart_rate_str,
                                            params_heart_.servo_idn_str,
                                            params_heart_.link_length_str});
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), results_) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        params_heart_.heart_rate_val = static_cast<int>(results_.get().at(0).as_int());
        params_heart_.servo_idn_val = static_cast<int>(results_.get().at(1).as_int());
        params_heart_.link_length_val = static_cast<double>(results_.get().at(2).as_double());
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get the parameter value.");
        exit(1);
    }
#endif

    resetFrameParam();

    // Obtaining joint angles from feedback or execute.
    auto bind_callback = std::bind(&SnakeFrameNode::cbTheta, this, std::placeholders::_1);
#ifdef FROM_FEEDBACK
    sub_theta_ = this->create_subscription<snake_msgs::msg::ServoFeedback>(sub_theta_str_,
                                                                           queue_size_,
                                                                           bind_callback);
#else
    sub_theta_ = this->create_subscription<snake_msgs::msg::ServoExecute>(sub_theta_str_,
                                                                          queue_size_,
                                                                          bind_callback);
#endif

    get_parameter(params_frame_.use_sim_time_str, params_frame_.use_sim_time_val);
    std::cout << params_frame_ << std::endl;
}

SnakeFrameNode::~SnakeFrameNode() {
}

void SnakeFrameNode::calculateTheta() {
    if (params_heart_.servo_idn_val != static_cast<int>(msg_theta_.id.size())) {
        params_heart_.servo_idn_val = static_cast<int>(msg_theta_.id.size());
        resetFrameParam();
    }

    // Get the transformation from snake_base to snake_camera_1.
    try {
        msg_tfs_base_ = tf2_buffer_->lookupTransform(frame_id_base_parent_,
                                                     frame_id_base_child_,
                                                     tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
    Eigen::Quaterniond q_pose;
    q_pose.w() = msg_tfs_base_.transform.rotation.w;
    q_pose.x() = msg_tfs_base_.transform.rotation.x;
    q_pose.y() = msg_tfs_base_.transform.rotation.y;
    q_pose.z() = msg_tfs_base_.transform.rotation.z;
    q_pose = q_pose.normalized();
    Eigen::Vector3d t_pose(msg_tfs_base_.transform.translation.x,
                           msg_tfs_base_.transform.translation.y,
                           msg_tfs_base_.transform.translation.z);
    frame_world_base_.setRotationMatrix(Sophus::SE3<double>(q_pose, t_pose).rotationMatrix());

    for (int i = 0; i < params_heart_.servo_idn_val; i++) {

#ifdef FROM_FEEDBACK
        double theta = -1 * msg_theta_.position.at(i);
#else
        double theta = -1 * msg_theta_.pos.at(i);
#endif

        if (i == 0) {
            theta_.at(i) = theta + M_PI_2;
        } else {
            theta_.at(i) = theta;
        }
    }

    snake_frame_.updateFrame(frame_world_base_, theta_);
    frame_link_rel_ = snake_frame_.getRelFrame();
    frame_link_abs_ = snake_frame_.getAbsFrame();

//    pubConstFrame();
    pubRelFrame();
}

void SnakeFrameNode::loadConstFrame() {
    cv::Mat frame_base_cam1_cv;
    cv::Mat frame_cam1_cam2_cv;
    cv::Mat frame_cam1_imu_cv;

    fs_.open((params_path_ + params_name_).c_str(), cv::FileStorage::READ);
    fs_["cam1_to_base"] >> frame_base_cam1_cv;
    fs_["cam2_to_cam1"] >> frame_cam1_cam2_cv;
    fs_["imu_to_cam1"] >> frame_cam1_imu_cv;
    fs_.release();

    // Homogeneous transformation matrix of 4x4.
    const cv::Size frame_size(4, 4);
    CV_Assert(frame_size == frame_base_cam1_cv.size() &&
              frame_size == frame_cam1_cam2_cv.size() &&
              frame_size == frame_cam1_imu_cv.size());

    Eigen::Matrix4d frame_base_cam1_eigen;
    Eigen::Matrix4d frame_cam1_cam2_eigen;
    Eigen::Matrix4d frame_cam1_imu_eigen;

    cv::cv2eigen(frame_base_cam1_cv, frame_base_cam1_eigen);
    cv::cv2eigen(frame_cam1_cam2_cv, frame_cam1_cam2_eigen);
    cv::cv2eigen(frame_cam1_imu_cv, frame_cam1_imu_eigen);

    FRAME_BASE_CAM1_ = Sophus::SE3<double>(frame_base_cam1_eigen.block<3, 3>(0, 0),
                                           frame_base_cam1_eigen.block<3, 1>(0, 3));
    FRAME_CAM1_CAM2_ = Sophus::SE3<double>(frame_cam1_cam2_eigen.block<3, 3>(0, 0),
                                           frame_cam1_cam2_eigen.block<3, 1>(0, 3));
    FRAME_CAM1_IMU_ = Sophus::SE3<double>(frame_cam1_imu_eigen.block<3, 3>(0, 0),
                                          frame_cam1_imu_eigen.block<3, 1>(0, 3));
    FRAME_CAM1_BASE_ = FRAME_BASE_CAM1_.inverse();
    FRAME_BASE_CAM2_ = FRAME_BASE_CAM1_ * FRAME_CAM1_CAM2_;
    FRAME_BASE_IMU_ = FRAME_BASE_CAM1_ * FRAME_CAM1_IMU_;
}

void SnakeFrameNode::resetFrameParam() {
    frame_world_base_ = Sophus::SE3<double>(Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d::Zero());

    alpha_.clear();
    a_.clear();
    theta_.clear();
    d_.clear();
    frame_id_links_child_.clear();
    msg_tfs_links_.clear();

    alpha_.resize(params_heart_.servo_idn_val);
    a_.resize(params_heart_.servo_idn_val);
    theta_.resize(params_heart_.servo_idn_val);
    d_.resize(params_heart_.servo_idn_val);
    frame_id_links_child_.resize(params_heart_.servo_idn_val);
    msg_tfs_links_.resize(params_heart_.servo_idn_val);

    for (int i = 0; i < params_heart_.servo_idn_val; i++) {
        if (i == 0) {
            alpha_.at(i) = 0.0;
            a_.at(i) = 0.0;
            theta_.at(i) = M_PI_2;
            d_.at(i) = 0.0;
        } else {
            alpha_.at(i) = pow(-1, i) * M_PI_2;
            a_.at(i) = params_heart_.link_length_val;
            theta_.at(i) = -0.0;
            d_.at(i) = 0.0;
        }
        frame_id_links_child_.at(i) = "snake_link_" + std::to_string(i + 1);
    }

    snake_frame_.initFrame(frame_world_base_,
                           alpha_,
                           a_,
                           theta_,
                           d_);
    frame_link_rel_ = snake_frame_.getRelFrame();
    frame_link_abs_ = snake_frame_.getAbsFrame();
}

void SnakeFrameNode::pubConstFrame() {
    // Base frame.
//    msg_tfs_base_.header.frame_id = frame_id_base_parent_;
//    msg_tfs_base_.child_frame_id = frame_id_base_child_;
//    msg_tfs_base_.header.stamp = now();
//    msg_tfs_base_.transform.rotation.x = 0;
//    msg_tfs_base_.transform.rotation.y = 0;
//    msg_tfs_base_.transform.rotation.z = 0;
//    msg_tfs_base_.transform.rotation.w = 1;
//    msg_tfs_base_.transform.translation.x = 0;
//    msg_tfs_base_.transform.translation.y = 0;
//    msg_tfs_base_.transform.translation.z = 0;

//    tf2_broadcaster_->sendTransform(msg_tfs_base_);

    // Camera1 frame.
    msg_tfs_cam1_.header.frame_id = frame_id_cam1_parent_;
    msg_tfs_cam1_.child_frame_id = frame_id_cam1_child_;
    msg_tfs_cam1_.header.stamp = now();
    msg_tfs_cam1_.transform.rotation.x = FRAME_BASE_CAM1_.unit_quaternion().x();
    msg_tfs_cam1_.transform.rotation.y = FRAME_BASE_CAM1_.unit_quaternion().y();
    msg_tfs_cam1_.transform.rotation.z = FRAME_BASE_CAM1_.unit_quaternion().z();
    msg_tfs_cam1_.transform.rotation.w = FRAME_BASE_CAM1_.unit_quaternion().w();
    msg_tfs_cam1_.transform.translation.x = FRAME_BASE_CAM1_.translation().x();
    msg_tfs_cam1_.transform.translation.y = FRAME_BASE_CAM1_.translation().y();
    msg_tfs_cam1_.transform.translation.z = FRAME_BASE_CAM1_.translation().z();

    tf2_broadcaster_->sendTransform(msg_tfs_cam1_);

    // Camera2 frame.
    msg_tfs_cam2_.header.frame_id = frame_id_cam2_parent_;
    msg_tfs_cam2_.child_frame_id = frame_id_cam2_child_;
    msg_tfs_cam2_.header.stamp = now();
    msg_tfs_cam2_.transform.rotation.x = FRAME_BASE_CAM2_.unit_quaternion().x();
    msg_tfs_cam2_.transform.rotation.y = FRAME_BASE_CAM2_.unit_quaternion().y();
    msg_tfs_cam2_.transform.rotation.z = FRAME_BASE_CAM2_.unit_quaternion().z();
    msg_tfs_cam2_.transform.rotation.w = FRAME_BASE_CAM2_.unit_quaternion().w();
    msg_tfs_cam2_.transform.translation.x = FRAME_BASE_CAM2_.translation().x();
    msg_tfs_cam2_.transform.translation.y = FRAME_BASE_CAM2_.translation().y();
    msg_tfs_cam2_.transform.translation.z = FRAME_BASE_CAM2_.translation().z();

    tf2_broadcaster_->sendTransform(msg_tfs_cam2_);

    // IMU frame.
    msg_tfs_imu_.header.frame_id = frame_id_imu_parent_;
    msg_tfs_imu_.child_frame_id = frame_id_imu_child_;
    msg_tfs_imu_.header.stamp = now();
    msg_tfs_imu_.transform.rotation.x = FRAME_BASE_IMU_.unit_quaternion().x();
    msg_tfs_imu_.transform.rotation.y = FRAME_BASE_IMU_.unit_quaternion().y();
    msg_tfs_imu_.transform.rotation.z = FRAME_BASE_IMU_.unit_quaternion().z();
    msg_tfs_imu_.transform.rotation.w = FRAME_BASE_IMU_.unit_quaternion().w();
    msg_tfs_imu_.transform.translation.x = FRAME_BASE_IMU_.translation().x();
    msg_tfs_imu_.transform.translation.y = FRAME_BASE_IMU_.translation().y();
    msg_tfs_imu_.transform.translation.z = FRAME_BASE_IMU_.translation().z();

    tf2_broadcaster_->sendTransform(msg_tfs_imu_);
}

void SnakeFrameNode::pubRelFrame() {
    for (int i = 0; i < static_cast<int>(params_heart_.servo_idn_val); i++) {
        if (i == 0) {
            msg_tfs_links_.at(i).header.frame_id = frame_id_base_child_;
        } else {
            msg_tfs_links_.at(i).header.frame_id = frame_id_links_child_.at(i - 1);
        }
        msg_tfs_links_.at(i).child_frame_id = frame_id_links_child_.at(i);
        msg_tfs_links_.at(i).header.stamp = msg_theta_.header.stamp;

        msg_tfs_links_.at(i).transform.rotation.x = frame_link_rel_.at(i).unit_quaternion().x();
        msg_tfs_links_.at(i).transform.rotation.y = frame_link_rel_.at(i).unit_quaternion().y();
        msg_tfs_links_.at(i).transform.rotation.z = frame_link_rel_.at(i).unit_quaternion().z();
        msg_tfs_links_.at(i).transform.rotation.w = frame_link_rel_.at(i).unit_quaternion().w();
        msg_tfs_links_.at(i).transform.translation.x = frame_link_rel_.at(i).translation().x();
        msg_tfs_links_.at(i).transform.translation.y = frame_link_rel_.at(i).translation().y();
        msg_tfs_links_.at(i).transform.translation.z = frame_link_rel_.at(i).translation().z();
    }

    if (!params_frame_.use_sim_time_val) {
        tf2_broadcaster_->sendTransform(msg_tfs_links_);

        RCLCPP_INFO(get_logger(),
                    "Seq:[%d]:link1\t===>q:x[%f]y[%f]z[%f]w[%f]\tt:x[%f]y[%f]z[%f]",
                    seq_val_,
                    msg_tfs_links_.at(0).transform.rotation.x,
                    msg_tfs_links_.at(0).transform.rotation.y,
                    msg_tfs_links_.at(0).transform.rotation.z,
                    msg_tfs_links_.at(0).transform.rotation.w,
                    msg_tfs_links_.at(0).transform.translation.x,
                    msg_tfs_links_.at(0).transform.translation.y,
                    msg_tfs_links_.at(0).transform.translation.z);
    }
}

#ifdef FROM_FEEDBACK

void SnakeFrameNode::cbTheta(const snake_msgs::msg::ServoFeedback::SharedPtr msg_theta) {
    msg_theta_.header = msg_theta->header;
    msg_theta_.seq = msg_theta->seq;
    msg_theta_.id = msg_theta->id;
    msg_theta_.position = msg_theta->position;
    msg_theta_.speed = msg_theta->speed;
    msg_theta_.load = msg_theta->load;
    msg_theta_.voltage = msg_theta->voltage;
    msg_theta_.temperature = msg_theta->temperature;
    msg_theta_.move = msg_theta->move;
    msg_theta_.current = msg_theta->current;

    calculateTheta();
}

#else

void SnakeFrameNode::cbTheta(const snake_msgs::msg::ServoExecute::SharedPtr msg_theta) {
    msg_theta_.header = msg_theta->header;
    msg_theta_.seq = msg_theta->seq;
    msg_theta_.id = msg_theta->id;
    msg_theta_.pos = msg_theta->pos;
    msg_theta_.speed = msg_theta->speed;
    msg_theta_.acc = msg_theta->acc;

    calculateTheta();
}

#endif
