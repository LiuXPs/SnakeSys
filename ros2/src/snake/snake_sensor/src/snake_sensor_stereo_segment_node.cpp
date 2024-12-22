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

#include "snake_sensor/snake_sensor_stereo_segment_node.h"

SnakeSensorStereoSegmentNode::SnakeSensorStereoSegmentNode(std::string name) : rclcpp::Node(name) {
//    it_ = std::make_shared<image_transport::ImageTransport>(static_cast<SharedPtr>(this));
    params_sensor_ = std::make_shared<snake::SnakeParamsSensor>();
    params_comp_.clear();
    params_comp_.push_back(cv::IMWRITE_JPEG_QUALITY);
    params_comp_.push_back(95);

    declare_parameter<std::string>(params_sensor_->topic_stereo_str_, params_sensor_->topic_stereo_val_);
    declare_parameter<std::string>(params_sensor_->topic_left_str_, params_sensor_->topic_left_val_);
    declare_parameter<std::string>(params_sensor_->topic_right_str_, params_sensor_->topic_right_val_);

    get_parameter(params_sensor_->topic_stereo_str_, params_sensor_->topic_stereo_val_);
    get_parameter(params_sensor_->topic_left_str_, params_sensor_->topic_left_val_);
    get_parameter(params_sensor_->topic_right_str_, params_sensor_->topic_right_val_);

    auto bind_stereo = std::bind(&SnakeSensorStereoSegmentNode::cbStereo, this, std::placeholders::_1);
//    sub_stereo_ = it_->subscribe(params_sensor_->topic_stereo_val_, queue_size_, bind_stereo);
//    pub_left_ = it_->advertise(params_sensor_->topic_left_val_, queue_size_);
//    pub_right_ = it_->advertise(params_sensor_->topic_right_val_, queue_size_);

    sub_stereo_ = this->create_subscription<sensor_msgs::msg::Image>(
            params_sensor_->topic_stereo_val_, rclcpp::QoS(queue_size_), bind_stereo);
    pub_left_ = this->create_publisher<sensor_msgs::msg::Image>(params_sensor_->topic_left_val_,
                                                                rclcpp::QoS(queue_size_));
    pub_right_ = this->create_publisher<sensor_msgs::msg::Image>(params_sensor_->topic_right_val_,
                                                                 rclcpp::QoS(queue_size_));
    pub_left_comp_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            params_sensor_->topic_left_val_ + "/compressed", rclcpp::QoS(queue_size_));
    pub_right_comp_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            params_sensor_->topic_right_val_ + "/compressed", rclcpp::QoS(queue_size_));
}

SnakeSensorStereoSegmentNode::~SnakeSensorStereoSegmentNode() {
}

void SnakeSensorStereoSegmentNode::cbStereo(const sensor_msgs::msg::Image::ConstSharedPtr msg_stereo) {
    try {
//        cb_ptr_ = cv_bridge::toCvShare(msg_stereo, sensor_msgs::image_encodings::RGB8);
        cb_ptr_ = cv_bridge::toCvShare(msg_stereo, msg_stereo->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        exit(1);
    }

    cb_ptr_->image.copyTo(cv_stereo_);
    cv_left_ = cv_stereo_(cv::Rect(0, 0,
                                   cv_stereo_.size().width / 2, cv_stereo_.size().height));
    cv_right_ = cv_stereo_(cv::Rect(cv_stereo_.size().width / 2, 0,
                                    cv_stereo_.size().width / 2, cv_stereo_.size().height));

    cb_left_.header.stamp = cb_ptr_->header.stamp;
    cb_right_.header.stamp = cb_ptr_->header.stamp;
    cb_left_.encoding = msg_stereo->encoding;
    cb_right_.encoding = msg_stereo->encoding;
    cb_left_.image = cv_left_;
    cb_right_.image = cv_right_;

    cb_left_.toImageMsg(msg_left_);
    cb_right_.toImageMsg(msg_right_);

    msg_left_comp_.header.stamp = cb_ptr_->header.stamp;
    msg_right_comp_.header.stamp = cb_ptr_->header.stamp;
    msg_left_comp_.format = "jpeg";
    msg_right_comp_.format = "jpeg";
    cv::imencode(".jpg", cv_left_, msg_left_comp_.data, params_comp_);
    cv::imencode(".jpg", cv_right_, msg_right_comp_.data, params_comp_);

    pub_left_->publish(msg_left_);
    pub_right_->publish(msg_right_);
    pub_left_comp_->publish(msg_left_comp_);
    pub_right_comp_->publish(msg_right_comp_);
}
