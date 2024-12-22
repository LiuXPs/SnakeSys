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

#ifndef SNAKE_SENSOR_STEREO_SEGMENT_NODE_H
#define SNAKE_SENSOR_STEREO_SEGMENT_NODE_H

#include <iostream>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <snake_params/snake_params_sensor.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>

class SnakeSensorStereoSegmentNode : public rclcpp::Node {
public:
    SnakeSensorStereoSegmentNode() = delete;

    explicit SnakeSensorStereoSegmentNode(std::string name);

    ~SnakeSensorStereoSegmentNode() override;

private:
    void cbStereo(const sensor_msgs::msg::Image::ConstSharedPtr msg_stereo);

private:
    const int queue_size_ = 5;
//    std::shared_ptr<image_transport::ImageTransport> it_;
    std::shared_ptr<snake::SnakeParamsSensor> params_sensor_;

//    image_transport::Subscriber sub_stereo_;
//    image_transport::Publisher pub_left_;
//    image_transport::Publisher pub_right_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stereo_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_left_comp_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_right_comp_;

    cv_bridge::CvImageConstPtr cb_ptr_;
    cv_bridge::CvImage cb_left_;
    cv_bridge::CvImage cb_right_;

    cv::Mat cv_stereo_;
    cv::Mat cv_left_;
    cv::Mat cv_right_;

    sensor_msgs::msg::Image msg_left_;
    sensor_msgs::msg::Image msg_right_;
    sensor_msgs::msg::CompressedImage msg_left_comp_;
    sensor_msgs::msg::CompressedImage msg_right_comp_;

    std::vector<int> params_comp_;
};

#endif //SNAKE_SENSOR_STEREO_SEGMENT_NODE_H
