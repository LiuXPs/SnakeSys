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

#ifndef SNAKE_SLAM_NODE_H
#define SNAKE_SLAM_NODE_H

#define IMAGE_RAW

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/System.h"
#include "include/ImuTypes.h"

// CPP.
#include <iostream>
#include <algorithm>
#include <fstream>
#include <functional>
#include <cmath>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>
#include <utility>

// ROS2.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <snake_params/snake_params_slam.h>

// OpenCV.
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Eigen and Sophus.
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

// Boost.
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>

class SnakeSLAMNode : public rclcpp::Node {
public:
    enum SLAMSystem {
        ORB_SLAM3 = 0,
    };
    enum SLAMType {
        MONO = 0,
        STEREO = 1,
        RGBD = 2,
        MONO_IMU = 3,
        STEREO_IMU = 4,
        RGBD_IMU = 5,
    };
public:
    static double toSec(const std_msgs::msg::Header &header);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SnakeSLAMNode() = delete;

    explicit SnakeSLAMNode(std::string name, SnakeSLAMNode::SLAMSystem system, SnakeSLAMNode::SLAMType type);

    ~SnakeSLAMNode();

private:
    void initSystem();

    // ROS2.
    void pubCameraPose(const Sophus::SE3f &trans_w_c, const rclcpp::Time &msg_time);

    void pubTrackedPoints(const std::vector<ORB_SLAM3::MapPoint *> &map_points, const rclcpp::Time &msg_time);

    void pubBaseTF(const Sophus::SE3f &trans_w_c, const rclcpp::Time &msg_time);

    sensor_msgs::msg::PointCloud2 toPointCloud(const std::vector<ORB_SLAM3::MapPoint *> &map_points,
                                               const rclcpp::Time &msg_time);

    // Grab Data.
    cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr msg_img, const std::string encode);

    // IMU.
    void grabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    // Monocular.
    void grabMono(const sensor_msgs::msg::Image::ConstSharedPtr &msg_mono);

    // Monocular compressed.
    void grabMonoComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_mono);

    // Monocular+IMU.
    void grabMonoIMU(const sensor_msgs::msg::Image::ConstSharedPtr &msg_mono);

    // Monocular+IMU compressed.
    void grabMonoIMUComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_mono);

    // Stereo.
    void grabStereo(const sensor_msgs::msg::Image::ConstSharedPtr &msg_left,
                    const sensor_msgs::msg::Image::ConstSharedPtr &msg_right);

    // Stereo compressed.
    void grabStereoComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_left,
                        const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_right);

    // Stereo+IMU.
    void grabStereoIMU(const sensor_msgs::msg::Image::ConstSharedPtr &msg_left,
                       const sensor_msgs::msg::Image::ConstSharedPtr &msg_right);

    // Stereo+IMU compressed.
    void grabStereoIMUComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_left,
                           const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_right);

    // RGB-D.
    void grabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msg_rgb,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msg_depth);

    // RGB-D compressed.
    void grabRGBDComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_rgb,
                      const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_depth);

    // RGB-D+IMU.
    void grabRGBDIMU(const sensor_msgs::msg::Image::ConstSharedPtr &msg_rgb,
                     const sensor_msgs::msg::Image::ConstSharedPtr &msg_depth);

    // RGB-D+IMU compressed.
    void grabRGBDIMUComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_rgb,
                         const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_depth);

private:
    int queue_size_ = 5;
    SnakeSLAMNode::SLAMSystem slam_system_;
    SnakeSLAMNode::SLAMType slam_type_;
    std::shared_ptr<snake::SnakeParamsSLAM> params_slam_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_camera_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_pc_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf2_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

    geometry_msgs::msg::PoseStamped msg_camera_pose_;
    sensor_msgs::msg::PointCloud2 msg_map_points_;
    geometry_msgs::msg::TransformStamped msg_camera_frame_;
    geometry_msgs::msg::TransformStamped msg_base_frame_;
    geometry_msgs::msg::TransformStamped msg_base_camera_;

    Sophus::SE3f trans_c0_w_;
    Sophus::SE3<double> frame_base_camera_;

    std::shared_ptr<image_transport::ImageTransport> it_;

    // IMU data.
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> buff_imu_;
    std::mutex mutex_imu_;

    // Image data.
    image_transport::Subscriber sub_mono_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_left_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_right_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_rgb_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_depth_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;

    cv_bridge::CvImageConstPtr cb_mono_ptr_;
    cv_bridge::CvImageConstPtr cb_left_ptr_;
    cv_bridge::CvImageConstPtr cb_right_ptr_;
    cv_bridge::CvImageConstPtr cb_rgb_ptr_;
    cv_bridge::CvImageConstPtr cb_depth_ptr_;

    double t_mono_img_;
    double t_left_img_;
    double t_right_img_;
    double t_rgb_img_;
    double t_depth_img_;

    // Image data compressed.
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_mono_comp_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_left_comp_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_right_comp_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_rgb_comp_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_depth_comp_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage> sync_pol_comp;
    std::shared_ptr<message_filters::Synchronizer<sync_pol_comp>> sync_comp_;

    cv::Mat cv_mono_img_;
    cv::Mat cv_left_img_;
    cv::Mat cv_right_img_;
    cv::Mat cv_rgb_img_;
    cv::Mat cv_depth_img_;

    // ORB-SLAM3 params.
    const double orb_slam3_max_time_diff_ = 0.01;
    std::shared_ptr<ORB_SLAM3::System> orb_slam3_ptr_;
    ORB_SLAM3::System::eSensor orb_slam3_sensor_type_;
};

#endif //SNAKE_SLAM_NODE_H
