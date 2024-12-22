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

#include "snake_slam/snake_slam_node.h"

SnakeSLAMNode::SnakeSLAMNode(std::string name, SnakeSLAMNode::SLAMSystem system, SnakeSLAMNode::SLAMType type)
        : rclcpp::Node(name) {
    params_slam_ = std::make_shared<snake::SnakeParamsSLAM>();
    slam_system_ = system;
    slam_type_ = type;

    it_ = std::make_shared<image_transport::ImageTransport>(static_cast<SharedPtr>(this));

    tf2_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    initSystem();

    pub_camera_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(params_slam_->topic_camera_pose_val,
                                                                               rclcpp::QoS(queue_size_));
    pub_map_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_slam_->topic_map_points_val,
                                                                        rclcpp::QoS(queue_size_));

    trans_c0_w_.rotationMatrix() = Eigen::Matrix3f(Eigen::AngleAxisf(0, Eigen::Vector3f(1, 0, 0)));
    trans_c0_w_.translation() = Eigen::Vector3f::Zero();
}

SnakeSLAMNode::~SnakeSLAMNode() {
    // Stop all threads
    orb_slam3_ptr_->Shutdown();

    // Save camera trajectory
    orb_slam3_ptr_->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    orb_slam3_ptr_->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
}

double SnakeSLAMNode::toSec(const std_msgs::msg::Header &header) {
    return (header.stamp.sec + header.stamp.nanosec * 1e-9);
}

void SnakeSLAMNode::initSystem() {
    // Snake params.
    declare_parameter<std::string>(params_slam_->frame_id_world_str, params_slam_->frame_id_world_val);
    declare_parameter<std::string>(params_slam_->frame_id_base_str, params_slam_->frame_id_base_val);
    declare_parameter<std::string>(params_slam_->frame_id_camera_str, params_slam_->frame_id_camera_val);
    get_parameter(params_slam_->frame_id_world_str, params_slam_->frame_id_world_val);
    get_parameter(params_slam_->frame_id_base_str, params_slam_->frame_id_base_val);
    get_parameter(params_slam_->frame_id_camera_str, params_slam_->frame_id_camera_val);

    declare_parameter<std::string>(params_slam_->topic_camera_pose_str, params_slam_->topic_camera_pose_val);
    declare_parameter<std::string>(params_slam_->topic_map_points_str, params_slam_->topic_map_points_val);
    get_parameter(params_slam_->topic_camera_pose_str, params_slam_->topic_camera_pose_val);
    get_parameter(params_slam_->topic_map_points_str, params_slam_->topic_map_points_val);

    // Camera and IMU topics.
    declare_parameter<std::string>(params_slam_->topic_imu_str, params_slam_->topic_imu_val);
    declare_parameter<std::string>(params_slam_->topic_img_mono_str, params_slam_->topic_img_mono_val);
    declare_parameter<std::string>(params_slam_->topic_img_left_str, params_slam_->topic_img_left_val);
    declare_parameter<std::string>(params_slam_->topic_img_right_str, params_slam_->topic_img_right_val);
    declare_parameter<std::string>(params_slam_->topic_img_rgb_str, params_slam_->topic_img_rgb_val);
    declare_parameter<std::string>(params_slam_->topic_img_depth_str, params_slam_->topic_img_depth_val);
    get_parameter(params_slam_->topic_imu_str, params_slam_->topic_imu_val);
    get_parameter(params_slam_->topic_img_mono_str, params_slam_->topic_img_mono_val);
    get_parameter(params_slam_->topic_img_left_str, params_slam_->topic_img_left_val);
    get_parameter(params_slam_->topic_img_right_str, params_slam_->topic_img_right_val);
    get_parameter(params_slam_->topic_img_rgb_str, params_slam_->topic_img_rgb_val);
    get_parameter(params_slam_->topic_img_depth_str, params_slam_->topic_img_depth_val);

    // ORB_SLAM3 params.
    declare_parameter<std::string>(params_slam_->orb_slam3_voc_file_str,
                                   params_slam_->orb_slam3_voc_file_val);
    declare_parameter<std::string>(params_slam_->orb_slam3_settings_file_str,
                                   params_slam_->orb_slam3_settings_file_val);
    declare_parameter<bool>(params_slam_->orb_slam3_enable_pangolin_str,
                            params_slam_->orb_slam3_enable_pangolin_val);
    get_parameter(params_slam_->orb_slam3_voc_file_str, params_slam_->orb_slam3_voc_file_val);
    get_parameter(params_slam_->orb_slam3_settings_file_str, params_slam_->orb_slam3_settings_file_val);
    get_parameter(params_slam_->orb_slam3_enable_pangolin_str, params_slam_->orb_slam3_enable_pangolin_val);

    if (slam_type_ == SnakeSLAMNode::SLAMType::MONO) {
        orb_slam3_sensor_type_ = ORB_SLAM3::System::MONOCULAR;
#ifdef IMAGE_RAW
        sub_mono_ = it_->subscribe(params_slam_->topic_img_mono_val, queue_size_,
                                   &SnakeSLAMNode::grabMono, this);
#else
        auto bind_mono = std::bind(&SnakeSLAMNode::grabMonoComp, this, std::placeholders::_1);
        sub_mono_comp_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                params_slam_->topic_img_mono_val + "/compressed",
                rclcpp::QoS(queue_size_),
                bind_mono);
#endif

    } else if (slam_type_ == SnakeSLAMNode::SLAMType::MONO_IMU) {
        orb_slam3_sensor_type_ = ORB_SLAM3::System::IMU_MONOCULAR;
#ifdef IMAGE_RAW
        sub_mono_ = it_->subscribe(params_slam_->topic_img_mono_val, queue_size_,
                                   &SnakeSLAMNode::grabMonoIMU, this);
        auto bind_imu = std::bind(&SnakeSLAMNode::grabImu, this, std::placeholders::_1);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(params_slam_->topic_imu_val,
                                                                    rclcpp::QoS(queue_size_),
                                                                    bind_imu);
#else
        auto bind_mono = std::bind(&SnakeSLAMNode::grabMonoIMUComp, this, std::placeholders::_1);
        sub_mono_comp_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                params_slam_->topic_img_mono_val + "/compressed",
                rclcpp::QoS(queue_size_),
                bind_mono);
        auto bind_imu = std::bind(&SnakeSLAMNode::grabImu, this, std::placeholders::_1);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                params_slam_->topic_imu_val,
                rclcpp::QoS(queue_size_),
                bind_imu);
#endif

    } else if (slam_type_ == SnakeSLAMNode::SLAMType::STEREO) {
        orb_slam3_sensor_type_ = ORB_SLAM3::System::STEREO;
#ifdef IMAGE_RAW
        sub_left_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_left_val);
        sub_right_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_right_val);
        sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *sub_left_, *sub_right_);
        sync_->registerCallback(&SnakeSLAMNode::grabStereo, this);
#else
        sub_left_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_left_val + "/compressed");
        sub_right_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_right_val + "/compressed");
        sync_comp_ = std::make_shared<message_filters::Synchronizer<sync_pol_comp>>(
                sync_pol_comp(10),
                *sub_left_comp_,
                *sub_right_comp_);
        sync_comp_->registerCallback(&SnakeSLAMNode::grabStereoComp, this);
#endif

    } else if (slam_type_ == SnakeSLAMNode::SLAMType::STEREO_IMU) {
        orb_slam3_sensor_type_ = ORB_SLAM3::System::IMU_STEREO;
#ifdef IMAGE_RAW
        sub_left_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_left_val);
        sub_right_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_right_val);
        sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *sub_left_, *sub_right_);
        sync_->registerCallback(&SnakeSLAMNode::grabStereoIMU, this);
        auto bind_imu = std::bind(&SnakeSLAMNode::grabImu, this, std::placeholders::_1);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                params_slam_->topic_imu_val, rclcpp::QoS(queue_size_), bind_imu);
#else
        sub_left_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_left_val + "/compressed");
        sub_right_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_right_val + "/compressed");
        sync_comp_ = std::make_shared<message_filters::Synchronizer<sync_pol_comp>>(
                sync_pol_comp(10),
                *sub_left_comp_,
                *sub_right_comp_);
        sync_comp_->registerCallback(&SnakeSLAMNode::grabStereoIMUComp, this);
        auto bind_imu = std::bind(&SnakeSLAMNode::grabImu, this, std::placeholders::_1);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                params_slam_->topic_imu_val, rclcpp::QoS(queue_size_), bind_imu);
#endif

    } else if (slam_type_ == SnakeSLAMNode::SLAMType::RGBD) {
        orb_slam3_sensor_type_ = ORB_SLAM3::System::RGBD;
#ifdef IMAGE_RAW
        sub_rgb_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_rgb_val);
        sub_depth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_depth_val);
        sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *sub_rgb_, *sub_depth_);
        sync_->registerCallback(&SnakeSLAMNode::grabRGBD, this);
#else
        sub_rgb_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_rgb_val + "/compressed");
        sub_depth_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_depth_val + "/compressed");
        sync_comp_ = std::make_shared<message_filters::Synchronizer<sync_pol_comp>>(
                sync_pol_comp(10),
                *sub_rgb_comp_,
                *sub_depth_comp_);
        sync_comp_->registerCallback(&SnakeSLAMNode::grabRGBDComp, this);
#endif

    } else if (slam_type_ == SnakeSLAMNode::SLAMType::RGBD_IMU) {
        orb_slam3_sensor_type_ = ORB_SLAM3::System::IMU_RGBD;
#ifdef IMAGE_RAW
        sub_rgb_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_rgb_val);
        sub_depth_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, params_slam_->topic_img_depth_val);
        sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *sub_rgb_, *sub_depth_);
        sync_->registerCallback(&SnakeSLAMNode::grabRGBDIMU, this);
        auto bind_imu = std::bind(&SnakeSLAMNode::grabImu, this, std::placeholders::_1);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                params_slam_->topic_imu_val, rclcpp::QoS(queue_size_), bind_imu);
#else
        sub_rgb_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_rgb_val + "/compressed");
        sub_depth_comp_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
                this, params_slam_->topic_img_depth_val + "/compressed");
        sync_comp_ = std::make_shared<message_filters::Synchronizer<sync_pol_comp>>(
                sync_pol_comp(10),
                *sub_rgb_comp_,
                *sub_depth_comp_);
        sync_comp_->registerCallback(&SnakeSLAMNode::grabRGBDIMUComp, this);
        auto bind_imu = std::bind(&SnakeSLAMNode::grabImu, this, std::placeholders::_1);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                params_slam_->topic_imu_val, rclcpp::QoS(queue_size_), bind_imu);
#endif

    } else {
        std::cout << "Undefined SLAM system type ..." << std::endl;
    }

    orb_slam3_ptr_ = std::make_shared<ORB_SLAM3::System>(params_slam_->orb_slam3_voc_file_val,
                                                         params_slam_->orb_slam3_settings_file_val,
                                                         orb_slam3_sensor_type_,
                                                         params_slam_->orb_slam3_enable_pangolin_val);
}

void SnakeSLAMNode::pubCameraPose(const Sophus::SE3f &trans_w_c, const rclcpp::Time &msg_time) {
    msg_camera_pose_.header.frame_id = params_slam_->frame_id_world_val;
    msg_camera_pose_.header.stamp = msg_time;

    msg_camera_pose_.pose.position.x = trans_w_c.translation().x();
    msg_camera_pose_.pose.position.y = trans_w_c.translation().y();
    msg_camera_pose_.pose.position.z = trans_w_c.translation().z();

    msg_camera_pose_.pose.orientation.x = trans_w_c.unit_quaternion().coeffs().x();
    msg_camera_pose_.pose.orientation.y = trans_w_c.unit_quaternion().coeffs().y();
    msg_camera_pose_.pose.orientation.z = trans_w_c.unit_quaternion().coeffs().z();
    msg_camera_pose_.pose.orientation.w = trans_w_c.unit_quaternion().coeffs().w();

    pub_camera_pose_->publish(msg_camera_pose_);
}

void
SnakeSLAMNode::pubTrackedPoints(const std::vector<ORB_SLAM3::MapPoint *> &map_points, const rclcpp::Time &msg_time) {
    msg_map_points_ = toPointCloud(map_points, msg_time);
    pub_map_pc_->publish(msg_map_points_);
}

void SnakeSLAMNode::pubBaseTF(const Sophus::SE3f &trans_w_c, const rclcpp::Time &msg_time) {
    // Get the transformation from snake_base to snake_camera_1.
    try {
        msg_base_camera_ = tf2_buffer_->lookupTransform(params_slam_->frame_id_base_val,
                                                        params_slam_->frame_id_camera_val,
                                                        tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
    Eigen::Quaterniond q_pose1;
    q_pose1.w() = msg_base_camera_.transform.rotation.w;
    q_pose1.x() = msg_base_camera_.transform.rotation.x;
    q_pose1.y() = msg_base_camera_.transform.rotation.y;
    q_pose1.z() = msg_base_camera_.transform.rotation.z;
    q_pose1 = q_pose1.normalized();
    Eigen::Vector3d t_pose1(msg_base_camera_.transform.translation.x,
                            msg_base_camera_.transform.translation.y,
                            msg_base_camera_.transform.translation.z);
    frame_base_camera_.setRotationMatrix(Sophus::SE3<double>(q_pose1, t_pose1).rotationMatrix());

    // Calculate the transformation from snake_world to snake_base.
    Eigen::Quaterniond q_pose2;
    q_pose2.w() = trans_w_c.unit_quaternion().coeffs().w();
    q_pose2.x() = trans_w_c.unit_quaternion().coeffs().x();
    q_pose2.y() = trans_w_c.unit_quaternion().coeffs().y();
    q_pose2.z() = trans_w_c.unit_quaternion().coeffs().z();
    q_pose2 = q_pose2.normalized();
    Eigen::Vector3d t_pose2(trans_w_c.translation().x(),
                            trans_w_c.translation().y(),
                            trans_w_c.translation().z());
    Sophus::SE3<double> trans_w_b = Sophus::SE3<double>(q_pose2, t_pose2) * frame_base_camera_.inverse();


    // Publish the transformation from snake_world to snake_base.
    msg_base_frame_.header.stamp = msg_time;
    msg_base_frame_.header.frame_id = params_slam_->frame_id_world_val;
    msg_base_frame_.child_frame_id = params_slam_->frame_id_base_val;

    msg_base_frame_.transform.rotation.x = trans_w_b.unit_quaternion().coeffs().x();
    msg_base_frame_.transform.rotation.y = trans_w_b.unit_quaternion().coeffs().y();
    msg_base_frame_.transform.rotation.z = trans_w_b.unit_quaternion().coeffs().z();
    msg_base_frame_.transform.rotation.w = trans_w_b.unit_quaternion().coeffs().w();

    msg_base_frame_.transform.translation.x = trans_w_b.translation().x();
    msg_base_frame_.transform.translation.y = trans_w_b.translation().y();
    msg_base_frame_.transform.translation.z = trans_w_b.translation().z();

    tf2_broadcaster_->sendTransform(msg_base_frame_);
}

sensor_msgs::msg::PointCloud2 SnakeSLAMNode::toPointCloud(const std::vector<ORB_SLAM3::MapPoint *> &map_points,
                                                          const rclcpp::Time &msg_time) {
    const int num_channels = 3; // x y z.

    if (map_points.empty()) {
        std::cout << "Map points vector is empty!" << std::endl;
//        exit(1);
    }

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = params_slam_->frame_id_world_val;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points[i]) {
            // Original data.
            Eigen::Vector3f pMPw = map_points[i]->GetWorldPos();
//            ORB_SLAM3::Map *maps = map_points[i]->GetMap();
//            maps->GetAllMapPoints();

            // Apply world frame orientation for non-IMU cases.
            if (orb_slam3_sensor_type_ == ORB_SLAM3::System::MONOCULAR ||
                orb_slam3_sensor_type_ == ORB_SLAM3::System::STEREO) {
                Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), pMPw);
                Sophus::SE3f Twmp = trans_c0_w_.inverse() * Tc0mp;
                pMPw = Twmp.translation();
            }

            tf2::Vector3 point_translation(pMPw.x(), pMPw.y(), pMPw.z());
            float data_array[num_channels] = {static_cast<float>(point_translation.x()),
                                              static_cast<float>(point_translation.y()),
                                              static_cast<float>(point_translation.z())};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}

cv::Mat SnakeSLAMNode::getImage(const sensor_msgs::msg::Image::SharedPtr msg_img, const std::string encode) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cb_img_ptr;
    try {
        cb_img_ptr = cv_bridge::toCvShare(msg_img, encode);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//        std::exit(1);
    }

    if (cb_img_ptr->image.type() == 0) {
        return cb_img_ptr->image.clone();
    } else {
        std::cerr << "Error image type" << std::endl;
        return cb_img_ptr->image.clone();
    }
}

void SnakeSLAMNode::grabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    mutex_imu_.lock();
    buff_imu_.push(imu_msg);
    mutex_imu_.unlock();
}

void SnakeSLAMNode::grabMono(const sensor_msgs::msg::Image::ConstSharedPtr &msg_mono) {
    try {
        cb_mono_ptr_ = cv_bridge::toCvShare(msg_mono, msg_mono->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    t_mono_img_ = toSec(cb_mono_ptr_->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        // ORB-SLAM3 runs in TrackMonocular()
        Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackMonocular(cb_mono_ptr_->image, t_mono_img_);
        Sophus::SE3f trans_w_c = trans_c_w.inverse();
        rclcpp::Time msg_time = cb_mono_ptr_->header.stamp;
        pubCameraPose(trans_w_c, msg_time);
        pubBaseTF(trans_w_c, msg_time);
        pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
    }
}

void SnakeSLAMNode::grabMonoComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_mono) {
    cv::imdecode(cv::Mat(msg_mono->data), cv::IMREAD_COLOR, &cv_mono_img_);

    t_mono_img_ = toSec(msg_mono->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        // ORB-SLAM3 runs in TrackMonocular()
        Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackMonocular(cv_mono_img_, t_mono_img_);
        Sophus::SE3f trans_w_c = trans_c_w.inverse();
        rclcpp::Time msg_time = msg_mono->header.stamp;
        pubCameraPose(trans_w_c, msg_time);
        pubBaseTF(trans_w_c, msg_time);
        pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
    }
}

void SnakeSLAMNode::grabMonoIMU(const sensor_msgs::msg::Image::ConstSharedPtr &msg_mono) {
    try {
        cb_mono_ptr_ = cv_bridge::toCvShare(msg_mono, msg_mono->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    t_mono_img_ = toSec(cb_mono_ptr_->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        vector<ORB_SLAM3::IMU::Point> imu_meas_vec;

        if (!buff_imu_.empty()) {
            imu_meas_vec.clear();

            // Load imu measurements from buffer.
            mutex_imu_.lock();
            while (!buff_imu_.empty() && toSec(buff_imu_.front()->header) <= t_mono_img_) {
                double t = toSec(buff_imu_.front()->header);

                cv::Point3f acc(buff_imu_.front()->linear_acceleration.x,
                                buff_imu_.front()->linear_acceleration.y,
                                buff_imu_.front()->linear_acceleration.z);

                cv::Point3f gyr(buff_imu_.front()->angular_velocity.x,
                                buff_imu_.front()->angular_velocity.y,
                                buff_imu_.front()->angular_velocity.z);

                imu_meas_vec.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                buff_imu_.pop();
            }
            mutex_imu_.unlock();
        }

        if (!imu_meas_vec.empty()) {
            // ORB-SLAM3 runs in TrackMonocular()
            Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackMonocular(cb_mono_ptr_->image, t_mono_img_, imu_meas_vec);
            Sophus::SE3f trans_w_c = trans_c_w.inverse();
            rclcpp::Time msg_time = cb_mono_ptr_->header.stamp;
            pubCameraPose(trans_w_c, msg_time);
            pubBaseTF(trans_w_c, msg_time);
            pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
        }
    }
}

void SnakeSLAMNode::grabMonoIMUComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_mono) {
    cv::imdecode(cv::Mat(msg_mono->data), cv::IMREAD_COLOR, &cv_mono_img_);

    t_mono_img_ = toSec(msg_mono->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        vector<ORB_SLAM3::IMU::Point> imu_meas_vec;

        if (!buff_imu_.empty()) {
            imu_meas_vec.clear();

            // Load imu measurements from buffer.
            mutex_imu_.lock();
            while (!buff_imu_.empty() && toSec(buff_imu_.front()->header) <= t_mono_img_) {
                double t = toSec(buff_imu_.front()->header);

                cv::Point3f acc(buff_imu_.front()->linear_acceleration.x,
                                buff_imu_.front()->linear_acceleration.y,
                                buff_imu_.front()->linear_acceleration.z);

                cv::Point3f gyr(buff_imu_.front()->angular_velocity.x,
                                buff_imu_.front()->angular_velocity.y,
                                buff_imu_.front()->angular_velocity.z);

                imu_meas_vec.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                buff_imu_.pop();
            }
            mutex_imu_.unlock();
        }

        if (!imu_meas_vec.empty()) {
            // ORB-SLAM3 runs in TrackMonocular()
            Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackMonocular(cv_mono_img_, t_mono_img_, imu_meas_vec);
            Sophus::SE3f trans_w_c = trans_c_w.inverse();
            rclcpp::Time msg_time = msg_mono->header.stamp;
            pubCameraPose(trans_w_c, msg_time);
            pubBaseTF(trans_w_c, msg_time);
            pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
        }
    }
}

void SnakeSLAMNode::grabStereo(const sensor_msgs::msg::Image::ConstSharedPtr &msg_left,
                               const sensor_msgs::msg::Image::ConstSharedPtr &msg_right) {
    try {
        cb_left_ptr_ = cv_bridge::toCvShare(msg_left, msg_left->encoding);
        cb_right_ptr_ = cv_bridge::toCvShare(msg_right, msg_right->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    t_left_img_ = toSec(cb_left_ptr_->header);
    t_right_img_ = toSec(cb_right_ptr_->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        // ORB-SLAM3 runs in TrackStereo()
        Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackStereo(cb_left_ptr_->image, cb_right_ptr_->image, t_left_img_);
        Sophus::SE3f trans_w_c = trans_c_w.inverse();
        rclcpp::Time msg_time = cb_left_ptr_->header.stamp;
        pubCameraPose(trans_w_c, msg_time);
        pubBaseTF(trans_w_c, msg_time);
        pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
    }
}

void SnakeSLAMNode::grabStereoComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_left,
                                   const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_right) {
    cv::imdecode(cv::Mat(msg_left->data), cv::IMREAD_COLOR, &cv_left_img_);
    cv::imdecode(cv::Mat(msg_right->data), cv::IMREAD_COLOR, &cv_right_img_);

    t_left_img_ = toSec(msg_left->header);
    t_right_img_ = toSec(msg_right->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        // ORB-SLAM3 runs in TrackStereo()
        Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackStereo(cv_left_img_, cv_right_img_, t_left_img_);
        Sophus::SE3f trans_w_c = trans_c_w.inverse();
        rclcpp::Time msg_time = msg_left->header.stamp;
        pubCameraPose(trans_w_c, msg_time);
        pubBaseTF(trans_w_c, msg_time);
        pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
    }
}

void SnakeSLAMNode::grabStereoIMU(const sensor_msgs::msg::Image::ConstSharedPtr &msg_left,
                                  const sensor_msgs::msg::Image::ConstSharedPtr &msg_right) {
    try {
        cb_left_ptr_ = cv_bridge::toCvShare(msg_left, msg_left->encoding);
        cb_right_ptr_ = cv_bridge::toCvShare(msg_right, msg_right->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    t_left_img_ = toSec(cb_left_ptr_->header);
    t_right_img_ = toSec(cb_right_ptr_->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        vector<ORB_SLAM3::IMU::Point> imu_meas_vec;

        if (!buff_imu_.empty()) {
            imu_meas_vec.clear();

            // Load imu measurements from buffer.
            mutex_imu_.lock();
            while (!buff_imu_.empty() && toSec(buff_imu_.front()->header) <= t_left_img_) {
                double t = toSec(buff_imu_.front()->header);

                cv::Point3f acc(buff_imu_.front()->linear_acceleration.x,
                                buff_imu_.front()->linear_acceleration.y,
                                buff_imu_.front()->linear_acceleration.z);

                cv::Point3f gyr(buff_imu_.front()->angular_velocity.x,
                                buff_imu_.front()->angular_velocity.y,
                                buff_imu_.front()->angular_velocity.z);

                imu_meas_vec.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                buff_imu_.pop();
            }
            mutex_imu_.unlock();
        }

        if (!imu_meas_vec.empty()) {
            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackStereo(cb_left_ptr_->image, cb_right_ptr_->image, t_left_img_,
                                                                 imu_meas_vec);
            Sophus::SE3f trans_w_c = trans_c_w.inverse();
            rclcpp::Time msg_time = cb_left_ptr_->header.stamp;
            pubCameraPose(trans_w_c, msg_time);
            pubBaseTF(trans_w_c, msg_time);
            pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
        }
    }
}

void SnakeSLAMNode::grabStereoIMUComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_left,
                                      const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_right) {
    cv::imdecode(cv::Mat(msg_left->data), cv::IMREAD_COLOR, &cv_left_img_);
    cv::imdecode(cv::Mat(msg_right->data), cv::IMREAD_COLOR, &cv_right_img_);

    t_left_img_ = toSec(msg_left->header);
    t_right_img_ = toSec(msg_right->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        vector<ORB_SLAM3::IMU::Point> imu_meas_vec;

        if (!buff_imu_.empty()) {
            imu_meas_vec.clear();

            // Load imu measurements from buffer.
            mutex_imu_.lock();
            while (!buff_imu_.empty() && toSec(buff_imu_.front()->header) <= t_left_img_) {
                double t = toSec(buff_imu_.front()->header);

                cv::Point3f acc(buff_imu_.front()->linear_acceleration.x,
                                buff_imu_.front()->linear_acceleration.y,
                                buff_imu_.front()->linear_acceleration.z);

                cv::Point3f gyr(buff_imu_.front()->angular_velocity.x,
                                buff_imu_.front()->angular_velocity.y,
                                buff_imu_.front()->angular_velocity.z);

                imu_meas_vec.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                buff_imu_.pop();
            }
            mutex_imu_.unlock();
        }

        if (!imu_meas_vec.empty()) {
            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackStereo(cv_left_img_, cv_right_img_, t_left_img_,
                                                                 imu_meas_vec);
            Sophus::SE3f trans_w_c = trans_c_w.inverse();
            rclcpp::Time msg_time = msg_left->header.stamp;
            pubCameraPose(trans_w_c, msg_time);
            pubBaseTF(trans_w_c, msg_time);
            pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
        }
    }
}

void SnakeSLAMNode::grabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr &msg_rgb,
                             const sensor_msgs::msg::Image::ConstSharedPtr &msg_depth) {
    try {
        cb_rgb_ptr_ = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding);
        cb_depth_ptr_ = cv_bridge::toCvShare(msg_depth, msg_depth->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    t_rgb_img_ = toSec(cb_rgb_ptr_->header);
    t_depth_img_ = toSec(cb_depth_ptr_->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        // ORB-SLAM3 runs in TrackRGBD()
        Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackRGBD(cb_rgb_ptr_->image, cb_depth_ptr_->image, t_rgb_img_);
        Sophus::SE3f trans_w_c = trans_c_w.inverse();
        rclcpp::Time msg_time = cb_rgb_ptr_->header.stamp;
        pubCameraPose(trans_w_c, msg_time);
        pubBaseTF(trans_w_c, msg_time);
        pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
    }
}

void SnakeSLAMNode::grabRGBDComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_rgb,
                                 const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_depth) {
    cv::imdecode(cv::Mat(msg_rgb->data), cv::IMREAD_COLOR, &cv_rgb_img_);
    cv::imdecode(cv::Mat(msg_depth->data), cv::IMREAD_COLOR, &cv_depth_img_);

    t_rgb_img_ = toSec(msg_rgb->header);
    t_depth_img_ = toSec(msg_depth->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        // ORB-SLAM3 runs in TrackRGBD()
        Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackRGBD(cv_rgb_img_, cv_depth_img_, t_rgb_img_);
        Sophus::SE3f trans_w_c = trans_c_w.inverse();
        rclcpp::Time msg_time = msg_rgb->header.stamp;
        pubCameraPose(trans_w_c, msg_time);
        pubBaseTF(trans_w_c, msg_time);
        pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
    }
}

void SnakeSLAMNode::grabRGBDIMU(const sensor_msgs::msg::Image::ConstSharedPtr &msg_rgb,
                                const sensor_msgs::msg::Image::ConstSharedPtr &msg_depth) {
    try {
        cb_rgb_ptr_ = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding);
        cb_depth_ptr_ = cv_bridge::toCvShare(msg_depth, msg_depth->encoding);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    t_rgb_img_ = toSec(cb_rgb_ptr_->header);
    t_depth_img_ = toSec(cb_depth_ptr_->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        vector<ORB_SLAM3::IMU::Point> imu_meas_vec;

        if (!buff_imu_.empty()) {
            imu_meas_vec.clear();

            // Load imu measurements from buffer.
            mutex_imu_.lock();
            while (!buff_imu_.empty() && toSec(buff_imu_.front()->header) <= t_rgb_img_) {
                double t = toSec(buff_imu_.front()->header);

                cv::Point3f acc(buff_imu_.front()->linear_acceleration.x,
                                buff_imu_.front()->linear_acceleration.y,
                                buff_imu_.front()->linear_acceleration.z);

                cv::Point3f gyr(buff_imu_.front()->angular_velocity.x,
                                buff_imu_.front()->angular_velocity.y,
                                buff_imu_.front()->angular_velocity.z);

                imu_meas_vec.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                buff_imu_.pop();
            }
            mutex_imu_.unlock();
        }

        if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
            // ORB-SLAM3 runs in TrackRGBD()
            Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackRGBD(cb_rgb_ptr_->image, cb_depth_ptr_->image, t_rgb_img_,
                                                               imu_meas_vec);
            Sophus::SE3f trans_w_c = trans_c_w.inverse();
            rclcpp::Time msg_time = cb_rgb_ptr_->header.stamp;
            pubCameraPose(trans_w_c, msg_time);
            pubBaseTF(trans_w_c, msg_time);
            pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
        }
    }
}

void SnakeSLAMNode::grabRGBDIMUComp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_rgb,
                                    const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_depth) {
    cv::imdecode(cv::Mat(msg_rgb->data), cv::IMREAD_COLOR, &cv_rgb_img_);
    cv::imdecode(cv::Mat(msg_depth->data), cv::IMREAD_COLOR, &cv_depth_img_);

    t_rgb_img_ = toSec(msg_rgb->header);
    t_depth_img_ = toSec(msg_depth->header);

    if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
        vector<ORB_SLAM3::IMU::Point> imu_meas_vec;

        if (!buff_imu_.empty()) {
            imu_meas_vec.clear();

            // Load imu measurements from buffer.
            mutex_imu_.lock();
            while (!buff_imu_.empty() && toSec(buff_imu_.front()->header) <= t_rgb_img_) {
                double t = toSec(buff_imu_.front()->header);

                cv::Point3f acc(buff_imu_.front()->linear_acceleration.x,
                                buff_imu_.front()->linear_acceleration.y,
                                buff_imu_.front()->linear_acceleration.z);

                cv::Point3f gyr(buff_imu_.front()->angular_velocity.x,
                                buff_imu_.front()->angular_velocity.y,
                                buff_imu_.front()->angular_velocity.z);

                imu_meas_vec.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                buff_imu_.pop();
            }
            mutex_imu_.unlock();
        }

        if (slam_system_ == SnakeSLAMNode::SLAMSystem::ORB_SLAM3) {
            // ORB-SLAM3 runs in TrackRGBD()
            Sophus::SE3f trans_c_w = orb_slam3_ptr_->TrackRGBD(cv_rgb_img_, cv_depth_img_, t_rgb_img_,
                                                               imu_meas_vec);
            Sophus::SE3f trans_w_c = trans_c_w.inverse();
            rclcpp::Time msg_time = msg_rgb->header.stamp;
            pubCameraPose(trans_w_c, msg_time);
            pubBaseTF(trans_w_c, msg_time);
            pubTrackedPoints(orb_slam3_ptr_->GetTrackedMapPoints(), msg_time);
        }
    }
}
