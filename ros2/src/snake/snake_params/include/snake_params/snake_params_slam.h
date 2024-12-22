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

#ifndef SNAKE_PARAMS_SLAM_H
#define SNAKE_PARAMS_SLAM_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

namespace snake {
    class SnakeParamsSLAM {
    public:
        SnakeParamsSLAM();

        SnakeParamsSLAM(const SnakeParamsSLAM &params);

        virtual ~SnakeParamsSLAM();

        SnakeParamsSLAM &operator=(const SnakeParamsSLAM &params);

        friend std::ostream &operator<<(std::ostream &os, SnakeParamsSLAM &params);

    public:
        const std::string frame_id_world_str = "frame_id_world";
        const std::string frame_id_base_str = "frame_id_base";
        const std::string frame_id_camera_str = "frame_id_camera";
        std::string frame_id_world_val;
        std::string frame_id_base_val;
        std::string frame_id_camera_val;

        const std::string topic_camera_pose_str = "camera_pose";
        const std::string topic_map_points_str = "map_points";
        std::string topic_camera_pose_val;
        std::string topic_map_points_val;

        const std::string topic_imu_str = "topic_imu";
        const std::string topic_img_mono_str = "topic_img_mono";
        const std::string topic_img_left_str = "topic_img_left";
        const std::string topic_img_right_str = "topic_img_right";
        const std::string topic_img_rgb_str = "topic_img_rgb";
        const std::string topic_img_depth_str = "topic_img_depth";
        std::string topic_imu_val;
        std::string topic_img_mono_val;
        std::string topic_img_left_val;
        std::string topic_img_right_val;
        std::string topic_img_rgb_val;
        std::string topic_img_depth_val;

        // ORB_SLAM3.
        const std::string orb_slam3_voc_file_str = "orb_slam3_voc_file";
        const std::string orb_slam3_settings_file_str = "orb_slam3_settings_file";
        const std::string orb_slam3_enable_pangolin_str = "orb_slam3_enable_pangolin";
        std::string orb_slam3_voc_file_val;
        std::string orb_slam3_settings_file_val;
        bool orb_slam3_enable_pangolin_val;

    public:
        const std::string use_sim_time_str = "use_sim_time";
        bool use_sim_time_val;
    };
}

#endif //SNAKE_PARAMS_SLAM_H
