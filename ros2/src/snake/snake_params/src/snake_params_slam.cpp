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

#include "snake_params/snake_params_slam.h"

namespace snake {
    snake::SnakeParamsSLAM::SnakeParamsSLAM() {

    }

    SnakeParamsSLAM::SnakeParamsSLAM(const SnakeParamsSLAM &params) {
        frame_id_world_val = params.frame_id_world_val;
        frame_id_base_val = params.frame_id_base_val;
        frame_id_camera_val = params.frame_id_camera_val;
        topic_camera_pose_val = params.topic_camera_pose_val;
        topic_map_points_val = params.topic_map_points_val;
        topic_imu_val = params.topic_imu_val;
        topic_img_mono_val = params.topic_img_mono_val;
        topic_img_left_val = params.topic_img_left_val;
        topic_img_right_val = params.topic_img_right_val;
        topic_img_rgb_val = params.topic_img_rgb_val;
        topic_img_depth_val = params.topic_img_depth_val;
        orb_slam3_voc_file_val = params.orb_slam3_voc_file_val;
        orb_slam3_settings_file_val = params.orb_slam3_settings_file_val;
        orb_slam3_enable_pangolin_val = params.orb_slam3_enable_pangolin_val;

        use_sim_time_val = params.use_sim_time_val;
    }

    snake::SnakeParamsSLAM::~SnakeParamsSLAM() {
    }

    SnakeParamsSLAM &SnakeParamsSLAM::operator=(const SnakeParamsSLAM &params) {
        if (this == &params) {
            return *this;
        }

        frame_id_world_val = params.frame_id_world_val;
        frame_id_base_val = params.frame_id_base_val;
        frame_id_camera_val = params.frame_id_camera_val;
        topic_camera_pose_val = params.topic_camera_pose_val;
        topic_map_points_val = params.topic_map_points_val;
        topic_imu_val = params.topic_imu_val;
        topic_img_mono_val = params.topic_img_mono_val;
        topic_img_left_val = params.topic_img_left_val;
        topic_img_right_val = params.topic_img_right_val;
        topic_img_rgb_val = params.topic_img_rgb_val;
        topic_img_depth_val = params.topic_img_depth_val;
        orb_slam3_voc_file_val = params.orb_slam3_voc_file_val;
        orb_slam3_settings_file_val = params.orb_slam3_settings_file_val;
        orb_slam3_enable_pangolin_val = params.orb_slam3_enable_pangolin_val;

        use_sim_time_val = params.use_sim_time_val;

        return *this;
    }

    std::ostream &operator<<(std::ostream &os, SnakeParamsSLAM &params) {
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
        os << params.frame_id_world_str << "\t" << params.frame_id_world_val << std::endl;
        os << params.frame_id_base_str << "\t" << params.frame_id_base_val << std::endl;
        os << params.frame_id_camera_str << "\t" << params.frame_id_camera_val << std::endl;
        os << params.topic_camera_pose_str << "\t" << params.topic_camera_pose_val << std::endl;
        os << params.topic_map_points_str << "\t" << params.topic_map_points_val << std::endl;
        os << params.topic_imu_str << "\t" << params.topic_imu_val << std::endl;
        os << params.topic_img_mono_str << "\t" << params.topic_img_mono_val << std::endl;
        os << params.topic_img_left_str << "\t" << params.topic_img_left_val << std::endl;
        os << params.topic_img_right_str << "\t" << params.topic_img_right_val << std::endl;
        os << params.topic_img_rgb_str << "\t" << params.topic_img_rgb_val << std::endl;
        os << params.topic_img_depth_str << "\t" << params.topic_img_depth_val << std::endl;
        os << params.orb_slam3_voc_file_str << "\t" << params.orb_slam3_voc_file_val << std::endl;
        os << params.orb_slam3_settings_file_str << "\t" << params.orb_slam3_settings_file_val << std::endl;
        os << params.orb_slam3_enable_pangolin_str << "\t" << params.orb_slam3_enable_pangolin_val << std::endl;
        os << params.use_sim_time_str << "\t" << params.use_sim_time_val << std::endl;
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;

        return os;
    }
}
