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

#ifndef SNAKE_FRAME_H
#define SNAKE_FRAME_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <rclcpp/rclcpp.hpp>

namespace snake {
    class SnakeFrame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SnakeFrame();

        virtual ~SnakeFrame();

    private:
        /*!
         * @brief Calculate the relative transformation matrix.
         * @param theta Unit: radian. Input joint Angle.
         */
        void calRelFrame(const std::vector<double> &theta);

        /*!
         * @brief Calculate the absolute transformation matrix.
         * @param base_frame Input base frame.
         */
        void calAbsFrame(const Sophus::SE3<double> &base_frame);

        /*!
         * @brief Calculate each joint position and centroid position in local coordinate system.
         */
        void calLocalPos();

        /*!
         * @brief Calculate each joint position and centroid position in global coordinate system.
         */
        void calGlobalPos();

    private:
        int link_n_;

        std::vector<double> alpha_;
        std::vector<double> a_;
        std::vector<double> theta_;
        std::vector<double> d_;

        Sophus::SE3<double> base_frame_;

        std::vector<Sophus::SE3<double>> frame_rel_;
        std::vector<Sophus::SE3<double>> frame_tmp_;
        std::vector<Sophus::SE3<double>> frame_abs_;

        std::vector<Eigen::Vector4d> joint_local_; //The joint position is at the origin of the coordinate system.
        std::vector<Eigen::Vector4d> joint_global_;
        std::vector<Eigen::Vector4d> centr_local_; //The centroid of each link is in the middle.
        std::vector<Eigen::Vector4d> centr_global_;

    public:
        /*!
         * @brief
         * @param base_frame
         * @param alpha
         * @param a
         * @param theta
         * @param d
         */
        void initFrame(const Sophus::SE3<double> &base_frame,
                       const std::vector<double> &alpha,
                       const std::vector<double> &a,
                       const std::vector<double> &theta,
                       const std::vector<double> &d);

        /*!
         * @brief Update the relative and absolute transformation matrix.
         * @param base_frame Input base frame.
         * @param theta Unit: radian. Input joint Angle.
         */
        void updateFrame(const Sophus::SE3<double> &base_frame,
                         const std::vector<double> &theta);

        std::vector<Sophus::SE3<double>> getRelFrame() const;

        std::vector<Sophus::SE3<double>> getAbsFrame() const;

        std::vector<Eigen::Vector4d> getGlobalJoint() const;

        std::vector<Eigen::Vector4d> getGlobalCentroid() const;
    };
}

#endif //SNAKE_FRAME_H
