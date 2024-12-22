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

#include "snake_frame/snake_frame.h"

namespace snake {

    SnakeFrame::SnakeFrame() {

    }

    SnakeFrame::~SnakeFrame() {

    }

    void SnakeFrame::calRelFrame(const std::vector<double> &theta) {
        for (int i = 0; i < link_n_; i++) {
            theta_.at(i) = theta.at(i);
        }

        for (int i = 0; i < link_n_; i++) {
            // Calculate the four-parameter transformation matrix.
            Eigen::Matrix3d rotx(Eigen::AngleAxisd(alpha_.at(i), Eigen::Vector3d(1, 0, 0)));
            Eigen::Vector3d transx(a_.at(i), 0, 0);
            Eigen::Matrix3d rotz(Eigen::AngleAxisd(theta_.at(i), Eigen::Vector3d(0, 0, 1)));
            Eigen::Vector3d transz(0, 0, d_.at(i));

            Sophus::SE3<double> Rt_1(rotx, Eigen::Vector3d::Zero());
            Sophus::SE3<double> Rt_2(Eigen::Matrix3d::Identity(), transx);
            Sophus::SE3<double> Rt_3(rotz, Eigen::Vector3d::Zero());
            Sophus::SE3<double> Rt_4(Eigen::Matrix3d::Identity(), transz);

            // Calculate the relative transformation matrix.
            Sophus::SE3<double> Rt = Rt_1 * Rt_2 * Rt_3 * Rt_4;
            frame_rel_.at(i) = Rt;
        }

        // Calculate the relative to absolute temporary transformation matrix.
        for (int i = 0; i < link_n_; i++) {
            if (i == 0) {
                frame_tmp_.at(i) = frame_rel_.at(i);
            } else {
                frame_tmp_.at(i) = frame_tmp_.at(i - 1) * frame_rel_.at(i);
            }
        }
    }

    void SnakeFrame::calAbsFrame(const Sophus::SE3<double> &base_frame) {
        base_frame_ = base_frame;

        // Calculate the absolute transformation matrix.
        for (int i = 0; i < link_n_; i++) {
            frame_abs_.at(i) = base_frame_ * frame_tmp_.at(i);
        }

        calGlobalPos();
    }

    void SnakeFrame::calLocalPos() {
        for (int i = 0; i < link_n_; i++) {
            joint_local_.at(i) = Eigen::Vector4d(0, 0, 0, 1);
            if (i != (link_n_ - 1)) {
                centr_local_.at(i) = Eigen::Vector4d(a_.at(i + 1) / 2, 0, 0, 1);
            } else {
                centr_local_.at(i) = Eigen::Vector4d(a_.at(i) / 2, 0, 0, 1);
            }
        }
    }

    void SnakeFrame::calGlobalPos() {
        for (int i = 0; i < link_n_; i++) {
            joint_global_.at(i) = frame_abs_.at(i) * joint_local_.at(i);
            centr_global_.at(i) = frame_abs_.at(i) * centr_local_.at(i);
        }
    }

    void SnakeFrame::initFrame(const Sophus::SE3<double> &base_frame, const std::vector<double> &alpha,
                               const std::vector<double> &a, const std::vector<double> &theta,
                               const std::vector<double> &d) {
        link_n_ = static_cast<int>(theta.size());
        assert(link_n_ == static_cast<int>(alpha.size()) &&
               link_n_ == static_cast<int>(a.size()) &&
               link_n_ == static_cast<int>(theta.size()) &&
               link_n_ == static_cast<int>(d.size()));

        alpha_.clear();
        a_.clear();
        d_.clear();
        theta_.clear();
        frame_rel_.clear();
        frame_tmp_.clear();
        frame_abs_.clear();
        joint_local_.clear();
        centr_local_.clear();
        joint_global_.clear();
        centr_global_.clear();

        alpha_.insert(alpha_.begin(), alpha.begin(), alpha.end());
        a_.insert(a_.begin(), a.begin(), a.end());
        d_.insert(d_.begin(), d.begin(), d.end());
        theta_.resize(link_n_);
        frame_rel_.resize(link_n_);
        frame_tmp_.resize(link_n_);
        frame_abs_.resize(link_n_);
        joint_local_.resize(link_n_);
        centr_local_.resize(link_n_);
        joint_global_.resize(link_n_);
        centr_global_.resize(link_n_);

        calLocalPos();
        calRelFrame(theta);
        calAbsFrame(base_frame);
    }

    void SnakeFrame::updateFrame(const Sophus::SE3<double> &base_frame, const std::vector<double> &theta) {
        // First update the relative transformation matrix.
        calRelFrame(theta);

        // Then update the absolute transformation matrix.
        calAbsFrame(base_frame);
    }

    std::vector<Sophus::SE3<double>> SnakeFrame::getRelFrame() const {
        return frame_rel_;
    }

    std::vector<Sophus::SE3<double>> SnakeFrame::getAbsFrame() const {
        return frame_abs_;
    }

    std::vector<Eigen::Vector4d> SnakeFrame::getGlobalJoint() const {
        return joint_global_;
    }

    std::vector<Eigen::Vector4d> SnakeFrame::getGlobalCentroid() const {
        return centr_global_;
    }
}