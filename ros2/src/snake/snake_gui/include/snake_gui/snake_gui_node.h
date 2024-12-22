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

#ifndef SNAKE_GUI_NODE_H
#define SNAKE_GUI_NODE_H

#include <iostream>
#include <string>
#include <utility>

#include <QDebug>

#include <rclcpp/rclcpp.hpp>
#include <snake_params/snake_params_heart.h>
#include <snake_params/snake_params_cpg_hopf.h>

namespace snake {
    class SnakeGUINode : public rclcpp::Node {
    public:
        SnakeGUINode() = delete;

        explicit SnakeGUINode(std::string name);

        ~SnakeGUINode() override;

    public:
        std::shared_ptr<rclcpp::AsyncParametersClient> client_heart_;
        std::shared_ptr<rclcpp::AsyncParametersClient> client_cpg_hopf_;
        std::shared_future<std::vector<rclcpp::Parameter>> results_get_heart_;
        std::shared_future<std::vector<rclcpp::Parameter>> results_get_cpg_hopf_;
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> results_set_heart_;
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> results_set_cpg_hopf_;

        const std::string node_heart_str_ = "snake_heart";
        const std::string node_cpg_hopf_str_ = "snake_cpg_hopf";
        std::shared_ptr<snake::SnakeParamsHeart> params_heart_;
        std::shared_ptr<snake::SnakeParamsCPGHopf> params_cpg_hopf_;
    };
}

#endif //SNAKE_GUI_NODE_H
