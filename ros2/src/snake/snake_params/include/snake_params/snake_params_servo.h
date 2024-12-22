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

#ifndef SNAKE_PARAMS_SERVO_H
#define SNAKE_PARAMS_SERVO_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

namespace snake {
    class SnakeParamsServo {
    public:
        SnakeParamsServo();

        SnakeParamsServo(const SnakeParamsServo &params);

        virtual ~SnakeParamsServo();

        SnakeParamsServo &operator=(const SnakeParamsServo &params);

        friend std::ostream &operator<<(std::ostream &os, SnakeParamsServo &params);

    public:
        const std::string port_str = "serial_port";
        const std::string rate_str = "baud_rate";
        const std::string timeout_str = "timeout";

        std::string port_val;
        int rate_val;
        int timeout_val;

    public:
        const std::string use_sim_time_str = "use_sim_time";
        bool use_sim_time_val;
    };
}

#endif //BUILD_SNAKE_PARAMS_SERVO_H
