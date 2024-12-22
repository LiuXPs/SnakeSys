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

#include "snake_params/snake_params_servo.h"

namespace snake {
    snake::SnakeParamsServo::SnakeParamsServo() {

    }

    SnakeParamsServo::SnakeParamsServo(const SnakeParamsServo &params) {
        port_val = params.port_val;
        rate_val = params.rate_val;
        timeout_val = params.timeout_val;

        use_sim_time_val = params.use_sim_time_val;
    }

    SnakeParamsServo::~SnakeParamsServo() {
    }

    SnakeParamsServo &SnakeParamsServo::operator=(const SnakeParamsServo &params) {
        if (this == &params) {
            return *this;
        }

        port_val = params.port_val;
        rate_val = params.rate_val;
        timeout_val = params.timeout_val;

        use_sim_time_val = params.use_sim_time_val;

        return *this;
    }

    std::ostream &operator<<(std::ostream &os, SnakeParamsServo &params) {
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
        os << params.port_str << "\t" << params.port_val << std::endl;
        os << params.rate_str << "\t" << params.rate_val << std::endl;
        os << params.timeout_str << "\t" << params.timeout_val << std::endl;
        os << params.use_sim_time_str << "\t" << params.use_sim_time_val << std::endl;
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;

        return os;
    }
}
