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

#include "snake_params/snake_params_heart.h"

namespace snake {
    SnakeParamsHeart::SnakeParamsHeart() {

    }

    SnakeParamsHeart::SnakeParamsHeart(const SnakeParamsHeart &params) {
        heart_rate_val = params.heart_rate_val;
        servo_idn_val = params.servo_idn_val;
        link_length_val = params.link_length_val;

        use_sim_time_val = params.use_sim_time_val;
    }

    SnakeParamsHeart::~SnakeParamsHeart() {
    }

    SnakeParamsHeart &SnakeParamsHeart::operator=(const SnakeParamsHeart &params) {
        if (this == &params) {
            return *this;
        }

        heart_rate_val = params.heart_rate_val;
        servo_idn_val = params.servo_idn_val;
        link_length_val = params.link_length_val;

        use_sim_time_val = params.use_sim_time_val;

        return *this;
    }

    std::ostream &operator<<(std::ostream &os, SnakeParamsHeart &params) {
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
        os << params.heart_rate_str << "\t" << params.heart_rate_val << std::endl;
        os << params.servo_idn_str << "\t" << params.servo_idn_val << std::endl;
        os << params.link_length_str << "\t" << params.link_length_val << std::endl;
        os << params.use_sim_time_str << "\t" << params.use_sim_time_val << std::endl;
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;

        return os;
    }
}
