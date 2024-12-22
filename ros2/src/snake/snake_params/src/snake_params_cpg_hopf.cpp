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

#include "snake_params/snake_params_cpg_hopf.h"

namespace snake {
    SnakeParamsCPGHopf::SnakeParamsCPGHopf() {

    }

    SnakeParamsCPGHopf::SnakeParamsCPGHopf(const SnakeParamsCPGHopf &params) {
        cpg_alpha_y_val = params.cpg_alpha_y_val;
        cpg_alpha_p_val = params.cpg_alpha_p_val;
        cpg_kg_y_val = params.cpg_kg_y_val;
        cpg_kg_p_val = params.cpg_kg_p_val;
        cpg_lambda_y_val = params.cpg_lambda_y_val;
        cpg_lambda_p_val = params.cpg_lambda_p_val;
        cpg_sigma_y_val = params.cpg_sigma_y_val;
        cpg_sigma_p_val = params.cpg_sigma_p_val;

        cpg_uvc_y_val = params.cpg_uvc_y_val;
        cpg_uvc_p_val = params.cpg_uvc_p_val;
        cpg_rho_y_val = params.cpg_rho_y_val;
        cpg_rho_p_val = params.cpg_rho_p_val;
        cpg_omega_y_val = params.cpg_omega_y_val;
        cpg_omega_p_val = params.cpg_omega_p_val;
        cpg_phi_y_val = params.cpg_phi_y_val;
        cpg_phi_p_val = params.cpg_phi_p_val;
        cpg_phi_yp_val = params.cpg_phi_yp_val;

        cpg_topo_mode_val = params.cpg_topo_mode_val;
        cpg_weight_mode_val = params.cpg_weight_mode_val;
        cpg_motion_mode_val = params.cpg_motion_mode_val;
        cpg_couple_mode_val = params.cpg_couple_mode_val;
        cpg_mode_val = params.cpg_mode_val;

        cpg_gauss_mu_val = params.cpg_gauss_mu_val;
        cpg_gauss_sigma_val = params.cpg_gauss_sigma_val;

        cpg_wave_kn_val = params.cpg_wave_kn_val;
        cpg_wave_ay_val = params.cpg_wave_ay_val;
        cpg_wave_ap_val = params.cpg_wave_ap_val;
        cpg_arc_r_val = params.cpg_arc_r_val;
        cpg_spiral_r_val = params.cpg_spiral_r_val;
        cpg_spiral_p_val = params.cpg_spiral_p_val;

        cpg_start_val = params.cpg_start_val;
        cpg_change_val = params.cpg_change_val;
        cpg_torque_val = params.cpg_torque_val;

        use_sim_time_val = params.use_sim_time_val;
    }

    SnakeParamsCPGHopf::~SnakeParamsCPGHopf() {
    }

    SnakeParamsCPGHopf &SnakeParamsCPGHopf::operator=(const SnakeParamsCPGHopf &params) {
        if (this == &params) {
            return *this;
        }

        cpg_alpha_y_val = params.cpg_alpha_y_val;
        cpg_alpha_p_val = params.cpg_alpha_p_val;
        cpg_kg_y_val = params.cpg_kg_y_val;
        cpg_kg_p_val = params.cpg_kg_p_val;
        cpg_lambda_y_val = params.cpg_lambda_y_val;
        cpg_lambda_p_val = params.cpg_lambda_p_val;
        cpg_sigma_y_val = params.cpg_sigma_y_val;
        cpg_sigma_p_val = params.cpg_sigma_p_val;

        cpg_uvc_y_val = params.cpg_uvc_y_val;
        cpg_uvc_p_val = params.cpg_uvc_p_val;
        cpg_rho_y_val = params.cpg_rho_y_val;
        cpg_rho_p_val = params.cpg_rho_p_val;
        cpg_omega_y_val = params.cpg_omega_y_val;
        cpg_omega_p_val = params.cpg_omega_p_val;
        cpg_phi_y_val = params.cpg_phi_y_val;
        cpg_phi_p_val = params.cpg_phi_p_val;
        cpg_phi_yp_val = params.cpg_phi_yp_val;

        cpg_topo_mode_val = params.cpg_topo_mode_val;
        cpg_weight_mode_val = params.cpg_weight_mode_val;
        cpg_motion_mode_val = params.cpg_motion_mode_val;
        cpg_couple_mode_val = params.cpg_couple_mode_val;
        cpg_mode_val = params.cpg_mode_val;

        cpg_gauss_mu_val = params.cpg_gauss_mu_val;
        cpg_gauss_sigma_val = params.cpg_gauss_sigma_val;

        cpg_wave_kn_val = params.cpg_wave_kn_val;
        cpg_wave_ay_val = params.cpg_wave_ay_val;
        cpg_wave_ap_val = params.cpg_wave_ap_val;
        cpg_arc_r_val = params.cpg_arc_r_val;
        cpg_spiral_r_val = params.cpg_spiral_r_val;
        cpg_spiral_p_val = params.cpg_spiral_p_val;

        cpg_start_val = params.cpg_start_val;
        cpg_change_val = params.cpg_change_val;
        cpg_torque_val = params.cpg_torque_val;

        use_sim_time_val = params.use_sim_time_val;

        return *this;
    }

    std::ostream &operator<<(std::ostream &os, SnakeParamsCPGHopf &params) {
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;
        os << params.cpg_alpha_y_str << "\t" << params.cpg_alpha_y_val << std::endl;
        os << params.cpg_alpha_p_str << "\t" << params.cpg_alpha_p_val << std::endl;
        os << params.cpg_kg_y_str << "\t" << params.cpg_kg_y_val << std::endl;
        os << params.cpg_kg_p_str << "\t" << params.cpg_kg_p_val << std::endl;
        os << params.cpg_lambda_y_str << "\t" << params.cpg_lambda_y_val << std::endl;
        os << params.cpg_lambda_p_str << "\t" << params.cpg_lambda_p_val << std::endl;
        os << params.cpg_sigma_y_str << "\t" << params.cpg_sigma_y_val << std::endl;
        os << params.cpg_sigma_p_str << "\t" << params.cpg_sigma_p_val << std::endl;

        os << params.cpg_uvc_y_str << "\t" << params.cpg_uvc_y_val << std::endl;
        os << params.cpg_uvc_p_str << "\t" << params.cpg_uvc_p_val << std::endl;
        os << params.cpg_rho_y_str << "\t" << params.cpg_rho_y_val << std::endl;
        os << params.cpg_rho_p_str << "\t" << params.cpg_rho_p_val << std::endl;
        os << params.cpg_omega_y_str << "\t" << params.cpg_omega_y_val << std::endl;
        os << params.cpg_omega_p_str << "\t" << params.cpg_omega_p_val << std::endl;
        os << params.cpg_phi_y_str << "\t" << params.cpg_phi_y_val << std::endl;
        os << params.cpg_phi_p_str << "\t" << params.cpg_phi_p_val << std::endl;
        os << params.cpg_phi_yp_str << "\t" << params.cpg_phi_yp_val << std::endl;

        os << params.cpg_topo_mode_str << "\t" << params.cpg_topo_mode_val << std::endl;
        os << params.cpg_weight_mode_str << "\t" << params.cpg_weight_mode_val << std::endl;
        os << params.cpg_motion_mode_str << "\t" << params.cpg_motion_mode_val << std::endl;
        os << params.cpg_couple_mode_str << "\t" << params.cpg_couple_mode_val << std::endl;
        os << params.cpg_mode_str << "\t" << params.cpg_mode_val << std::endl;

        os << params.cpg_gauss_mu_str << "\t" << params.cpg_gauss_mu_val << std::endl;
        os << params.cpg_gauss_sigma_str << "\t" << params.cpg_gauss_sigma_val << std::endl;

        os << params.cpg_wave_kn_str << "\t" << params.cpg_wave_kn_val << std::endl;
        os << params.cpg_wave_ay_str << "\t" << params.cpg_wave_ay_val << std::endl;
        os << params.cpg_wave_ap_str << "\t" << params.cpg_wave_ap_val << std::endl;
        os << params.cpg_arc_r_str << "\t" << params.cpg_arc_r_val << std::endl;
        os << params.cpg_spiral_r_str << "\t" << params.cpg_spiral_r_val << std::endl;
        os << params.cpg_spiral_p_str << "\t" << params.cpg_spiral_p_val << std::endl;

        os << params.cpg_start_str << "\t" << params.cpg_start_val << std::endl;
        os << params.cpg_change_str << "\t" << params.cpg_change_val << std::endl;
        os << params.cpg_torque_str << "\t" << params.cpg_torque_val << std::endl;

        os << params.use_sim_time_str << "\t" << params.use_sim_time_val << std::endl;
        os << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;

        return os;
    }
}