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

#ifndef SNAKE_PARAMS_CPG_HOPF_H
#define SNAKE_PARAMS_CPG_HOPF_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

/*
 * ########## Topology Mode ##########
 * 1: FCS. Fully Connected Structure.
 * 2: NNS. Nearest Neighbor Structure.
 * 3: NNNS. Next Nearest Neighbor Structure.
 * 4: SCS1. Single Chain Structure1.
 * 5: SCS2. Single Chain Structure2.
 * 6: BS. Bipartite Structure.
 * 7: DS. Dorsal Structure.
*/

/*
 * ########## Weight Mode ##########
 * 1: GW. Gaussian Weight.
 * 2: EW. Equal Weight.
*/

/*
 * ########## Motion Mode ##########
 * 0: CML. Custom Mode Locomotion.
 * 1: CL. Creeping Locomotion.
 * 2: TWL. Traveling Wave Locomotion.
 * 3: SWL. Side Winding Locomotion.
 * 4: ARL. Arc Rolling Locomotion.
 * 5: SRL. Spiral Rolling Locomotion.
*/

/*
 * ########## Coupling Mode ##########
 * 1: Decoupling.
 * 2: Coupling.
*/

/*
 * ########## CPG Mode ##########
 * 1: Improved CPG model.
 * 2: Original CPG model.
*/

namespace snake {
    class SnakeParamsCPGHopf {
    public:
        enum TopologyMode {
            FULLY_CONNECT_STRUCTURE = 1,
            NEAREST_NEIGHBOR_STRUCTURE = 2,
            NEXT_NEAREST_NEIGHBOR_STRUCTURE = 3,
            SINGLE_CHAIN_STRUCTURE_1 = 4,
            SINGLE_CHAIN_STRUCTURE_2 = 5,
            BIPARTITE_STRUCTURE = 6,
            DORSAL_STRUCTURE = 7,
        };

        enum WeightMode {
            GAUSSIAN_WEIGHT = 1,
            EQUAL_WEIGHT = 2,
        };

        enum MotionMode {
            CUSTOM_MODE_LOCOMOTION = 0,
            CREEPING_LOCOMOTION = 1,
            TRAVELING_LOCOMOTION = 2,
            SIDE_WINDING_LOCOMOTION = 3,
            ARC_ROLLING_LOCOMOTION = 4,
            SPIRAL_ROLLING_LOCOMOTION = 5,
        };

        enum CoupleMode {
            DECOUPLING = 1,
            COUPLING = 2,
        };

        enum CPGMode {
            IMPROVED_MODEL = 1,
            ORIGINAL_MODEL = 2,
        };

    public:
        SnakeParamsCPGHopf();

        SnakeParamsCPGHopf(const SnakeParamsCPGHopf &params);

        virtual ~SnakeParamsCPGHopf();

        SnakeParamsCPGHopf &operator=(const SnakeParamsCPGHopf &params);

        friend std::ostream &operator<<(std::ostream &os, SnakeParamsCPGHopf &params);

    public:
        const std::string cpg_alpha_y_str = "cpg_alpha_y";
        const std::string cpg_alpha_p_str = "cpg_alpha_p";
        double cpg_alpha_y_val; // unit: 1. Coupling gain coefficient of yaw oscillator.
        double cpg_alpha_p_val; // unit: 1. Coupling gain coefficient of pitch oscillator.

        const std::string cpg_kg_y_str = "cpg_kg_y";
        const std::string cpg_kg_p_str = "cpg_kg_p";
        double cpg_kg_y_val; // unit: 1. Amplitude error scaling coefficient of yaw oscillator.
        double cpg_kg_p_val; // unit: 1. Amplitude error scaling coefficient of pitch oscillator.

        const std::string cpg_lambda_y_str = "cpg_lambda_y";
        const std::string cpg_lambda_p_str = "cpg_lambda_p";
        double cpg_lambda_y_val; // unit: 1. Attraction factor of yaw oscillator.
        double cpg_lambda_p_val; // unit: 1. Attraction factor of pitch oscillator.

        const std::string cpg_sigma_y_str = "cpg_sigma_y";
        const std::string cpg_sigma_p_str = "cpg_sigma_p";
        double cpg_sigma_y_val; // unit: 1. Bifurcation parameter of yaw oscillator.
        double cpg_sigma_p_val; // unit: 1. Bifurcation parameter of pitch oscillator.

        const std::string cpg_uvc_y_str = "cpg_uvc_y";
        const std::string cpg_uvc_p_str = "cpg_uvc_p";
        double cpg_uvc_y_val; // unit: degree. Offset angle of yaw oscillator.
        double cpg_uvc_p_val; // unit: degree. Offset angle of pitch oscillator.

        const std::string cpg_rho_y_str = "cpg_rho_y";
        const std::string cpg_rho_p_str = "cpg_rho_p";
        double cpg_rho_y_val; // unit: degree. Amplitude parameter of yaw oscillator.
        double cpg_rho_p_val; // unit: degree. Amplitude parameter of pitch oscillator.

        const std::string cpg_omega_y_str = "cpg_omega_y";
        const std::string cpg_omega_p_str = "cpg_omega_p";
        double cpg_omega_y_val; // unit: pi*rad/s. Oscillation frequency of yaw oscillator.
        double cpg_omega_p_val; // unit: pi*rad/s. Oscillation frequency of pitch oscillator.

        const std::string cpg_phi_y_str = "cpg_phi_y";
        const std::string cpg_phi_p_str = "cpg_phi_p";
        const std::string cpg_phi_yp_str = "cpg_phi_yp";
        double cpg_phi_y_val; // unit: pi. Phase difference between yaw and yaw oscillator.
        double cpg_phi_p_val; // unit: pi. Phase difference between pitch and pitch oscillator.
        double cpg_phi_yp_val; // unit: pi. Phase difference between yaw and pitch oscillator.

        const std::string cpg_topo_mode_str = "cpg_topo_mode";
        const std::string cpg_weight_mode_str = "cpg_weight_mode";
        const std::string cpg_motion_mode_str = "cpg_motion_mode";
        const std::string cpg_couple_mode_str = "cpg_couple_mode";
        const std::string cpg_mode_str = "cpg_mode";
        TopologyMode cpg_topo_mode_val; // Topology mode.
        WeightMode cpg_weight_mode_val; // Weight mode.
        MotionMode cpg_motion_mode_val; // Motion mode.
        CoupleMode cpg_couple_mode_val; // Coupling mode.
        CPGMode cpg_mode_val; // CPG mode.

        const std::string cpg_gauss_mu_str = "cpg_gauss_mu";
        const std::string cpg_gauss_sigma_str = "cpg_gauss_sigma";
        double cpg_gauss_mu_val; // unit: 1. Position of symmetry axis of gaussian function.
        double cpg_gauss_sigma_val; // unit: 1. Scaling factor of gaussian function.

        const std::string cpg_wave_kn_str = "cpg_wave_kn";
        const std::string cpg_wave_ay_str = "cpg_wave_ay";
        const std::string cpg_wave_ap_str = "cpg_wave_ap";
        double cpg_wave_kn_val; // unit: 1. Wave number of snake robot.
        double cpg_wave_ay_val; // unit: degrees. Initial wave angle of yaw joint of snake robot.
        double cpg_wave_ap_val; // unit: degrees. Initial wave angle of pitch joint of snake robot.

        const std::string cpg_arc_r_str = "cpg_arc_r";
        double cpg_arc_r_val; // unit: m. Arc radius of snake robot.

        const std::string cpg_spiral_r_str = "cpg_spiral_r";
        const std::string cpg_spiral_p_str = "cpg_spiral_p";
        double cpg_spiral_r_val; // unit: m. Spiral curve radius of snake robot.
        double cpg_spiral_p_val; // unit: m. Spiral curve pitch of snake robot.

        const std::string cpg_start_str = "cpg_start";
        const std::string cpg_change_str = "cpg_change";
        const std::string cpg_torque_str = "cpg_torque";
        bool cpg_start_val; // Start and stop control flag.
        bool cpg_change_val; // Parameters change flag.
        bool cpg_torque_val; // Torque enable flag.

    public:
        const std::string use_sim_time_str = "use_sim_time";
        bool use_sim_time_val;
    };
}

#endif //SNAKE_PARAMS_CPG_HOPF_H
