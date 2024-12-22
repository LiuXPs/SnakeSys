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

#include "snake_cpg/snake_cpg_hopf_node.h"

SnakeCPGHopfNode *SnakeCPGHopfNode::cpg_ptr = nullptr;

SnakeCPGHopfNode::SnakeCPGHopfNode(std::string name) : rclcpp::Node(name) {
    seq_val_ = 0;
    oscillator_cal_flag_ = false;
    oscillator_init_flag_ = false;

    oscillator_uv_ = nullptr;
    oscillator_sys_ = nullptr;
    oscillator_driver_ = nullptr;

    params_heart_ = std::make_shared<snake::SnakeParamsHeart>();
    params_cpg_hopf_ = std::make_shared<snake::SnakeParamsCPGHopf>();

    client_param_ = std::make_shared<rclcpp::AsyncParametersClient>(this, node_heart_str_);
    while (!client_param_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting ...");
            exit(1);
        }
        RCLCPP_ERROR(get_logger(), "Service not available, waiting again ...");
    }
    results_ = client_param_->get_parameters({params_heart_->heart_rate_str,
                                              params_heart_->servo_idn_str,
                                              params_heart_->link_length_str});
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), results_) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        params_heart_->heart_rate_val = static_cast<int>(results_.get().at(0).as_int());
        params_heart_->servo_idn_val = static_cast<int>(results_.get().at(1).as_int());
        params_heart_->link_length_val = static_cast<double>(results_.get().at(2).as_double());

        oscillator_n_ = params_heart_->servo_idn_val / 2;
        RCLCPP_INFO(get_logger(), "Get parameters successfully.");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get the parameter value.");
        exit(1);
    }

    initParams();
    calChange();

    time_duration_ = rclcpp::Duration(static_cast<long>(1e9 / params_heart_->heart_rate_val));
    time_now_ = now();
    time_last_ = time_now_ - time_duration_;
    oscillator_t_start_ = 0.0;
    oscillator_t_end_ = oscillator_t_start_ + time_duration_.seconds();

    oscillator_result_ = 0.1 * Eigen::MatrixXd::Ones(4 * oscillator_n_, 1);
    oscillator_angle_ = Eigen::MatrixXd::Zero(2 * oscillator_n_, 1);

    resetOscillator();

    auto bind_feedback = std::bind(&SnakeCPGHopfNode::cbFeedback, this, std::placeholders::_1);
    sub_feedback_ = this->create_subscription<snake_msgs::msg::ServoFeedback>(sub_feedback_str_,
                                                                              queue_size_,
                                                                              bind_feedback);

    pub_execute_ = this->create_publisher<snake_msgs::msg::ServoExecute>(pub_execute_str_,
                                                                         queue_size_);
}

SnakeCPGHopfNode::~SnakeCPGHopfNode() {
    delete[] oscillator_uv_;
    delete oscillator_sys_;
    if (oscillator_driver_ != nullptr) {
        gsl_odeiv2_driver_free(oscillator_driver_);
    }
}

int SnakeCPGHopfNode::hopfOscillator(double t, const double uv[], double duvdt[], void *params) {
    (void) (t); // Avoid unused parameter warning.
//    SnakeCPGHopfNode *cpg_ptr = (SnakeCPGHopfNode *) params;
    cpg_ptr = (SnakeCPGHopfNode *) params;

    Eigen::MatrixXd mat_uv(4 * cpg_ptr->oscillator_n_, 1);
    for (int i = 0; i < 4 * cpg_ptr->oscillator_n_; i++) {
        mat_uv(i, 0) = uv[i] - cpg_ptr->mat_uvc_yp_(i, 0);
    }
    Eigen::MatrixXd mat_omega = Eigen::MatrixXd::Zero(4 * cpg_ptr->oscillator_n_, 4 * cpg_ptr->oscillator_n_);
    Eigen::MatrixXd mat_gamma = Eigen::MatrixXd::Zero(4 * cpg_ptr->oscillator_n_, 4 * cpg_ptr->oscillator_n_);
    Eigen::MatrixXd mat_couple = Eigen::MatrixXd::Zero(4 * cpg_ptr->oscillator_n_, 4 * cpg_ptr->oscillator_n_);

    if (cpg_ptr->params_cpg_hopf_->cpg_mode_val == snake::SnakeParamsCPGHopf::CPGMode::IMPROVED_MODEL) {
        for (int i = 0; i < 2 * cpg_ptr->oscillator_n_; i++) {
            mat_omega.block<2, 2>(2 * i, 2 * i) = cpg_ptr->mat_omega_yp_(i, 0) * cpg_ptr->mat_2s_;
        }
        for (int i = 0; i < 2 * cpg_ptr->oscillator_n_; i++) {
            double u = mat_uv(2 * i, 0);
            double v = mat_uv(2 * i + 1, 0);
            double e_g = cpg_ptr->mat_sigma_yp_(i, 0) - (pow(u, 2) + pow(v, 2)) / (pow(cpg_ptr->mat_rho_yp_(i, 0), 2));
            double xi_gamma;
            if (cpg_ptr->mat_sigma_yp_(i, 0) > 0 && e_g < 0) {
                xi_gamma = cpg_ptr->mat_lambda_yp_(i, 0) *
                           cpg_ptr->sigmoidFun(e_g, cpg_ptr->mat_kg_yp_(i, 0), cpg_ptr->mat_sigma_yp_(i, 0));
            } else {
                xi_gamma = cpg_ptr->mat_lambda_yp_(i, 0) * e_g;
            }
            mat_gamma.block<2, 2>(2 * i, 2 * i) = xi_gamma * cpg_ptr->mat_2i_;
        }
        for (int i = 0; i < 2 * cpg_ptr->oscillator_n_; i++) {
            mat_couple.row(2 * i) = -1.0 * cpg_ptr->mat_alpha_yp_(i, 0) * cpg_ptr->mat_gmatrix_.row(2 * i);
            mat_couple.row(2 * i + 1) = -1.0 * cpg_ptr->mat_alpha_yp_(i, 0) * cpg_ptr->mat_gmatrix_.row(2 * i + 1);
        }

        Eigen::MatrixXd mat_duv = (mat_omega + mat_gamma + mat_couple) * mat_uv;
        for (int i = 0; i < 4 * cpg_ptr->oscillator_n_; i++) {
            duvdt[i] = mat_duv(i, 0);
        }
    } else if (cpg_ptr->params_cpg_hopf_->cpg_mode_val == snake::SnakeParamsCPGHopf::CPGMode::ORIGINAL_MODEL) {
        for (int i = 0; i < 2 * cpg_ptr->oscillator_n_; i++) {
            mat_omega.block<2, 2>(2 * i, 2 * i) = cpg_ptr->mat_omega_yp_(i, 0) * cpg_ptr->mat_2s_;
        }
        for (int i = 0; i < 2 * cpg_ptr->oscillator_n_; i++) {
            double u = mat_uv(2 * i, 0);
            double v = mat_uv(2 * i + 1, 0);
            double e_g = cpg_ptr->mat_sigma_yp_(i, 0) - (pow(u, 2) + pow(v, 2)) / (pow(cpg_ptr->mat_rho_yp_(i, 0), 2));
            double xi_gamma = cpg_ptr->mat_lambda_yp_(i, 0) * e_g;
            mat_gamma.block<2, 2>(2 * i, 2 * i) = xi_gamma * cpg_ptr->mat_2i_;
        }
        for (int i = 0; i < 2 * cpg_ptr->oscillator_n_; i++) {
            mat_couple.row(2 * i) = -1.0 * cpg_ptr->mat_alpha_yp_(i, 0) * cpg_ptr->mat_gmatrix_.row(2 * i);
            mat_couple.row(2 * i + 1) = -1.0 * cpg_ptr->mat_alpha_yp_(i, 0) * cpg_ptr->mat_gmatrix_.row(2 * i + 1);
        }

        Eigen::MatrixXd mat_duv = (mat_omega + mat_gamma + mat_couple) * mat_uv;
        for (int i = 0; i < 4 * cpg_ptr->oscillator_n_; i++) {
            duvdt[i] = mat_duv(i, 0);
        }
    }

    return GSL_SUCCESS;
}

Sophus::SE2d SnakeCPGHopfNode::rot2DFun(const double &angle) {
    Eigen::Matrix2d mat_2d;
    mat_2d << cos(angle), -sin(angle),
            sin(angle), cos(angle);

    Sophus::SE2d mat_se2(mat_2d, Eigen::Vector2d::Zero());
    return mat_se2;
}

Sophus::SE2d SnakeCPGHopfNode::trans2DFun(const double &x, const double &y) {
    Eigen::Matrix2d mat_2d(Eigen::Matrix2d::Identity());
    Sophus::SE2d mat_se2(mat_2d, Eigen::Vector2d(x, y));
    return mat_se2;
}

Sophus::SE3d SnakeCPGHopfNode::rot3DXFun(const double &angle) {
    Eigen::Matrix3d rotx(Eigen::AngleAxisd(angle, Eigen::Vector3d(1, 0, 0)));
    Sophus::SE3d mat_se3(rotx, Eigen::Vector3d::Zero());
    return mat_se3;
}

Sophus::SE3d SnakeCPGHopfNode::rot3DYFun(const double &angle) {
    Eigen::Matrix3d roty(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 1, 0)));
    Sophus::SE3d mat_se3(roty, Eigen::Vector3d::Zero());
    return mat_se3;
}

Sophus::SE3d SnakeCPGHopfNode::rot3DZFun(const double &angle) {
    Eigen::Matrix3d rotz(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));
    Sophus::SE3d mat_se3(rotz, Eigen::Vector3d::Zero());
    return mat_se3;
}

Sophus::SE3d SnakeCPGHopfNode::trans3DFun(const double &x, const double &y, const double &z) {
    Sophus::SE3d mat_se3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, z));
    return mat_se3;
}

double SnakeCPGHopfNode::gaussianFun(const int &x, const double &mu, const double &sigma) {
    double value = exp(-1.0 * pow(x - mu, 2) / (2 * pow(sigma, 2)));
    return value;
}

double SnakeCPGHopfNode::sigmoidFun(const double &x, const double &k, const double &sigma) {
    double value = sigma * (2.0 / (1.0 + exp(-1.0 * k * x)) - 1);
    return value;
}

int SnakeCPGHopfNode::distanceFun(const int &num_i, const int &num_j) {
    int dist = 0;
    if (num_i == num_j) {
        dist = 0;
    } else if ((num_i % 2 != 0 && num_j % 2 != 0) || (num_i % 2 == 0 && num_j % 2 == 0)) {
        dist = static_cast<int>(fabs(ceil(num_i * 0.5) - ceil(num_j * 0.5)));
    } else if ((num_i % 2 != 0 && num_j % 2 == 0) || (num_i % 2 == 0 && num_j % 2 != 0)) {
        dist = static_cast<int>(fabs(ceil(num_i * 0.5) - ceil(num_j * 0.5))) + 1;
    }
    return dist;
}

Eigen::MatrixXd SnakeCPGHopfNode::adjacencyFun(const int &n,
                                               const snake::SnakeParamsCPGHopf::TopologyMode &topo_mode) {
    Eigen::MatrixXd adjacency = Eigen::MatrixXd::Zero(2 * n, 2 * n);

    if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::FULLY_CONNECT_STRUCTURE) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else {
                    adjacency(i, j) = 1;
                }
            }
        }
    } else if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::NEAREST_NEIGHBOR_STRUCTURE) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else if (distanceFun(i + 1, j + 1) == 1) {
                    adjacency(i, j) = 1;
                } else {
                    adjacency(i, j) = 0;
                }
            }
        }
    } else if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::NEXT_NEAREST_NEIGHBOR_STRUCTURE) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else if (distanceFun(i + 1, j + 1) == 1) {
                    adjacency(i, j) = 1;
                } else if (distanceFun(i + 1, j + 1) == 2) {
                    if (((i + 1) % 2 != 0 && (j + 1) % 2 == 0) || ((i + 1) % 2 == 0 && (j + 1) % 2 != 0)) {
                        adjacency(i, j) = 1;
                    } else {
                        adjacency(i, j) = 0;
                    }
                } else {
                    adjacency(i, j) = 0;
                }
            }
        }
    } else if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::SINGLE_CHAIN_STRUCTURE_1) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else if (((i + 1) == 1 && (j + 1) == 2) || ((i + 1) == 2 && (j + 1) == 1)) {
                    adjacency(i, j) = 1;
                } else if (((i + 1) % 2 != 0 && (j + 1) % 2 != 0) || ((i + 1) % 2 == 0 && (j + 1) % 2 == 0)) {
                    if (distanceFun(i + 1, j + 1) == 1)
                        adjacency(i, j) = 1;
                    else {
                        adjacency(i, j) = 0;
                    }
                } else {
                    adjacency(i, j) = 0;
                }
            }
        }
    } else if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::SINGLE_CHAIN_STRUCTURE_2) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else if (abs(i - j) == 1) {
                    adjacency(i, j) = 1;
                } else {
                    adjacency(i, j) = 0;
                }
            }
        }
    } else if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::BIPARTITE_STRUCTURE) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else if (((i + 1) % 2 != 0 && (j + 1) % 2 == 0) || ((i + 1) % 2 == 0 && (j + 1) % 2 != 0)) {
                    adjacency(i, j) = 1;
                } else {
                    adjacency(i, j) = 0;
                }
            }
        }
    } else if (topo_mode == snake::SnakeParamsCPGHopf::TopologyMode::DORSAL_STRUCTURE) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    adjacency(i, j) = 0;
                } else if (((i + 1) % 2 != 0 && (j + 1) % 2 != 0) && (distanceFun(i + 1, j + 1) == 1)) {
                    adjacency(i, j) = 1;
                } else if (((i + 1) % 2 != 0 && (j + 1) % 2 == 0) || ((i + 1) % 2 == 0 && (j + 1) % 2 != 0)) {
                    if (distanceFun(i + 1, j + 1) == 1) {
                        adjacency(i, j) = 1;
                    } else {
                        adjacency(i, j) = 0;
                    }
                } else {
                    adjacency(i, j) = 0;
                }
            }
        }
    }

    return adjacency;
}

Eigen::MatrixXd SnakeCPGHopfNode::weightFun(const int &n,
                                            const snake::SnakeParamsCPGHopf::WeightMode &weight_mode,
                                            const double &mu, const double &sigma) {
    Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(2 * n, 2 * n);

    if (weight_mode == snake::SnakeParamsCPGHopf::WeightMode::GAUSSIAN_WEIGHT) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    weight(i, j) = 0;
                } else {
                    weight(i, j) = gaussianFun(distanceFun(i + 1, j + 1), mu, sigma);
                }
            }
        }
    } else if (weight_mode == snake::SnakeParamsCPGHopf::WeightMode::EQUAL_WEIGHT) {
        for (int i = 0; i < 2 * n; i++) {
            for (int j = 0; j < 2 * n; j++) {
                if (i == j) {
                    weight(i, j) = 0;
                } else {
                    weight(i, j) = 1;
                }
            }
        }
    }

    return weight;
}

Eigen::MatrixXd SnakeCPGHopfNode::laplaceFun(const int &n,
                                             const snake::SnakeParamsCPGHopf::TopologyMode &topo_mode,
                                             const snake::SnakeParamsCPGHopf::WeightMode &weight_mode,
                                             const double &gaussian_mu,
                                             const double &gaussian_sigma) {
    Eigen::MatrixXd mat_laplace = Eigen::MatrixXd::Zero(4 * n, 4 * n);
    Eigen::MatrixXd mat_adjacency = adjacencyFun(n, topo_mode);
    Eigen::MatrixXd mat_weight = weightFun(n, weight_mode, gaussian_mu, gaussian_sigma);
    Eigen::Matrix2d mat_2i = Eigen::Matrix2d::Identity();

    for (int i = 0; i < 2 * n; i++) {
        for (int j = 0; j < 2 * n; j++) {
            if (i == j) {
                Eigen::MatrixXd row_adjacency_dot_weight = mat_adjacency.row(i).array() * mat_weight.row(i).array();
                double factor = row_adjacency_dot_weight.sum() - row_adjacency_dot_weight(0, i);
                mat_laplace.block<2, 2>(2 * i, 2 * j) = factor * mat_2i;
            } else {
                double factor = -mat_adjacency(i, j) * mat_weight(i, j);
                mat_laplace.block<2, 2>(2 * i, 2 * j) = factor * mat_2i;
            }
        }
    }

    return mat_laplace;
}

Eigen::MatrixXd SnakeCPGHopfNode::phiFun(const int &n,
                                         const double &phi_y, const double &phi_p, const double &phi_yp) {
    Eigen::MatrixXd phi(2 * n, 2 * n);

    double phi_yaw = phi_y * M_PI;
    double phi_pitch = phi_p * M_PI;
    double phi_yaw_pitch = phi_yp * M_PI;

    for (int i = 0; i < 2 * n; i++) {
        for (int j = 0; j < 2 * n; j++) {
            if (i == j) {
                phi(i, j) = 0;
            } else if ((i + 1) % 2 != 0 && (j + 1) % 2 != 0) {
                phi(i, j) = (ceil((j + 1) * 0.5) - ceil((i + 1) * 0.5)) * phi_yaw;
            } else if ((i + 1) % 2 == 0 && (j + 1) % 2 == 0) {
                phi(i, j) = (ceil((j + 1) * 0.5) - ceil((i + 1) * 0.5)) * phi_pitch;
            } else if ((i + 1) % 2 != 0 && (j + 1) % 2 == 0) {
                phi(i, j) = (ceil((j + 1) * 0.5) - ceil((i + 1) * 0.5)) * phi_pitch + phi_yaw_pitch;
            } else if ((i + 1) % 2 == 0 && (j + 1) % 2 != 0) {
                phi(i, j) = (ceil((j + 1) * 0.5) - ceil((i + 1) * 0.5)) * phi_yaw - phi_yaw_pitch;
            }
        }
    }

    return phi;
}

Eigen::MatrixXd SnakeCPGHopfNode::transFun(const int &n, const snake::SnakeParamsCPGHopf::CoupleMode &couple_mode,
                                           const Eigen::MatrixXd &rho_yp, const Eigen::MatrixXd &sigma_yp,
                                           const double &phi_y, const double &phi_p, const double &phi_yp) {
    Eigen::MatrixXd mat_d = Eigen::MatrixXd::Zero(4 * n, 4 * n);
    Eigen::MatrixXd mat_phi = phiFun(n, phi_y, phi_p, phi_yp);

    for (int i = 0; i < 2 * n; i++) {
        double ratio = 0;
        if (couple_mode == snake::SnakeParamsCPGHopf::CoupleMode::DECOUPLING) {
            ratio = (pow(sigma_yp(0, 0), 0.5) * rho_yp(0, 0)) / (pow(sigma_yp(i, 0), 0.5) * rho_yp(i, 0));
        } else if (couple_mode == snake::SnakeParamsCPGHopf::CoupleMode::COUPLING) {
            ratio = rho_yp(0, 0) / rho_yp(i, 0);
        }

        Eigen::Matrix2d rot_2d;
        double angle = mat_phi(0, i);
        rot_2d << cos(angle), -sin(angle),
                sin(angle), cos(angle);

        mat_d.block<2, 2>(2 * i, 2 * i) = ratio * rot_2d;
    }

    return mat_d;
}

Eigen::MatrixXd SnakeCPGHopfNode::uvcCalFun(const double &uvc_y, const double &uvc_p) {
    Eigen::MatrixXd uvc(4, 1);
    uvc << uvc_y, 0, uvc_p, 0;
    return uvc;
}

Eigen::MatrixXd SnakeCPGHopfNode::rhoCalFun(const int &n, const snake::SnakeParamsCPGHopf::MotionMode &motion_mode,
                                            const double &l, const double &kn,
                                            const double &ay, const double &ap,
                                            const double &ar,
                                            const double &sr, const double &sp,
                                            const double &ry, const double &rp) {
    Eigen::MatrixXd rho(2, 1);

    double rho_yaw = 0.0;
    double rho_pitch = 0.0;
    if (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::CUSTOM_MODE_LOCOMOTION) {
        rho_yaw = ry;
        rho_pitch = rp;
    } else if ((motion_mode == snake::SnakeParamsCPGHopf::MotionMode::CREEPING_LOCOMOTION) ||
               (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::TRAVELING_LOCOMOTION) ||
               (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::SIDE_WINDING_LOCOMOTION)) {
        rho_yaw = 2 * ay * sin((kn * M_PI) / n);
        rho_pitch = 2 * ap * sin((kn * M_PI) / n);
    } else if (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::ARC_ROLLING_LOCOMOTION) {
        rho_yaw = (2.0 * l / ar) / M_PI * 180;
        rho_pitch = rho_yaw;
    } else if (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::SPIRAL_ROLLING_LOCOMOTION) {
        double kappa = sr / (pow(sr, 2) + pow(sp, 2));
        double tau = sp / (pow(sr, 2) + pow(sp, 2));
        rho_yaw = (2.0 * kappa / tau * sin(tau * l)) / M_PI * 180;
        rho_pitch = rho_yaw;
    }

    rho << rho_yaw, rho_pitch;
    return rho;
}

Eigen::Matrix2Xd SnakeCPGHopfNode::omegaCalFun(const double &oy, const double &op) {
    Eigen::Matrix2Xd omega(2, 1);
    omega << oy * M_PI, op * M_PI;
    return omega;
}

Eigen::MatrixXd SnakeCPGHopfNode::phiCalFun(const int &n, const snake::SnakeParamsCPGHopf::MotionMode &motion_mode,
                                            const double &l, const double &kn,
                                            const double &sr, const double &sp,
                                            const double &py, const double &pp) {
    Eigen::MatrixXd phi(2, 1);

    double phi_yaw = 0.0;
    double phi_pitch = 0.0;
    if (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::CUSTOM_MODE_LOCOMOTION) {
        phi_yaw = py;
        phi_pitch = pp;
    } else if ((motion_mode == snake::SnakeParamsCPGHopf::MotionMode::CREEPING_LOCOMOTION) ||
               (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::TRAVELING_LOCOMOTION) ||
               (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::SIDE_WINDING_LOCOMOTION)) {
        phi_yaw = 2.0 * kn / n;
        phi_pitch = phi_yaw;
    } else if (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::ARC_ROLLING_LOCOMOTION) {
        phi_yaw = 0.0;
        phi_pitch = 0.0;
    } else if (motion_mode == snake::SnakeParamsCPGHopf::MotionMode::SPIRAL_ROLLING_LOCOMOTION) {
        double kappa = sr / (pow(sr, 2) + pow(sp, 2));
        double tau = sp / (pow(sr, 2) + pow(sp, 2));
        phi_yaw = tau * l / M_PI;
        phi_pitch = phi_yaw;
    }

    phi << phi_yaw, phi_pitch;
    return phi;
}

void SnakeCPGHopfNode::initParams() {
    declare_parameter<double>(params_cpg_hopf_->cpg_alpha_y_str, params_cpg_hopf_->cpg_alpha_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_alpha_p_str, params_cpg_hopf_->cpg_alpha_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_kg_y_str, params_cpg_hopf_->cpg_kg_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_kg_p_str, params_cpg_hopf_->cpg_kg_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_lambda_y_str, params_cpg_hopf_->cpg_lambda_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_lambda_p_str, params_cpg_hopf_->cpg_lambda_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_sigma_y_str, params_cpg_hopf_->cpg_sigma_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_sigma_p_str, params_cpg_hopf_->cpg_sigma_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_uvc_y_str, params_cpg_hopf_->cpg_uvc_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_uvc_p_str, params_cpg_hopf_->cpg_uvc_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_rho_y_str, params_cpg_hopf_->cpg_rho_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_rho_p_str, params_cpg_hopf_->cpg_rho_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_omega_y_str, params_cpg_hopf_->cpg_omega_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_omega_p_str, params_cpg_hopf_->cpg_omega_p_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_phi_y_str, params_cpg_hopf_->cpg_phi_y_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_phi_p_str, params_cpg_hopf_->cpg_phi_p_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_phi_yp_str, params_cpg_hopf_->cpg_phi_yp_val);

    int topo_mode = static_cast<int>(params_cpg_hopf_->cpg_topo_mode_val);
    int weight_mode = static_cast<int>(params_cpg_hopf_->cpg_weight_mode_val);
    int motion_mode = static_cast<int>(params_cpg_hopf_->cpg_motion_mode_val);
    int couple_mode = static_cast<int>(params_cpg_hopf_->cpg_couple_mode_val);
    int cpg_mode = static_cast<int>(params_cpg_hopf_->cpg_mode_val);
    declare_parameter<int>(params_cpg_hopf_->cpg_topo_mode_str, topo_mode);
    declare_parameter<int>(params_cpg_hopf_->cpg_weight_mode_str, weight_mode);
    declare_parameter<int>(params_cpg_hopf_->cpg_motion_mode_str, motion_mode);
    declare_parameter<int>(params_cpg_hopf_->cpg_couple_mode_str, couple_mode);
    declare_parameter<int>(params_cpg_hopf_->cpg_mode_str, cpg_mode);

    declare_parameter<double>(params_cpg_hopf_->cpg_gauss_mu_str, params_cpg_hopf_->cpg_gauss_mu_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_gauss_sigma_str, params_cpg_hopf_->cpg_gauss_sigma_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_wave_kn_str, params_cpg_hopf_->cpg_wave_kn_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_wave_ay_str, params_cpg_hopf_->cpg_wave_ay_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_wave_ap_str, params_cpg_hopf_->cpg_wave_ap_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_arc_r_str, params_cpg_hopf_->cpg_arc_r_val);

    declare_parameter<double>(params_cpg_hopf_->cpg_spiral_r_str, params_cpg_hopf_->cpg_spiral_r_val);
    declare_parameter<double>(params_cpg_hopf_->cpg_spiral_p_str, params_cpg_hopf_->cpg_spiral_p_val);

    declare_parameter<bool>(params_cpg_hopf_->cpg_start_str, params_cpg_hopf_->cpg_start_val);
    declare_parameter<bool>(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val);
    declare_parameter<bool>(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val);
}

void SnakeCPGHopfNode::getParams() {
    get_parameter(params_cpg_hopf_->cpg_alpha_y_str, params_cpg_hopf_->cpg_alpha_y_val);
    get_parameter(params_cpg_hopf_->cpg_alpha_p_str, params_cpg_hopf_->cpg_alpha_p_val);

    get_parameter(params_cpg_hopf_->cpg_kg_y_str, params_cpg_hopf_->cpg_kg_y_val);
    get_parameter(params_cpg_hopf_->cpg_kg_p_str, params_cpg_hopf_->cpg_kg_p_val);

    get_parameter(params_cpg_hopf_->cpg_lambda_y_str, params_cpg_hopf_->cpg_lambda_y_val);
    get_parameter(params_cpg_hopf_->cpg_lambda_p_str, params_cpg_hopf_->cpg_lambda_p_val);

    get_parameter(params_cpg_hopf_->cpg_sigma_y_str, params_cpg_hopf_->cpg_sigma_y_val);
    get_parameter(params_cpg_hopf_->cpg_sigma_p_str, params_cpg_hopf_->cpg_sigma_p_val);

    get_parameter(params_cpg_hopf_->cpg_uvc_y_str, params_cpg_hopf_->cpg_uvc_y_val);
    get_parameter(params_cpg_hopf_->cpg_uvc_p_str, params_cpg_hopf_->cpg_uvc_p_val);

    get_parameter(params_cpg_hopf_->cpg_rho_y_str, params_cpg_hopf_->cpg_rho_y_val);
    get_parameter(params_cpg_hopf_->cpg_rho_p_str, params_cpg_hopf_->cpg_rho_p_val);

    get_parameter(params_cpg_hopf_->cpg_omega_y_str, params_cpg_hopf_->cpg_omega_y_val);
    get_parameter(params_cpg_hopf_->cpg_omega_p_str, params_cpg_hopf_->cpg_omega_p_val);

    get_parameter(params_cpg_hopf_->cpg_phi_y_str, params_cpg_hopf_->cpg_phi_y_val);
    get_parameter(params_cpg_hopf_->cpg_phi_p_str, params_cpg_hopf_->cpg_phi_p_val);
    get_parameter(params_cpg_hopf_->cpg_phi_yp_str, params_cpg_hopf_->cpg_phi_yp_val);

    int topo_mode;
    int weight_mode;
    int motion_mode;
    int couple_mode;
    int cpg_mode;
    get_parameter(params_cpg_hopf_->cpg_topo_mode_str, topo_mode);
    get_parameter(params_cpg_hopf_->cpg_weight_mode_str, weight_mode);
    get_parameter(params_cpg_hopf_->cpg_motion_mode_str, motion_mode);
    get_parameter(params_cpg_hopf_->cpg_couple_mode_str, couple_mode);
    get_parameter(params_cpg_hopf_->cpg_mode_str, cpg_mode);
    params_cpg_hopf_->cpg_topo_mode_val = static_cast<snake::SnakeParamsCPGHopf::TopologyMode>(topo_mode);
    params_cpg_hopf_->cpg_weight_mode_val = static_cast<snake::SnakeParamsCPGHopf::WeightMode>(weight_mode);
    params_cpg_hopf_->cpg_motion_mode_val = static_cast<snake::SnakeParamsCPGHopf::MotionMode>(motion_mode);
    params_cpg_hopf_->cpg_couple_mode_val = static_cast<snake::SnakeParamsCPGHopf::CoupleMode>(couple_mode);
    params_cpg_hopf_->cpg_mode_val = static_cast<snake::SnakeParamsCPGHopf::CPGMode>(cpg_mode);

    get_parameter(params_cpg_hopf_->cpg_gauss_mu_str, params_cpg_hopf_->cpg_gauss_mu_val);
    get_parameter(params_cpg_hopf_->cpg_gauss_sigma_str, params_cpg_hopf_->cpg_gauss_sigma_val);

    get_parameter(params_cpg_hopf_->cpg_wave_kn_str, params_cpg_hopf_->cpg_wave_kn_val);
    get_parameter(params_cpg_hopf_->cpg_wave_ay_str, params_cpg_hopf_->cpg_wave_ay_val);
    get_parameter(params_cpg_hopf_->cpg_wave_ap_str, params_cpg_hopf_->cpg_wave_ap_val);

    get_parameter(params_cpg_hopf_->cpg_arc_r_str, params_cpg_hopf_->cpg_arc_r_val);

    get_parameter(params_cpg_hopf_->cpg_spiral_r_str, params_cpg_hopf_->cpg_spiral_r_val);
    get_parameter(params_cpg_hopf_->cpg_spiral_p_str, params_cpg_hopf_->cpg_spiral_p_val);

    get_parameter(params_cpg_hopf_->cpg_start_str, params_cpg_hopf_->cpg_start_val);
    get_parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val);
    get_parameter(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val);

    // Note: The name of the parameter should be the same as the index.
    client_param_->get_parameters(
            {
                    params_heart_->heart_rate_str,
                    params_heart_->servo_idn_str,
                    params_heart_->link_length_str,
            },
            [this](const std::shared_future<std::vector<rclcpp::Parameter>> &params) {
                params_heart_->heart_rate_val = static_cast<int>(params.get().at(0).as_int());
                params_heart_->servo_idn_val = static_cast<int>(params.get().at(1).as_int());
                params_heart_->link_length_val = static_cast<double>(params.get().at(2).as_double());
                RCLCPP_INFO(get_logger(), "Get parameters successfully.%d", seq_val_);

                // Modified. Because of use rclcpp::AsyncParametersClient.
                calParams();
                params_cpg_hopf_->cpg_change_val = false;
                setParams();
                oscillator_cal_flag_ = true;
            });
}

void SnakeCPGHopfNode::setParams() {
    int topo_mode = static_cast<int>(params_cpg_hopf_->cpg_topo_mode_val);
    int weight_mode = static_cast<int>(params_cpg_hopf_->cpg_weight_mode_val);
    int motion_mode = static_cast<int>(params_cpg_hopf_->cpg_motion_mode_val);
    int couple_mode = static_cast<int>(params_cpg_hopf_->cpg_couple_mode_val);
    int cpg_mode = static_cast<int>(params_cpg_hopf_->cpg_mode_val);

    set_parameters({rclcpp::Parameter(params_cpg_hopf_->cpg_alpha_y_str, params_cpg_hopf_->cpg_alpha_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_alpha_p_str, params_cpg_hopf_->cpg_alpha_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_kg_y_str, params_cpg_hopf_->cpg_kg_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_kg_p_str, params_cpg_hopf_->cpg_kg_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_lambda_y_str, params_cpg_hopf_->cpg_lambda_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_lambda_p_str, params_cpg_hopf_->cpg_lambda_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_sigma_y_str, params_cpg_hopf_->cpg_sigma_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_sigma_p_str, params_cpg_hopf_->cpg_sigma_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_uvc_y_str, params_cpg_hopf_->cpg_uvc_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_uvc_p_str, params_cpg_hopf_->cpg_uvc_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_rho_y_str, params_cpg_hopf_->cpg_rho_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_rho_p_str, params_cpg_hopf_->cpg_rho_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_omega_y_str, params_cpg_hopf_->cpg_omega_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_omega_p_str, params_cpg_hopf_->cpg_omega_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_phi_y_str, params_cpg_hopf_->cpg_phi_y_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_phi_p_str, params_cpg_hopf_->cpg_phi_p_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_phi_yp_str, params_cpg_hopf_->cpg_phi_yp_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_topo_mode_str, topo_mode),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_weight_mode_str, weight_mode),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_motion_mode_str, motion_mode),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_couple_mode_str, couple_mode),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_mode_str, cpg_mode),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_gauss_mu_str, params_cpg_hopf_->cpg_gauss_mu_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_gauss_sigma_str, params_cpg_hopf_->cpg_gauss_sigma_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_wave_kn_str, params_cpg_hopf_->cpg_wave_kn_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_wave_ay_str, params_cpg_hopf_->cpg_wave_ay_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_wave_ap_str, params_cpg_hopf_->cpg_wave_ap_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_arc_r_str, params_cpg_hopf_->cpg_arc_r_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_spiral_r_str, params_cpg_hopf_->cpg_spiral_r_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_spiral_p_str, params_cpg_hopf_->cpg_spiral_p_val),

                    rclcpp::Parameter(params_cpg_hopf_->cpg_start_str, params_cpg_hopf_->cpg_start_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val),
                    rclcpp::Parameter(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val),
                   });

    RCLCPP_INFO(get_logger(), "Set parameters successfully.%d", seq_val_);
}

void SnakeCPGHopfNode::calParams() {
    mat_2s_ << 0, -1, 1, 0;
    mat_2i_ = Eigen::Matrix2d::Identity();
    mat_4i_ = Eigen::Matrix4d::Identity();

    mat_4ni_ = Eigen::MatrixXd::Identity(4 * oscillator_n_, 4 * oscillator_n_);
    mat_2nx2i_ = Eigen::MatrixXd::Zero(2 * oscillator_n_, 2);
    mat_4nx4i_ = Eigen::MatrixXd::Zero(4 * oscillator_n_, 4);
    mat_2nx4n_ = Eigen::MatrixXd::Zero(2 * oscillator_n_, 4 * oscillator_n_);

    for (int i = 0; i < oscillator_n_; i++) {
        mat_2nx2i_.block<2, 2>(2 * i, 0) = mat_2i_;
        mat_4nx4i_.block<4, 4>(4 * i, 0) = mat_4i_;
    }
    for (int i = 0; i < 2 * oscillator_n_; i++) {
        mat_2nx4n_.row(i) = mat_4ni_.row(2 * i);
    }

    // Calculate special parameters.
    Eigen::MatrixXd mat_alpha_yp(2, 1);
    mat_alpha_yp << params_cpg_hopf_->cpg_alpha_y_val, params_cpg_hopf_->cpg_alpha_p_val;
    mat_alpha_yp_ = mat_2nx2i_ * mat_alpha_yp;

    Eigen::MatrixXd mat_kg_yp(2, 1);
    mat_kg_yp << params_cpg_hopf_->cpg_kg_y_val, params_cpg_hopf_->cpg_kg_p_val;
    mat_kg_yp_ = mat_2nx2i_ * mat_kg_yp;

    Eigen::MatrixXd mat_lambda_yp(2, 1);
    mat_lambda_yp << params_cpg_hopf_->cpg_lambda_y_val, params_cpg_hopf_->cpg_lambda_p_val;
    mat_lambda_yp_ = mat_2nx2i_ * mat_lambda_yp;

    Eigen::MatrixXd mat_sigma_yp(2, 1);
    mat_sigma_yp << params_cpg_hopf_->cpg_sigma_y_val, params_cpg_hopf_->cpg_sigma_p_val;
    mat_sigma_yp_ = mat_2nx2i_ * mat_sigma_yp;

    // Calculate uvc, rho, omega, phi.
    mat_uvc_yp_ = mat_4nx4i_ * uvcCalFun(params_cpg_hopf_->cpg_uvc_y_val, params_cpg_hopf_->cpg_uvc_p_val);

    Eigen::MatrixXd mat_rho_yaw_pitch = rhoCalFun(oscillator_n_, params_cpg_hopf_->cpg_motion_mode_val,
                                                  params_heart_->link_length_val, params_cpg_hopf_->cpg_wave_kn_val,
                                                  params_cpg_hopf_->cpg_wave_ay_val, params_cpg_hopf_->cpg_wave_ap_val,
                                                  params_cpg_hopf_->cpg_arc_r_val,
                                                  params_cpg_hopf_->cpg_spiral_r_val,
                                                  params_cpg_hopf_->cpg_spiral_p_val,
                                                  params_cpg_hopf_->cpg_rho_y_val, params_cpg_hopf_->cpg_rho_p_val);
    params_cpg_hopf_->cpg_rho_y_val = mat_rho_yaw_pitch(0, 0);
    params_cpg_hopf_->cpg_rho_p_val = mat_rho_yaw_pitch(1, 0);
    mat_rho_yp_ = mat_2nx2i_ * mat_rho_yaw_pitch;

    mat_omega_yp_ = mat_2nx2i_ * omegaCalFun(params_cpg_hopf_->cpg_omega_y_val, params_cpg_hopf_->cpg_omega_p_val);

    Eigen::MatrixXd mat_phi_yaw_pitch = phiCalFun(oscillator_n_, params_cpg_hopf_->cpg_motion_mode_val,
                                                  params_heart_->link_length_val, params_cpg_hopf_->cpg_wave_kn_val,
                                                  params_cpg_hopf_->cpg_spiral_r_val,
                                                  params_cpg_hopf_->cpg_spiral_p_val,
                                                  params_cpg_hopf_->cpg_phi_y_val, params_cpg_hopf_->cpg_phi_p_val);
    params_cpg_hopf_->cpg_phi_y_val = mat_phi_yaw_pitch(0, 0);
    params_cpg_hopf_->cpg_phi_p_val = mat_phi_yaw_pitch(1, 0);

    // Important matrix of CPG.
    mat_adjacency_ = adjacencyFun(oscillator_n_, params_cpg_hopf_->cpg_topo_mode_val);
    mat_weight_ = weightFun(oscillator_n_, params_cpg_hopf_->cpg_weight_mode_val,
                            params_cpg_hopf_->cpg_gauss_mu_val, params_cpg_hopf_->cpg_gauss_sigma_val);
    mat_ematrix_ = mat_2nx4n_;
    mat_laplace_ = laplaceFun(oscillator_n_, params_cpg_hopf_->cpg_topo_mode_val, params_cpg_hopf_->cpg_weight_mode_val,
                              params_cpg_hopf_->cpg_gauss_mu_val, params_cpg_hopf_->cpg_gauss_sigma_val);
    mat_dmatrix_ = transFun(oscillator_n_, params_cpg_hopf_->cpg_couple_mode_val, mat_rho_yp_, mat_sigma_yp_,
                            params_cpg_hopf_->cpg_phi_y_val, params_cpg_hopf_->cpg_phi_p_val,
                            params_cpg_hopf_->cpg_phi_yp_val);
    mat_gmatrix_ = mat_dmatrix_.inverse() * mat_laplace_ * mat_dmatrix_;
}

void SnakeCPGHopfNode::calChange() {
    oscillator_cal_flag_ = false;
    getParams();
}

void SnakeCPGHopfNode::calInitVal() {
    for (int i = 0; i < oscillator_result_.col(0).size(); i++) {
        oscillator_uv_[i] = oscillator_result_(i, 0);
    }
}

void SnakeCPGHopfNode::getInitVal() {
    if (msg_feedback_.header.frame_id == frame_id_feedback_str_) {
        // Prototype.
        for (int i = 0; i < static_cast<int>(msg_feedback_.id.size()); i++) {
            if (fabs(msg_feedback_.position.at(i)) < 0.1 / 180 * M_PI) {
                oscillator_uv_[2 * i] = 0.1;
            } else {
                oscillator_uv_[2 * i] = msg_feedback_.position.at(i) / M_PI * 180;
            }
            // TODO.
            oscillator_uv_[2 * i + 1] = 0.0;
        }
    } else if (msg_feedback_.header.frame_id == frame_id_simulate_str_) {
        // Simulate.
        for (int i = 0; i < static_cast<int>(msg_feedback_.id.size()); i++) {
            if (fabs(msg_feedback_.position.at(i)) < 0.1 / 180 * M_PI) {
                oscillator_uv_[2 * i] = 0.1;
            } else {
                oscillator_uv_[2 * i] = msg_feedback_.position.at(i) / M_PI * 180;
            }
            // TODO.
            oscillator_uv_[2 * i + 1] = 0.0;
        }
    }
}

void SnakeCPGHopfNode::resetOscillator() {
    oscillator_init_flag_ = false;
    oscillator_result_ = 0.1 * Eigen::MatrixXd::Ones(4 * oscillator_n_, 1);

    msg_execute_.id.resize(2 * oscillator_n_);
    msg_execute_.pos.resize(2 * oscillator_n_);
    msg_execute_.speed.resize(2 * oscillator_n_);
    msg_execute_.acc.resize(2 * oscillator_n_);
    msg_execute_.torque.resize(2 * oscillator_n_);

    oscillator_step_ = gsl_odeiv2_step_rk8pd;
    oscillator_t_h_ = 1e-6;
    oscillator_epsabs_ = 1e-6;
    oscillator_epsrel_ = 0.0;

    delete[] oscillator_uv_;
    oscillator_uv_ = new double[4 * oscillator_n_];

    delete oscillator_sys_;
    oscillator_sys_ = new gsl_odeiv2_system{SnakeCPGHopfNode::hopfOscillator,
                                            nullptr,
                                            static_cast<size_t>(4 * oscillator_n_),
                                            this};
    if (oscillator_driver_ != nullptr) {
        gsl_odeiv2_driver_free(oscillator_driver_);
    }
    oscillator_driver_ = gsl_odeiv2_driver_alloc_y_new(oscillator_sys_,
                                                       oscillator_step_, oscillator_t_h_,
                                                       oscillator_epsabs_, oscillator_epsrel_);

    initInterDiff();
}

void SnakeCPGHopfNode::cbFeedback(const snake_msgs::msg::ServoFeedback::SharedPtr msg_feedback) {
    msg_feedback_.header = msg_feedback->header;
    msg_feedback_.seq = msg_feedback->seq;
    msg_feedback_.id = msg_feedback->id;
    msg_feedback_.position = msg_feedback->position;
    msg_feedback_.speed = msg_feedback->speed;
    msg_feedback_.load = msg_feedback->load;
    msg_feedback_.voltage = msg_feedback->voltage;
    msg_feedback_.temperature = msg_feedback->temperature;
    msg_feedback_.move = msg_feedback->move;
    msg_feedback_.current = msg_feedback->current;

    if (oscillator_n_ != static_cast<int>(msg_feedback_.id.size() / 2)) {
        oscillator_n_ = static_cast<int>(msg_feedback_.id.size() / 2);
        resetOscillator();
    }

    time_last_ = time_now_;
    time_now_ = msg_feedback_.header.stamp;
    time_duration_ = time_now_ - time_last_;

    get_parameter(params_cpg_hopf_->cpg_start_str, params_cpg_hopf_->cpg_start_val);
    if (params_cpg_hopf_->cpg_start_val) {
        // If the parameters change, the CPG parameters need to be recalculated.
        get_parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val);
        if (params_cpg_hopf_->cpg_change_val) {
            calChange();
        }
        if (!oscillator_init_flag_) {
            getInitVal();
        }

        if (oscillator_cal_flag_) {
            // Solving differential equations.
            oscillator_t_end_ = oscillator_t_start_ + time_duration_.seconds();
            int status = gsl_odeiv2_driver_apply(oscillator_driver_,
                                                 &oscillator_t_start_, oscillator_t_end_,
                                                 oscillator_uv_);
            if (status != GSL_SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Odeiv2 driver failed with status [%d]", status);
                exit(1);
            }

            // The first state variable of each oscillator is extracted as the joint angle.
            for (int i = 0; i < 4 * oscillator_n_; i++) {
                oscillator_result_(i, 0) = oscillator_uv_[i];
            }
            oscillator_angle_ = mat_ematrix_ * oscillator_result_;

            // Interpolation and differentiation.
            updateInterDiff();

            // Publish execute message.
            seq_val_ += 1;
            msg_execute_.seq = seq_val_;
            msg_execute_.header.stamp = now();
            msg_execute_.header.frame_id = frame_id_execute_str_;
            for (int i = 0; i < 2 * oscillator_n_; i++) {
                msg_execute_.id.at(i) = static_cast<snake::u8>(i + 1);
                msg_execute_.pos.at(i) = static_cast<float>(oscillator_angle_(i, 0) * M_PI / 180);
                msg_execute_.speed.at(i) = inter_diff_data_1d_.at(i).back() * M_PI / 180;
                msg_execute_.acc.at(i) = inter_diff_data_2d_.at(i).back() * M_PI / 180;
                msg_execute_.torque.at(i) = true;
            }
//            if (params_cpg_hopf_->cpg_motion_mode_val == snake::SnakeParamsCPGHopf::MotionMode::CREEPING_LOCOMOTION) {
//                for (int i = 0; i < oscillator_n_; i++) {
//                    msg_execute_.torque.at(2 * i) = true;
//                    msg_execute_.torque.at(2 * i + 1) = false;
//                }
//            }
            pub_execute_->publish(msg_execute_);
            {
                oscillator_init_flag_ = true;
                params_cpg_hopf_->cpg_torque_val = oscillator_init_flag_;
                set_parameter(rclcpp::Parameter(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val));
            }
            RCLCPP_INFO(this->get_logger(),
                        "Execute ===> Seq:[%d]\tServo IDn:[%d]\tID[0] Pos:[%f]\tID[1] Pos:[%f]",
                        msg_execute_.seq,
                        msg_execute_.id.size(),
                        msg_execute_.pos.at(0),
                        msg_execute_.pos.at(1));

            // Update initial value.
            calInitVal();
        }
    } else {
        get_parameter(params_cpg_hopf_->cpg_change_str, params_cpg_hopf_->cpg_change_val);
        get_parameter(params_cpg_hopf_->cpg_torque_str, params_cpg_hopf_->cpg_torque_val);
        if (params_cpg_hopf_->cpg_change_val) {
            calChange();
        }

        if (oscillator_init_flag_ && !params_cpg_hopf_->cpg_torque_val) {
            oscillator_init_flag_ = params_cpg_hopf_->cpg_torque_val;

            // Publish unload torque message.
            seq_val_ += 1;
            msg_execute_.seq = seq_val_;
            msg_execute_.header.stamp = now();
            msg_execute_.header.frame_id = frame_id_execute_str_;
            for (int i = 0; i < 2 * oscillator_n_; i++) {
                msg_execute_.id.at(i) = static_cast<snake::u8>(i + 1);
                msg_execute_.pos.at(i) = 0.0;
                msg_execute_.speed.at(i) = 0.0;
                msg_execute_.acc.at(i) = 0.0;
                msg_execute_.torque.at(i) = false;
            }
            pub_execute_->publish(msg_execute_);
        }

        if (oscillator_init_flag_) {
            calInitVal();
        } else {
            getInitVal();
        }
        RCLCPP_INFO(get_logger(), "Waiting to start ... ");
    }
}

void SnakeCPGHopfNode::initInterDiff() {
    while (!inter_diff_time_.empty()) {
        inter_diff_time_.erase(inter_diff_time_.begin());
    }

    inter_diff_data_.clear();
    inter_diff_data_1d_.clear();
    inter_diff_data_2d_.clear();

    inter_diff_data_.resize(oscillator_n_ * 2);
    inter_diff_data_1d_.resize(oscillator_n_ * 2);
    inter_diff_data_2d_.resize(oscillator_n_ * 2);
}

void SnakeCPGHopfNode::updateInterDiff() {
    inter_diff_time_.push_back(oscillator_t_end_);
    for (int i = 0; i < 2 * oscillator_n_; i++) {
        inter_diff_data_.at(i).push_back(oscillator_angle_(i, 0));
        inter_diff_data_1d_.at(i).push_back(0.0);
        inter_diff_data_2d_.at(i).push_back(0.0);
    }
    while (inter_diff_time_.size() > static_cast<unsigned long>(slide_win_size_)) {
        inter_diff_time_.erase(inter_diff_time_.begin());
        for (int i = 0; i < 2 * oscillator_n_; i++) {
            inter_diff_data_.at(i).erase(inter_diff_data_.at(i).begin());
            inter_diff_data_1d_.at(i).erase(inter_diff_data_1d_.at(i).begin());
            inter_diff_data_2d_.at(i).erase(inter_diff_data_2d_.at(i).begin());
        }
    }

    if (inter_diff_time_.size() == 2) {
        const int offset = 2;
        const int seq_size = static_cast<int>(inter_diff_time_.size() - 1);

        for (int i = 0; i < 2 * oscillator_n_; i++) {
            calInterDiff2P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           0, inter_diff_data_1d_.at(i).at(seq_size - 1));
            calInterDiff2P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           1, inter_diff_data_1d_.at(i).at(seq_size));

            calInterDiff2P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           0, inter_diff_data_2d_.at(i).at(seq_size - 1));
            calInterDiff2P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           1, inter_diff_data_2d_.at(i).at(seq_size));
        }
    } else if (inter_diff_time_.size() == 3) {
        const int offset = 3;
        const int seq_size = static_cast<int>(inter_diff_time_.size() - 1);

        for (int i = 0; i < 2 * oscillator_n_; i++) {
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           0, inter_diff_data_1d_.at(i).at(seq_size - 2));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           1, inter_diff_data_1d_.at(i).at(seq_size - 1));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           2, inter_diff_data_1d_.at(i).at(seq_size));

            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           0, inter_diff_data_2d_.at(i).at(seq_size - 2));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           1, inter_diff_data_2d_.at(i).at(seq_size - 1));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           2, inter_diff_data_2d_.at(i).at(seq_size));
        }
    } else if (inter_diff_time_.size() == 4) {
        const int offset = 3;
        const int seq_size = static_cast<int>(inter_diff_time_.size() - 1);

        for (int i = 0; i < 2 * oscillator_n_; i++) {
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset - 1, inter_diff_time_.end() - 1),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset - 1,
                                               inter_diff_data_.at(i).end() - 1),
                           1, inter_diff_data_1d_.at(i).at(seq_size - 2));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           1, inter_diff_data_1d_.at(i).at(seq_size - 1));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           2, inter_diff_data_1d_.at(i).at(seq_size));

            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset - 1, inter_diff_time_.end() - 1),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset - 1,
                                               inter_diff_data_1d_.at(i).end() - 1),
                           1, inter_diff_data_2d_.at(i).at(seq_size - 2));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           1, inter_diff_data_2d_.at(i).at(seq_size - 1));
            calInterDiff3P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           2, inter_diff_data_2d_.at(i).at(seq_size));
        }
    } else if (inter_diff_time_.size() == 5) {
        const int offset = 5;
        const int seq_size = static_cast<int>(inter_diff_time_.size() - 1);

        for (int i = 0; i < 2 * oscillator_n_; i++) {
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           0, inter_diff_data_1d_.at(i).at(seq_size - 4));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           1, inter_diff_data_1d_.at(i).at(seq_size - 3));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           2, inter_diff_data_1d_.at(i).at(seq_size - 2));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           3, inter_diff_data_1d_.at(i).at(seq_size - 1));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           4, inter_diff_data_1d_.at(i).at(seq_size));

            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           0, inter_diff_data_2d_.at(i).at(seq_size - 4));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           1, inter_diff_data_2d_.at(i).at(seq_size - 3));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           2, inter_diff_data_2d_.at(i).at(seq_size - 2));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           3, inter_diff_data_2d_.at(i).at(seq_size - 1));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           4, inter_diff_data_2d_.at(i).at(seq_size));
        }
    } else if (inter_diff_time_.size() == 6) {
        const int offset = 5;
        const int seq_size = static_cast<int>(inter_diff_time_.size() - 1);

        for (int i = 0; i < 2 * oscillator_n_; i++) {
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           2, inter_diff_data_1d_.at(i).at(seq_size - 2));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           3, inter_diff_data_1d_.at(i).at(seq_size - 1));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           4, inter_diff_data_1d_.at(i).at(seq_size));

            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           2, inter_diff_data_2d_.at(i).at(seq_size - 2));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           3, inter_diff_data_2d_.at(i).at(seq_size - 1));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           4, inter_diff_data_2d_.at(i).at(seq_size));
        }
    } else if (inter_diff_time_.size() == 7) {
        const int offset = 5;
        const int seq_size = static_cast<int>(inter_diff_time_.size() - 1);

        for (int i = 0; i < 2 * oscillator_n_; i++) {
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           2, inter_diff_data_1d_.at(i).at(seq_size - 2));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           3, inter_diff_data_1d_.at(i).at(seq_size - 1));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_.at(i).end() - offset,
                                               inter_diff_data_.at(i).end()),
                           4, inter_diff_data_1d_.at(i).at(seq_size));

            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset - 2, inter_diff_time_.end() - 2),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset - 2,
                                               inter_diff_data_1d_.at(i).end() - 2),
                           2, inter_diff_data_2d_.at(i).at(seq_size - 4));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset - 1, inter_diff_time_.end() - 1),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset - 1,
                                               inter_diff_data_1d_.at(i).end() - 1),
                           2, inter_diff_data_2d_.at(i).at(seq_size - 3));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           2, inter_diff_data_2d_.at(i).at(seq_size - 2));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           3, inter_diff_data_2d_.at(i).at(seq_size - 1));
            calInterDiff5P(std::vector<double>(inter_diff_time_.end() - offset, inter_diff_time_.end()),
                           std::vector<double>(inter_diff_data_1d_.at(i).end() - offset,
                                               inter_diff_data_1d_.at(i).end()),
                           4, inter_diff_data_2d_.at(i).at(seq_size));
        }
    }
}

void SnakeCPGHopfNode::calInterDiff2P(const std::vector<double> &t, const std::vector<double> &y, const int &type,
                                      double &y_diff) {
    assert(t.size() == 2 && y.size() == 2);

    double h_0 = 1.0 / (t[0] - t[1]);
    double h_1 = 1.0 / (t[1] - t[0]);

    switch (type) {
        case 0:
            y_diff = y[0] * h_0 + y[1] * h_1;
            break;
        case 1:
            y_diff = y[0] * h_0 + y[1] * h_1;
            break;
        default:
            break;
    }
}

void SnakeCPGHopfNode::calInterDiff3P(const std::vector<double> &t, const std::vector<double> &y, const int &type,
                                      double &y_diff) {
    assert(t.size() == 3 && y.size() == 3);

    double h_0 = 1.0 / ((t[0] - t[1]) * (t[0] - t[2]));
    double h_1 = 1.0 / ((t[1] - t[0]) * (t[1] - t[2]));
    double h_2 = 1.0 / ((t[2] - t[0]) * (t[2] - t[1]));

    switch (type) {
        case 0:
            y_diff = (y[0] * ((t[0] - t[1]) + (t[0] - t[2])) * h_0) +
                     (y[1] * ((t[0] - t[0]) + (t[0] - t[2])) * h_1) +
                     (y[2] * ((t[0] - t[0]) + (t[0] - t[1])) * h_2);
            break;
        case 1:
            y_diff = (y[0] * ((t[1] - t[1]) + (t[1] - t[2])) * h_0) +
                     (y[1] * ((t[1] - t[0]) + (t[1] - t[2])) * h_1) +
                     (y[2] * ((t[1] - t[0]) + (t[1] - t[1])) * h_2);
            break;
        case 2:
            y_diff = (y[0] * ((t[2] - t[1]) + (t[2] - t[2])) * h_0) +
                     (y[1] * ((t[2] - t[0]) + (t[2] - t[2])) * h_1) +
                     (y[2] * ((t[2] - t[0]) + (t[2] - t[1])) * h_2);
            break;
        default:
            break;
    }
}

void SnakeCPGHopfNode::calInterDiff5P(const std::vector<double> &t, const std::vector<double> &y, const int &type,
                                      double &y_diff) {
    assert(t.size() == 5 && y.size() == 5);

    double h_0 = 1.0 / ((t[0] - t[1]) * (t[0] - t[2]) * (t[0] - t[3]) * (t[0] - t[4]));
    double h_1 = 1.0 / ((t[1] - t[0]) * (t[1] - t[2]) * (t[1] - t[3]) * (t[1] - t[4]));
    double h_2 = 1.0 / ((t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[3]) * (t[2] - t[4]));
    double h_3 = 1.0 / ((t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[2]) * (t[3] - t[4]));
    double h_4 = 1.0 / ((t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[2]) * (t[4] - t[3]));

    double val_0_1, val_0_2, val_0_3, val_0_4;
    double val_1_1, val_1_2, val_1_3, val_1_4;
    double val_2_1, val_2_2, val_2_3, val_2_4;
    double val_3_1, val_3_2, val_3_3, val_3_4;
    double val_4_1, val_4_2, val_4_3, val_4_4;

    switch (type) {
        case 0:
            val_0_1 = (t[0] - t[2]) * (t[0] - t[3]) * (t[0] - t[4]);
            val_0_2 = (t[0] - t[1]) * (t[0] - t[3]) * (t[0] - t[4]);
            val_0_3 = (t[0] - t[1]) * (t[0] - t[2]) * (t[0] - t[4]);
            val_0_4 = (t[0] - t[1]) * (t[0] - t[2]) * (t[0] - t[3]);

            val_1_1 = (t[0] - t[2]) * (t[0] - t[3]) * (t[0] - t[4]);
            val_1_2 = (t[0] - t[0]) * (t[0] - t[3]) * (t[0] - t[4]);
            val_1_3 = (t[0] - t[0]) * (t[0] - t[2]) * (t[0] - t[4]);
            val_1_4 = (t[0] - t[0]) * (t[0] - t[2]) * (t[0] - t[3]);

            val_2_1 = (t[0] - t[1]) * (t[0] - t[3]) * (t[0] - t[4]);
            val_2_2 = (t[0] - t[0]) * (t[0] - t[3]) * (t[0] - t[4]);
            val_2_3 = (t[0] - t[0]) * (t[0] - t[1]) * (t[0] - t[4]);
            val_2_4 = (t[0] - t[0]) * (t[0] - t[1]) * (t[0] - t[3]);

            val_3_1 = (t[0] - t[1]) * (t[0] - t[2]) * (t[0] - t[4]);
            val_3_2 = (t[0] - t[0]) * (t[0] - t[2]) * (t[0] - t[4]);
            val_3_3 = (t[0] - t[0]) * (t[0] - t[1]) * (t[0] - t[4]);
            val_3_4 = (t[0] - t[0]) * (t[0] - t[1]) * (t[0] - t[2]);

            val_4_1 = (t[0] - t[1]) * (t[0] - t[2]) * (t[0] - t[3]);
            val_4_2 = (t[0] - t[0]) * (t[0] - t[2]) * (t[0] - t[3]);
            val_4_3 = (t[0] - t[0]) * (t[0] - t[1]) * (t[0] - t[3]);
            val_4_4 = (t[0] - t[0]) * (t[0] - t[1]) * (t[0] - t[2]);

            y_diff = (y[0] * (val_0_1 + val_0_2 + val_0_3 + val_0_4) * h_0) +
                     (y[1] * (val_1_1 + val_1_2 + val_1_3 + val_1_4) * h_1) +
                     (y[2] * (val_2_1 + val_2_2 + val_2_3 + val_2_4) * h_2) +
                     (y[3] * (val_3_1 + val_3_2 + val_3_3 + val_3_4) * h_3) +
                     (y[4] * (val_4_1 + val_4_2 + val_4_3 + val_4_4) * h_4);
            break;
        case 1:
            val_0_1 = (t[1] - t[2]) * (t[1] - t[3]) * (t[1] - t[4]);
            val_0_2 = (t[1] - t[1]) * (t[1] - t[3]) * (t[1] - t[4]);
            val_0_3 = (t[1] - t[1]) * (t[1] - t[2]) * (t[1] - t[4]);
            val_0_4 = (t[1] - t[1]) * (t[0] - t[2]) * (t[1] - t[3]);

            val_1_1 = (t[1] - t[2]) * (t[1] - t[3]) * (t[1] - t[4]);
            val_1_2 = (t[1] - t[0]) * (t[1] - t[3]) * (t[1] - t[4]);
            val_1_3 = (t[1] - t[0]) * (t[1] - t[2]) * (t[1] - t[4]);
            val_1_4 = (t[1] - t[0]) * (t[1] - t[2]) * (t[1] - t[3]);

            val_2_1 = (t[1] - t[1]) * (t[1] - t[3]) * (t[1] - t[4]);
            val_2_2 = (t[1] - t[0]) * (t[1] - t[3]) * (t[1] - t[4]);
            val_2_3 = (t[1] - t[0]) * (t[1] - t[1]) * (t[1] - t[4]);
            val_2_4 = (t[1] - t[0]) * (t[1] - t[1]) * (t[1] - t[3]);

            val_3_1 = (t[1] - t[1]) * (t[1] - t[2]) * (t[1] - t[4]);
            val_3_2 = (t[1] - t[0]) * (t[1] - t[2]) * (t[1] - t[4]);
            val_3_3 = (t[1] - t[0]) * (t[1] - t[1]) * (t[1] - t[4]);
            val_3_4 = (t[1] - t[0]) * (t[1] - t[1]) * (t[1] - t[2]);

            val_4_1 = (t[1] - t[1]) * (t[1] - t[2]) * (t[1] - t[3]);
            val_4_2 = (t[1] - t[0]) * (t[1] - t[2]) * (t[1] - t[3]);
            val_4_3 = (t[1] - t[0]) * (t[1] - t[1]) * (t[1] - t[3]);
            val_4_4 = (t[1] - t[0]) * (t[1] - t[1]) * (t[1] - t[2]);

            y_diff = (y[0] * (val_0_1 + val_0_2 + val_0_3 + val_0_4) * h_0) +
                     (y[1] * (val_1_1 + val_1_2 + val_1_3 + val_1_4) * h_1) +
                     (y[2] * (val_2_1 + val_2_2 + val_2_3 + val_2_4) * h_2) +
                     (y[3] * (val_3_1 + val_3_2 + val_3_3 + val_3_4) * h_3) +
                     (y[4] * (val_4_1 + val_4_2 + val_4_3 + val_4_4) * h_4);
            break;
        case 2:
            val_0_1 = (t[2] - t[2]) * (t[2] - t[3]) * (t[2] - t[4]);
            val_0_2 = (t[2] - t[1]) * (t[2] - t[3]) * (t[2] - t[4]);
            val_0_3 = (t[2] - t[1]) * (t[2] - t[2]) * (t[2] - t[4]);
            val_0_4 = (t[2] - t[1]) * (t[2] - t[2]) * (t[2] - t[3]);

            val_1_1 = (t[2] - t[2]) * (t[2] - t[3]) * (t[2] - t[4]);
            val_1_2 = (t[2] - t[0]) * (t[2] - t[3]) * (t[2] - t[4]);
            val_1_3 = (t[2] - t[0]) * (t[2] - t[2]) * (t[2] - t[4]);
            val_1_4 = (t[2] - t[0]) * (t[2] - t[2]) * (t[2] - t[3]);

            val_2_1 = (t[2] - t[1]) * (t[2] - t[3]) * (t[2] - t[4]);
            val_2_2 = (t[2] - t[0]) * (t[2] - t[3]) * (t[2] - t[4]);
            val_2_3 = (t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[4]);
            val_2_4 = (t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[3]);

            val_3_1 = (t[2] - t[1]) * (t[2] - t[2]) * (t[2] - t[4]);
            val_3_2 = (t[2] - t[0]) * (t[2] - t[2]) * (t[2] - t[4]);
            val_3_3 = (t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[4]);
            val_3_4 = (t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[2]);

            val_4_1 = (t[2] - t[1]) * (t[2] - t[2]) * (t[2] - t[3]);
            val_4_2 = (t[2] - t[0]) * (t[2] - t[2]) * (t[2] - t[3]);
            val_4_3 = (t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[3]);
            val_4_4 = (t[2] - t[0]) * (t[2] - t[1]) * (t[2] - t[2]);

            y_diff = (y[0] * (val_0_1 + val_0_2 + val_0_3 + val_0_4) * h_0) +
                     (y[1] * (val_1_1 + val_1_2 + val_1_3 + val_1_4) * h_1) +
                     (y[2] * (val_2_1 + val_2_2 + val_2_3 + val_2_4) * h_2) +
                     (y[3] * (val_3_1 + val_3_2 + val_3_3 + val_3_4) * h_3) +
                     (y[4] * (val_4_1 + val_4_2 + val_4_3 + val_4_4) * h_4);
            break;
        case 3:
            val_0_1 = (t[3] - t[2]) * (t[3] - t[3]) * (t[3] - t[4]);
            val_0_2 = (t[3] - t[1]) * (t[3] - t[3]) * (t[3] - t[4]);
            val_0_3 = (t[3] - t[1]) * (t[3] - t[2]) * (t[3] - t[4]);
            val_0_4 = (t[3] - t[1]) * (t[3] - t[2]) * (t[3] - t[3]);

            val_1_1 = (t[3] - t[2]) * (t[3] - t[3]) * (t[3] - t[4]);
            val_1_2 = (t[3] - t[0]) * (t[3] - t[3]) * (t[3] - t[4]);
            val_1_3 = (t[3] - t[0]) * (t[3] - t[2]) * (t[3] - t[4]);
            val_1_4 = (t[3] - t[0]) * (t[3] - t[2]) * (t[3] - t[3]);

            val_2_1 = (t[3] - t[1]) * (t[3] - t[3]) * (t[3] - t[4]);
            val_2_2 = (t[3] - t[0]) * (t[3] - t[3]) * (t[3] - t[4]);
            val_2_3 = (t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[4]);
            val_2_4 = (t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[3]);

            val_3_1 = (t[3] - t[1]) * (t[3] - t[2]) * (t[3] - t[4]);
            val_3_2 = (t[3] - t[0]) * (t[3] - t[2]) * (t[3] - t[4]);
            val_3_3 = (t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[4]);
            val_3_4 = (t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[2]);

            val_4_1 = (t[3] - t[1]) * (t[3] - t[2]) * (t[3] - t[3]);
            val_4_2 = (t[3] - t[0]) * (t[3] - t[2]) * (t[3] - t[3]);
            val_4_3 = (t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[3]);
            val_4_4 = (t[3] - t[0]) * (t[3] - t[1]) * (t[3] - t[2]);

            y_diff = (y[0] * (val_0_1 + val_0_2 + val_0_3 + val_0_4) * h_0) +
                     (y[1] * (val_1_1 + val_1_2 + val_1_3 + val_1_4) * h_1) +
                     (y[2] * (val_2_1 + val_2_2 + val_2_3 + val_2_4) * h_2) +
                     (y[3] * (val_3_1 + val_3_2 + val_3_3 + val_3_4) * h_3) +
                     (y[4] * (val_4_1 + val_4_2 + val_4_3 + val_4_4) * h_4);
            break;
        case 4:
            val_0_1 = (t[4] - t[2]) * (t[4] - t[3]) * (t[4] - t[4]);
            val_0_2 = (t[4] - t[1]) * (t[4] - t[3]) * (t[4] - t[4]);
            val_0_3 = (t[4] - t[1]) * (t[4] - t[2]) * (t[4] - t[4]);
            val_0_4 = (t[4] - t[1]) * (t[4] - t[2]) * (t[4] - t[3]);

            val_1_1 = (t[4] - t[2]) * (t[4] - t[3]) * (t[4] - t[4]);
            val_1_2 = (t[4] - t[0]) * (t[4] - t[3]) * (t[4] - t[4]);
            val_1_3 = (t[4] - t[0]) * (t[4] - t[2]) * (t[4] - t[4]);
            val_1_4 = (t[4] - t[0]) * (t[4] - t[2]) * (t[4] - t[3]);

            val_2_1 = (t[4] - t[1]) * (t[4] - t[3]) * (t[4] - t[4]);
            val_2_2 = (t[4] - t[0]) * (t[4] - t[3]) * (t[4] - t[4]);
            val_2_3 = (t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[4]);
            val_2_4 = (t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[3]);

            val_3_1 = (t[4] - t[1]) * (t[4] - t[2]) * (t[4] - t[4]);
            val_3_2 = (t[4] - t[0]) * (t[4] - t[2]) * (t[4] - t[4]);
            val_3_3 = (t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[4]);
            val_3_4 = (t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[2]);

            val_4_1 = (t[4] - t[1]) * (t[4] - t[2]) * (t[4] - t[3]);
            val_4_2 = (t[4] - t[0]) * (t[4] - t[2]) * (t[4] - t[3]);
            val_4_3 = (t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[3]);
            val_4_4 = (t[4] - t[0]) * (t[4] - t[1]) * (t[4] - t[2]);

            y_diff = (y[0] * (val_0_1 + val_0_2 + val_0_3 + val_0_4) * h_0) +
                     (y[1] * (val_1_1 + val_1_2 + val_1_3 + val_1_4) * h_1) +
                     (y[2] * (val_2_1 + val_2_2 + val_2_3 + val_2_4) * h_2) +
                     (y[3] * (val_3_1 + val_3_2 + val_3_3 + val_3_4) * h_3) +
                     (y[4] * (val_4_1 + val_4_2 + val_4_3 + val_4_4) * h_4);
            break;
        default:
            break;
    }
}
