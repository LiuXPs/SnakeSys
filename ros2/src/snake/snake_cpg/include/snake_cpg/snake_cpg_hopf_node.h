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

#ifndef SNAKE_CPG_HOPF_NODE_H
#define SNAKE_CPG_HOPF_NODE_H

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_service.hpp>

#include <snake_servo/snake_feetech_servo.h>
#include <std_msgs/msg/string.hpp>
#include <snake_msgs/msg/snake_heart.hpp>
#include <snake_msgs/msg/servo_feedback.hpp>
#include <snake_msgs/msg/servo_execute.hpp>
#include <snake_params/snake_params_heart.h>
#include <snake_params/snake_params_cpg_hopf.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so2.hpp>
#include <sophus/se2.hpp>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

class SnakeCPGHopfNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SnakeCPGHopfNode() = delete;

    explicit SnakeCPGHopfNode(std::string name);

    ~SnakeCPGHopfNode() override;

public:
    /*!
     * @brief CPG composed of Hopf oscillator.
     * @param t Time.
     * @param uv The state variable uv of CPG.
     * @param duvdt The derivative duvdt of state variable uv of CPG.
     * @param params Input parameter.
     * @return Status.
     */
    static int hopfOscillator(double t, const double uv[], double duvdt[], void *params);

    static SnakeCPGHopfNode *cpg_ptr;

private:
    /*!
     * @brief
     * @param angle unit: radian. Rotation angle.
     * @return 2D homogeneous rotation matrix.
     */
    static Sophus::SE2d rot2DFun(const double &angle);

    /*!
     * @brief
     * @param x Translation along the x axis.
     * @param y Translation along the y axis.
     * @return 2D homogeneous translation matrix.
     */
    static Sophus::SE2d trans2DFun(const double &x, const double &y);

    /*!
     * @brief
     * @param angle unit: radian. Rotation angle on the x-axis.
     * @return 3D homogeneous rotation matrix about the X-axis.
     */
    static Sophus::SE3d rot3DXFun(const double &angle);

    /*!
     * @brief
     * @param angle unit: radian. Rotation angle on the y-axis.
     * @return 3D homogeneous rotation matrix about the y-axis.
     */
    static Sophus::SE3d rot3DYFun(const double &angle);

    /*!
     * @brief
     * @param angle unit: radian. Rotation angle on the z-axis.
     * @return 3D homogeneous rotation matrix about the z-axis.
     */
    static Sophus::SE3d rot3DZFun(const double &angle);

    /*!
     * @brief
     * @param x Translation along the x axis.
     * @param y Translation along the y axis.
     * @param z Translation along the z axis.
     * @return 3D homogeneous translation matrix.
     */
    static Sophus::SE3d trans3DFun(const double &x, const double &y, const double &z);

    /*!
     * @brief Gaussian function.
     * @param x Variable.
     * @param mu Position of symmetry axis of gaussian function.
     * @param sigma Scaling factor of gaussian function.
     * @return Gaussian value.
     */
    static double gaussianFun(const int &x, const double &mu, const double &sigma);

    /*!
     * @brief Sigmoid function.
     * @param x Variable.
     * @param k Scaling coefficient.
     * @param sigma Bifurcation parameter of the oscillator.
     * @return Sigmoid value.
     */
    static double sigmoidFun(const double &x, const double &k, const double &sigma);

    /*!
     * @brief Calculate the distance between oscillator i and oscillator j.
     * @param num_i Oscillator i.
     * @param num_j Oscillator j.
     * @return Distance.
     */
    static int distanceFun(const int &num_i, const int &num_j);

    /*!
     * @brief Calculate the adjacency matrix of CPG.
     * @param n The number of orthogonal joint.
     * @param topo_mode Topology mode.
     * @return Adjacency matrix.
     */
    static Eigen::MatrixXd adjacencyFun(const int &n, const snake::SnakeParamsCPGHopf::TopologyMode &topo_mode);

    /*!
     * @brief Calculate the weight matrix of CPG.
     * @param n The number of orthogonal joint.
     * @param weight_mode Weight mode.
     * @param mu Gaussian mu.
     * @param sigma Gaussian sigma.
     * @return Weight matrix.
     */
    static Eigen::MatrixXd weightFun(const int &n, const snake::SnakeParamsCPGHopf::WeightMode &weight_mode,
                                     const double &mu, const double &sigma);

    /*!
     * @brief Calculate the laplace matrix of oscillator coupling in CPG.
     * @param n The number of orthogonal joint.
     * @param topo_mode Topology mode.
     * @param weight_mode Weight mode.
     * @param gaussian_mu Gaussian mu.
     * @param gaussian_sigma Gaussian sigma.
     * @return Laplace matrix.
     */
    static Eigen::MatrixXd laplaceFun(const int &n,
                                      const snake::SnakeParamsCPGHopf::TopologyMode &topo_mode,
                                      const snake::SnakeParamsCPGHopf::WeightMode &weight_mode,
                                      const double &gaussian_mu, const double &gaussian_sigma);

    /*!
     * @brief Calculate the phase difference between CPG oscillators.
     * @param n The number of orthogonal joint.
     * @param phi_y Phase difference between yaw and yaw oscillator.
     * @param phi_p Phase difference between pitch and pitch oscillator.
     * @param phi_yp Phase difference between yaw and pitch oscillator.
     * @return Phase difference matrix.
     */
    static Eigen::MatrixXd phiFun(const int &n, const double &phi_y, const double &phi_p, const double &phi_yp);

    /*!
     * @brief Calculate the 2D rotation matrix of oscillator coupling in CPG.
     * @param n The number of orthogonal joint.
     * @param rho_yp Amplitude parameter of yaw and pitch oscillator.
     * @param sigma_yp Bifurcation parameter of yaw and pitch oscillator.
     * @param phi_y Phase difference between yaw and yaw oscillator.
     * @param phi_p Phase difference between pitch and pitch oscillator.
     * @param phi_yp Phase difference between yaw and pitch oscillator.
     * @return 2D rotation block matrix.
     */
    static Eigen::MatrixXd transFun(const int &n, const snake::SnakeParamsCPGHopf::CoupleMode &couple_mode,
                                    const Eigen::MatrixXd &rho_yp, const Eigen::MatrixXd &sigma_yp,
                                    const double &phi_y, const double &phi_p, const double &phi_yp);

    /*!
     * @brief Calculate offset angle of yaw and pitch oscillator.
     * @param uvc_y unit: degree. Offset angle of yaw oscillator.
     * @param uvc_p unit: degree. Offset angle of pitch oscillator.
     * @return unit: degree. Matrix of yaw and pitch offset angle.
     */
    static Eigen::MatrixXd uvcCalFun(const double &uvc_y, const double &uvc_p);

    /*!
     * @brief Calculate amplitude parameter of yaw and pitch oscillator.
     * @param n The number of orthogonal joint.
     * @param motion_mode Motion mode.
     * @param l Link length of snake robot.
     * @param kn Wave number of snake robot.
     * @param ay Initial wave angle of yaw joint of snake robot.
     * @param ap Initial wave angle of pitch joint of snake robot.
     * @param ar Arc radius of snake robot.
     * @param sr Spiral radius of snake robot.
     * @param sp Spiral pitch of snake robot.
     * @param ry unit: degree. Amplitude parameter of yaw oscillator.
     * @param rp unit: degree. Amplitude parameter of pitch oscillator.
     * @return unit: degree. Matrix of yaw and pitch amplitude parameter.
     */
    static Eigen::MatrixXd rhoCalFun(const int &n, const snake::SnakeParamsCPGHopf::MotionMode &motion_mode,
                                     const double &l, const double &kn,
                                     const double &ay, const double &ap,
                                     const double &ar,
                                     const double &sr, const double &sp,
                                     const double &ry, const double &rp);

    /*!
     * @brief Calculate oscillation frequency of yaw and pitch oscillator.
     * @param oy unit: pi*rad/s. Oscillation frequency of yaw oscillator.
     * @param op unit: pi*rad/s. Oscillation frequency of pitch oscillator.
     * @return unit: rad/s. Matrix of yaw and pitch oscillation frequency.
     */
    static Eigen::Matrix2Xd omegaCalFun(const double &oy, const double &op);

    /*!
     * @brief Calculate phase difference of yaw and pitch oscillator.
     * @param n The number of orthogonal joint.
     * @param motion_mode Motion mode.
     * @param l Link length of snake robot.
     * @param kn Wave number of snake robot.
     * @param sr Spiral radius of snake robot.
     * @param sp Spiral pitch of snake robot.
     * @param py unit: pi. Phase difference between yaw and yaw oscillator.
     * @param pp unit: pi. Phase difference between pitch and pitch oscillator.
     * @return unit: pi. Matrix of yaw and pitch phase difference.
     */
    static Eigen::MatrixXd phiCalFun(const int &n, const snake::SnakeParamsCPGHopf::MotionMode &motion_mode,
                                     const double &l, const double &kn,
                                     const double &sr, const double &sp,
                                     const double &py, const double &pp);

private:
    /*!
     * @brief Declare parameters.
     */
    void initParams();

    /*!
     * @brief Get parameters from parameter server.
     */
    void getParams();

    /*!
     * @brief Set parameters on parameter server.
     */
    void setParams();

    /*!
     * @brief Calculate CPG parameters.
     */
    void calParams();

    /*!
     * @brief The parameters need to be recalculated after they are changed.
     */
    void calChange();

    /*!
     * @brief Get the initial value from the last calculate result.
     */
    void calInitVal();

    /*!
     * @brief Get the initial value from the servo feedback.
     */
    void getInitVal();

    /*!
     * @brief The oscillator must be reset after the dimension change.
     */
    void resetOscillator();

    /*!
     * @brief After obtaining the feedback value, CPG was calculated and the required joint Angle was published.
     * @param msg_feedback
     */
    void cbFeedback(const snake_msgs::msg::ServoFeedback::SharedPtr msg_feedback);

private: // Interpolation differentiation.
    void initInterDiff();

    void updateInterDiff();

    void calInterDiff2P(const std::vector<double> &t, const std::vector<double> &y,
                        const int &type, double &y_diff);

    void calInterDiff3P(const std::vector<double> &t, const std::vector<double> &y,
                        const int &type, double &y_diff);

    void calInterDiff5P(const std::vector<double> &t, const std::vector<double> &y,
                        const int &type, double &y_diff);

private: // ROS2.
    uint32_t seq_val_;
    rclcpp::QoS queue_size_ = 10;
    const std::string frame_id_execute_str_ = "servo_execute_frame";
    const std::string frame_id_feedback_str_ = "servo_feedback_frame";
    const std::string frame_id_simulate_str_ = "simulate_feedback_frame";

    const std::string sub_feedback_str_ = "servo_feedback";
    const std::string pub_execute_str_ = "servo_execute";
    rclcpp::Subscription<snake_msgs::msg::ServoFeedback>::SharedPtr sub_feedback_;
    rclcpp::Publisher<snake_msgs::msg::ServoExecute>::SharedPtr pub_execute_;
    snake_msgs::msg::ServoFeedback msg_feedback_;
    snake_msgs::msg::ServoExecute msg_execute_;

    std::shared_ptr<rclcpp::AsyncParametersClient> client_param_;
    std::shared_future<std::vector<rclcpp::Parameter>> results_;

private: // Oscillator.
    int oscillator_n_;
    rclcpp::Duration time_duration_ = rclcpp::Duration(0, 0);
    rclcpp::Time time_last_;
    rclcpp::Time time_now_;
    double oscillator_t_start_;
    double oscillator_t_end_;
    double oscillator_t_h_;
    double oscillator_epsabs_;
    double oscillator_epsrel_;
    Eigen::MatrixXd oscillator_result_;
    Eigen::MatrixXd oscillator_angle_;

    double *oscillator_uv_;
    gsl_odeiv2_system *oscillator_sys_;
    gsl_odeiv2_driver *oscillator_driver_;
    const gsl_odeiv2_step_type *oscillator_step_;

    bool oscillator_cal_flag_;
    bool oscillator_init_flag_;

private: // Heart node.
    std::shared_ptr<snake::SnakeParamsHeart> params_heart_;
    const std::string node_heart_str_ = "snake_heart";

private: // CPG node.
    std::shared_ptr<snake::SnakeParamsCPGHopf> params_cpg_hopf_;

    Eigen::MatrixXd mat_alpha_yp_;
    Eigen::MatrixXd mat_kg_yp_;
    Eigen::MatrixXd mat_lambda_yp_;
    Eigen::MatrixXd mat_sigma_yp_;
    Eigen::MatrixXd mat_uvc_yp_;
    Eigen::MatrixXd mat_rho_yp_;
    Eigen::MatrixXd mat_omega_yp_;

    Eigen::MatrixXd mat_adjacency_; // Adjacency matrix: 2n * 2n.
    Eigen::MatrixXd mat_weight_; // Weight matrix: 2n * 2n.
    Eigen::MatrixXd mat_ematrix_; // Output E matrix: 2n * 4n.
    Eigen::MatrixXd mat_laplace_; // Laplacian matrix: 4n * 4n.
    Eigen::MatrixXd mat_dmatrix_; // Rotation matrix: 4n * 4n.
    Eigen::MatrixXd mat_gmatrix_; // Coupling matrix: 4n * 4n.

    Eigen::Matrix2d mat_2s_; // Matrix: 2 * 2.
    Eigen::Matrix2d mat_2i_; // Identity matrix: 2 * 2.
    Eigen::Matrix4d mat_4i_; // Identity matrix: 4 * 4.
    Eigen::MatrixXd mat_4ni_; // Identity matrix: 4n * 4n.
    Eigen::MatrixXd mat_2nx2i_; // Transform 2 matrix into 2n matrix: 2n * 2.
    Eigen::MatrixXd mat_4nx4i_; // Transform 4 matrix into 4n matrix: 4n * 4.
    Eigen::MatrixXd mat_2nx4n_; // Transform 4n matrix into 2n matrix: 2n * 4n.

private: // Interpolation differentiation.
    std::vector<double> inter_diff_time_;
    std::vector<std::vector<double>> inter_diff_data_;
    std::vector<std::vector<double>> inter_diff_data_1d_;
    std::vector<std::vector<double>> inter_diff_data_2d_;
    const int slide_win_size_ = 7;
};

#endif //SNAKE_CPG_HOPF_NODE_H
