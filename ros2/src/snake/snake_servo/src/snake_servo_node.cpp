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

#include "snake_servo/snake_servo_node.h"

SnakeServoNode::SnakeServoNode(std::string name) : rclcpp::Node(name) {
    max_rmp = 60.0 / max_speed * 60 / 360; // unit:rmp.
    unit_velocity = max_rmp / 60 * 2 * M_PI / 100; // unit:rad/s/100.
    unit_torque = max_torque * 9.8 / 100 / 1000; // unit:Nm/1000.

    seq_val_ = 0;
    joints_idn_ = 10;

    joints_id_ = new snake::u8[joints_idn_];
    position_ = new snake::s16[joints_idn_];
    speed_ = new snake::u16[joints_idn_];
    acc_ = new snake::u8[joints_idn_];
    position_fb_ = new int[joints_idn_];
    speed_fb_ = new int[joints_idn_];
    load_fb_ = new int[joints_idn_];
    voltage_fb_ = new int[joints_idn_];
    temper_fb_ = new int[joints_idn_];
    move_fb_ = new int[joints_idn_];
    current_fb_ = new int[joints_idn_];

    msg_feedback_.id.resize(joints_idn_);
    msg_feedback_.position.resize(joints_idn_);
    msg_feedback_.speed.resize(joints_idn_);
    msg_feedback_.load.resize(joints_idn_);
    msg_feedback_.voltage.resize(joints_idn_);
    msg_feedback_.temperature.resize(joints_idn_);
    msg_feedback_.move.resize(joints_idn_);
    msg_feedback_.current.resize(joints_idn_);

    for (snake::u8 i = 0; i < joints_idn_; i++) {
        joints_id_[i] = i + 1;
        msg_feedback_.id.at(i) = joints_id_[i];
    }

    auto bind_heart = std::bind(&SnakeServoNode::cbHeart, this, std::placeholders::_1);
    sub_heart_ = this->create_subscription<snake_msgs::msg::SnakeHeart>(sub_heart_str_,
                                                                        queue_size_,
                                                                        bind_heart);

    auto bind_execute = std::bind(&SnakeServoNode::cbExecute, this, std::placeholders::_1);
    sub_execute_ = this->create_subscription<snake_msgs::msg::ServoExecute>(sub_execute_str_,
                                                                            queue_size_,
                                                                            bind_execute);

    pub_feedback_ = this->create_publisher<snake_msgs::msg::ServoFeedback>(pub_feedback_str_,
                                                                           queue_size_);

    params_servo_.port_val = "/dev/ttyUSB0";
    params_servo_.rate_val = 1000000;
    params_servo_.timeout_val = 1000;
    declare_parameter<std::string>(params_servo_.port_str, params_servo_.port_val);
    declare_parameter<int>(params_servo_.rate_str, params_servo_.rate_val);
    declare_parameter<int>(params_servo_.timeout_str, params_servo_.timeout_val);
    get_parameter(params_servo_.port_str, params_servo_.port_val);
    get_parameter(params_servo_.rate_str, params_servo_.rate_val);
    get_parameter(params_servo_.timeout_str, params_servo_.timeout_val);
    get_parameter(params_servo_.use_sim_time_str, params_servo_.use_sim_time_val);
    std::cout << params_servo_ << std::endl;

    servo.setSerial(params_servo_.port_val, params_servo_.rate_val, params_servo_.timeout_val);
    servo.openSerial();
}

SnakeServoNode::~SnakeServoNode() {
    delete[]joints_id_;
    delete[]position_;
    delete[]speed_;
    delete[]acc_;
    delete[]position_fb_;
    delete[]speed_fb_;
    delete[]load_fb_;
    delete[]voltage_fb_;
    delete[]temper_fb_;
    delete[]move_fb_;
    delete[]current_fb_;

    servo.closeSerial();
}

SnakeServoNode &SnakeServoNode::operator=(const SnakeServoNode &msp) {
    if (this == &msp) {
        return *this;
    }

    max_rmp = msp.max_rmp;
    unit_velocity = msp.unit_velocity;
    unit_torque = msp.unit_torque;

    delete[]joints_id_;
    delete[]position_;
    delete[]speed_;
    delete[]acc_;
    delete[]position_fb_;
    delete[]speed_fb_;
    delete[]load_fb_;
    delete[]voltage_fb_;
    delete[]temper_fb_;
    delete[]move_fb_;
    delete[]current_fb_;

    queue_size_ = msp.queue_size_;
    seq_val_ = msp.seq_val_;
    params_servo_ = msp.params_servo_;

    joints_idn_ = msp.joints_idn_;
    joints_id_ = new snake::u8[joints_idn_];
    position_ = new snake::s16[joints_idn_];
    speed_ = new snake::u16[joints_idn_];
    acc_ = new snake::u8[joints_idn_];
    position_fb_ = new int[joints_idn_];
    speed_fb_ = new int[joints_idn_];
    load_fb_ = new int[joints_idn_];
    voltage_fb_ = new int[joints_idn_];
    temper_fb_ = new int[joints_idn_];
    move_fb_ = new int[joints_idn_];
    current_fb_ = new int[joints_idn_];

    for (snake::u8 i = 0; i < joints_idn_; i++) {
        joints_id_[i] = msp.joints_id_[i];
        position_[i] = msp.position_[i];
        speed_[i] = msp.speed_[i];
        acc_[i] = msp.acc_[i];
        position_fb_[i] = msp.position_fb_[i];
        speed_fb_[i] = msp.speed_fb_[i];
        load_fb_[i] = msp.load_fb_[i];
        voltage_fb_[i] = msp.voltage_fb_[i];
        temper_fb_[i] = msp.temper_fb_[i];
        move_fb_[i] = msp.move_fb_[i];
        current_fb_[i] = msp.current_fb_[i];
    }

    msg_feedback_ = msp.msg_feedback_;
    msg_execute_ = msp.msg_execute_;
    servo = msp.servo;
    sub_heart_ = msp.sub_heart_;
    sub_execute_ = msp.sub_execute_;
    pub_feedback_ = msp.pub_feedback_;

    return *this;
}

void SnakeServoNode::cbHeart(const snake_msgs::msg::SnakeHeart::SharedPtr msg_heart) {
    msg_heart_.header = msg_heart->header;
    msg_heart_.seq = msg_heart->seq;
    msg_heart_.heart = msg_heart->heart;
    msg_heart_.idn = msg_heart->idn;

    if (joints_idn_ != msg_heart_.idn) {
        joints_idn_ = msg_heart_.idn;

        delete[]joints_id_;
        delete[]position_;
        delete[]speed_;
        delete[]acc_;
        delete[]position_fb_;
        delete[]speed_fb_;
        delete[]load_fb_;
        delete[]voltage_fb_;
        delete[]temper_fb_;
        delete[]move_fb_;
        delete[]current_fb_;

        joints_id_ = new snake::u8[joints_idn_];
        position_ = new snake::s16[joints_idn_];
        speed_ = new snake::u16[joints_idn_];
        acc_ = new snake::u8[joints_idn_];
        position_fb_ = new int[joints_idn_];
        speed_fb_ = new int[joints_idn_];
        load_fb_ = new int[joints_idn_];
        voltage_fb_ = new int[joints_idn_];
        temper_fb_ = new int[joints_idn_];
        move_fb_ = new int[joints_idn_];
        current_fb_ = new int[joints_idn_];

        msg_feedback_.id.resize(joints_idn_);
        msg_feedback_.position.resize(joints_idn_);
        msg_feedback_.speed.resize(joints_idn_);
        msg_feedback_.load.resize(joints_idn_);
        msg_feedback_.voltage.resize(joints_idn_);
        msg_feedback_.temperature.resize(joints_idn_);
        msg_feedback_.move.resize(joints_idn_);
        msg_feedback_.current.resize(joints_idn_);

        for (snake::u8 i = 0; i < joints_idn_; i++) {
            joints_id_[i] = i + 1;
            msg_feedback_.id.at(i) = joints_id_[i];
        }
    }

    if (servo.syncFeedback(joints_id_, joints_idn_) != -1) {
        servo.syncFeedbackPosition(joints_id_, joints_idn_, position_fb_);
        servo.syncFeedbackSpeed(joints_id_, joints_idn_, speed_fb_);
        servo.syncFeedbackLoad(joints_id_, joints_idn_, load_fb_);
        servo.syncFeedbackVoltage(joints_id_, joints_idn_, voltage_fb_);
        servo.syncFeedbackTemperature(joints_id_, joints_idn_, temper_fb_);
        servo.syncFeedbackMoveStatus(joints_id_, joints_idn_, move_fb_);
        servo.syncFeedbackCurrent(joints_id_, joints_idn_, current_fb_);

        for (int i = 0; i < joints_idn_; i++) {
            // unit:degree. Real position. Counterclockwise is positive and clockwise is negative.
            auto pos_temp = static_cast<float>((4096 - 2047 - position_fb_[i]) * 2 * M_PI / 4096);
            msg_feedback_.position.at(i) = pos_temp;
            // unit:rad/s. Real speed. Counterclockwise is positive and clockwise is negative.
            msg_feedback_.speed.at(i) = static_cast<float>(-1.0 * speed_fb_[i] * unit_velocity);
            // unit:Nm. Real load. The positive and negative are subject to the official protocol.
            msg_feedback_.load.at(i) = static_cast<float>(load_fb_[i] * unit_torque);
            // unit:V. Real voltage.
            msg_feedback_.voltage.at(i) = static_cast<float>(voltage_fb_[i] / 10.0);
            // unit:degree. Real temperature.
            msg_feedback_.temperature.at(i) = static_cast<float>(temper_fb_[i]);
            // unit:bool. Real move state.
            msg_feedback_.move.at(i) = static_cast<float>(move_fb_[i]);
            // Unconverted current.
            msg_feedback_.current.at(i) = static_cast<float>(current_fb_[i]);
        }

        seq_val_ += 1;
        msg_feedback_.seq = seq_val_;
        msg_feedback_.header.stamp = now();
        msg_feedback_.header.frame_id = frame_id_str_;

        pub_feedback_->publish(msg_feedback_);

        RCLCPP_INFO(this->get_logger(),
                    "Feedback ===> Seq:[%d]\tServo IDn:[%d]\tID[0] Pos:[%f]\tID[1] Pos:[%f]",
                    msg_feedback_.seq,
                    joints_idn_,
                    msg_feedback_.position.at(0),
                    msg_feedback_.position.at(1));
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for the servo to start ...");
    }
}

void SnakeServoNode::cbExecute(const snake_msgs::msg::ServoExecute::SharedPtr msg_execute) {
    if (joints_idn_ != msg_execute->id.size()) {
        RCLCPP_INFO(this->get_logger(),
                    "Data invalid due to inconsistent IDn.");
    } else {
        msg_execute_.header = msg_execute->header;
        msg_execute_.seq = msg_execute->seq;
        msg_execute_.id = msg_execute->id;
        msg_execute_.pos = msg_execute->pos;
        msg_execute_.speed = msg_execute->speed;
        msg_execute_.acc = msg_execute->acc;
        msg_execute_.torque = msg_execute->torque;

        for (snake::u8 i = 0; i < joints_idn_; i++) {
//            auto pos_temp = static_cast<snake::s16>(std::lround(msg_execute_.pos.at(i) * 4096.0 / 360 + 2047));
            auto pos_temp = static_cast<snake::s16>(std::lround(msg_execute_.pos.at(i) * 4096.0 / (2 * M_PI) + 2047));
            if (pos_temp < servo_pos_min_) {
                pos_temp = servo_pos_min_;
            } else if (pos_temp > servo_pos_max_) {
                pos_temp = servo_pos_max_;
            }

            // Counterclockwise is positive and clockwise is negative.
            position_[i] = static_cast<snake::s16>(4096 - pos_temp);
            speed_[i] = static_cast<snake::u16>(msg_execute_.speed.at(i));
            acc_[i] = static_cast<snake::u8>(msg_execute_.acc.at(i));
        }

#ifdef ENABLE_SERVO
        servo.syncWritePosExe(joints_id_, joints_idn_, position_, speed_, acc_);
        bool torque_enable = false;
        for (int i = 0; i < joints_idn_; i++) {
            if (msg_execute_.torque.at(i)) {
                torque_enable = true;
                break;
            }
        }

        if (torque_enable) {
            servo.syncWritePosExe(joints_id_, joints_idn_, position_, speed_, acc_);
        } else {
            for (snake::u8 i = 0; i < joints_idn_; i++) {
                servo.torqueEnable(joints_id_[i], static_cast<snake::u8>(msg_execute_.torque.at(i)));
//                servo.asyncWritePosExe(joints_id_[i], position_[i], speed_[i], acc_[i]);
//                usleep(10);
            }
        }
#endif
        RCLCPP_INFO(this->get_logger(),
                    "Seq:[%d]\tServo IDn:[%d]\tID[0] Pos:[%f]\tID[1] Pos[1]:[%f]",
                    msg_execute_.seq,
                    joints_idn_,
                    msg_execute_.pos.at(0),
                    msg_execute_.pos.at(1));
    }
}