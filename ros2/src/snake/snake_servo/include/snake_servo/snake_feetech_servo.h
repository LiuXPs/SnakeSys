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

#ifndef SNAKE_FEETECH_SERVO_H
#define SNAKE_FEETECH_SERVO_H

#include "snake_servo/snake_feetech_proto.h"

namespace snake {
    class FeeTechServo : public FeeTechProto {
    public:
        FeeTechServo();

        explicit FeeTechServo(u8 End);

        FeeTechServo(u8 End, u8 Level);

        ~FeeTechServo() override;

        FeeTechServo &operator=(const FeeTechServo &feetech_servo);

    private:
        u8 memory_[254][SMBL_NOW_CURRENT_H - SMBL_NOW_POSITION_L + 1]{};
    public:
        /*!
         * Generally write single servo1 position command.
         * @param ID Servo ID.
         * @param Position Goal position.
         * @param Speed Goal speed.
         * @param Acc Goal acceleration.
         * @return Data length.
         * Timeout return -1.
         */
        int genWritePosExe(u8 ID, s16 Position, u16 Speed, u8 Acc = 0);

        /*!
         * Asynchronously write single servo1 position command.
         * @param ID Servo ID.
         * @param Position Goal position.
         * @param Speed Goal speed.
         * @param Acc Goal acceleration.
         * @return Data length.
         * Timeout return -1.
         */
        int asyncWritePosExe(u8 ID, s16 Position, u16 Speed, u8 Acc = 0);

        /*!
         * Synchronously write multiple servo1 position command.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Position Goal position.
         * @param Speed Goal speed.
         * @param Acc Goal acceleration.
         */
        void syncWritePosExe(const u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 Acc[]);

        /*!
         * Set operation mode.
         * @param ID Servo ID.
         * @param Mode
         * 0: Servo mode.
         * 1: speed_ mode.
         * 2: PWM mode.
         * @return Data length.
         * Timeout return -1.
         */
        int setOperationMode(u8 ID, u8 Mode = 0);

        /*!
         * speed_ mode control command.
         * @param ID Servo ID.
         * @param Speed Servo speed.
         * @param Acc Servo acceleration.
         * @return Data length.
         * Timeout return -1.
         */
        int writeSpeed(u8 ID, s16 Speed, u8 Acc = 0);

        /*!
         * Torque control command.
         * @param ID Servo ID.
         * @param Enable
         * 0: Close torque.
         * 1: Open torque.
         * After receiving the position write command,
         * the torque switch will be automatically set to 1 to turn on the torque output.
         * @return Data length.
         * Timeout return -1.
         */
        int torqueEnable(u8 ID, u8 Enable);

        /*!
         * Eeprom lock control command.
         * @param ID Servo ID.
         * @param lock
         * 0: Close eeprom lock.
         * 1: Open eeprom lock.
         * When the lock function bit is set to 0,
         * the writing speed of intelligent servo1 will slow down and the parameters in
         * EEPROM area will be written frequently input operation will affect the service
         * life of intelligent servo1 EEPROM.
         * @return Data length.
         * Timeout return -1.
         */
        int eepromLock(u8 ID, u8 lock);

        /*!
         * Generally feedback servo1 information.
         * @param ID Servo ID.
         * @return Data length.
         * Timeout return -1.
         */
        int genFeedback(u8 ID);

        /*!
         * Generally feedback servo1 position.
         * @param ID Servo ID.
         * @param Position Feedback position.
         */
        void genFeedbackPosition(int ID, int &Position);

        /*!
         * Generally feedback servo1 speed.
         * @param ID Servo ID.
         * @param Speed Feedback speed.
         */
        void genFeedbackSpeed(int ID, int &Speed);

        /*!
         * Generally feedback servo1 load.
         * @param ID Servo ID.
         * @param Load Feedback load.
         */
        void genFeedbackLoad(int ID, int &Load);

        /*!
         * Generally feedback servo1 voltage.
         * @param ID Servo ID.
         * @param Voltage Feedback voltage.
         */
        void genFeedbackVoltage(int ID, int &Voltage);

        /*!
         * Generally feedback servo1 temperature.
         * @param ID Servo ID.
         * @param Temperature Feedback temperature.
         */
        void genFeedbackTemperature(int ID, int &Temperature);

        /*!
         * Generally feedback servo1 move status.
         * @param ID Servo ID.
         * @param Move Feedback move status.
         * 0: Goal position command execution completed.
         * 1: Goal position command execution in progress.
         */
        void genFeedbackMoveStatus(int ID, int &Move);

        /*!
         * Generally feedback servo1 current.
         * @param ID Servo ID.
         * @param Current Feedback current.
         */
        void genFeedbackCurrent(int ID, int &Current);

        /*!
         * Synchronously feedback servo1 information.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @return Data length.
         * Timeout return -1.
         */
        int syncFeedback(const u8 ID[], u8 IDN);

        /*!
         * Synchronously feedback servo1 position.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Position Feedback position.
         */
        void syncFeedbackPosition(const u8 ID[], u8 IDN, int Position[]);

        /*!
         * Synchronously feedback servo1 speed.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Speed Feedback speed.
         */
        void syncFeedbackSpeed(const u8 ID[], u8 IDN, int Speed[]);

        /*!
         * Synchronously feedback servo1 load.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Load Feedback load.
         */
        void syncFeedbackLoad(const u8 ID[], u8 IDN, int Load[]);

        /*!
         * Synchronously feedback servo1 voltage.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Voltage Feedback voltage.
         */
        void syncFeedbackVoltage(const u8 ID[], u8 IDN, int Voltage[]);

        /*!
         * Synchronously feedback servo1 temperature.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Temper Feedback temperature.
         */
        void syncFeedbackTemperature(const u8 ID[], u8 IDN, int Temper[]);

        /*!
         * Synchronously feedback servo1 move status.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Move Feedback move status.
         * 0: Goal position command execution completed.
         * 1: Goal position command execution in progress.
         */
        void syncFeedbackMoveStatus(const u8 ID[], u8 IDN, int Move[]);

        /*!
         * Synchronously feedback servo1 current.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param Current Feedback current.
         */
        void syncFeedbackCurrent(const u8 ID[], u8 IDN, int Current[]);
    };
}

#endif // SNAKE_FEETECH_SERVO_H
