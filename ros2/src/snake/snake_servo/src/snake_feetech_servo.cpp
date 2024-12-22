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

#include "snake_servo/snake_feetech_servo.h"

namespace snake {
    FeeTechServo::FeeTechServo() = default;

    FeeTechServo::FeeTechServo(u8 End) : FeeTechProto(End) {}

    FeeTechServo::FeeTechServo(u8 End, u8 Level) : FeeTechProto(End, Level) {}

    FeeTechServo::~FeeTechServo() = default;

    FeeTechServo &FeeTechServo::operator=(const FeeTechServo &feetech_servo) {
        if (this == &feetech_servo) {
            return *this;
        }

        static_cast<FeeTechProto &>(*this) = feetech_servo;
        u8 rows = sizeof(decltype(this->memory_)) / sizeof(decltype(this->memory_[0]));
        u8 cols = sizeof(decltype(this->memory_[0]));
        for (u8 i = 0; i < rows; i++) {
            for (u8 j = 0; j < cols; j++) {
                this->memory_[i][j] = feetech_servo.memory_[i][j];
            }
        }
        return *this;
    }

    int FeeTechServo::genWritePosExe(const u8 ID, s16 Position, u16 Speed, u8 Acc) {
        if (Position < 0) {
            Position = -Position;
            Position |= (1 << 15);
        }
        u8 b_buf[7];
        b_buf[0] = Acc;
        Host2SMBL(b_buf + 1, b_buf + 2, Position);
        Host2SMBL(b_buf + 3, b_buf + 4, 0);
        Host2SMBL(b_buf + 5, b_buf + 6, Speed);

        return genWrite(ID, SMBL_GOAL_ACCELERATION, b_buf, 7);
    }

    int FeeTechServo::asyncWritePosExe(const u8 ID, s16 Position, u16 Speed, u8 Acc) {
        if (Position < 0) {
            Position = -Position;
            Position |= (1 << 15);
        }
        u8 b_buf[7];
        u16 V;
        if (Speed) {
            V = Speed;
        } else {
            V = 0;
        }
        if (Acc) {
            b_buf[0] = Acc;
        } else {
            b_buf[0] = 0;
        }
        Host2SMBL(b_buf + 1, b_buf + 2, Position);
        Host2SMBL(b_buf + 3, b_buf + 4, 0);
        Host2SMBL(b_buf + 5, b_buf + 6, V);

        return asyncWrite(ID, SMBL_GOAL_ACCELERATION, b_buf, 7);
    }

    void FeeTechServo::syncWritePosExe(const u8 ID[], const u8 IDN, s16 Position[], u16 Speed[], u8 Acc[]) {
        auto m_buf = new u8[IDN][7];
//        u8 m_buf[IDN][7];
        for (u8 i = 0; i < IDN; i++) {
            if (Position[i] < 0) {
                Position[i] = -Position[i];
                Position[i] |= (1 << 15);
            }
            u8 b_buf[7];
            u16 V;
            if (Speed) {
                V = Speed[i];
            } else {
                V = 0;
            }
            if (Acc) {
                b_buf[0] = Acc[i];
            } else {
                b_buf[0] = 0;
            }
            Host2SMBL(b_buf + 1, b_buf + 2, Position[i]);
            Host2SMBL(b_buf + 3, b_buf + 4, 0);
            Host2SMBL(b_buf + 5, b_buf + 6, V);
            memcpy(m_buf[i], b_buf, 7);
        }
        syncWrite(ID, IDN, SMBL_GOAL_ACCELERATION, (u8 *) m_buf, 7);
        delete[] m_buf;
    }

    int FeeTechServo::setOperationMode(const u8 ID, u8 Mode) {
        return writeByte(ID, SMBL_OPERATION_MODE, Mode);
    }

    int FeeTechServo::writeSpeed(const u8 ID, s16 Speed, u8 Acc) {
        if (Speed < 0) {
            Speed = -Speed;
            Speed |= (1 << 15);
        }
        writeByte(ID, SMBL_GOAL_ACCELERATION, Acc);
        return writeWord(ID, SMBL_GOAL_SPEED_L, Speed);
    }

    int FeeTechServo::torqueEnable(const u8 ID, u8 Enable) {
        return writeByte(ID, SMBL_TORQUE_ENABLE, Enable);
    }

    int FeeTechServo::eepromLock(const u8 ID, u8 lock) {
        return writeByte(ID, SMBL_EEPROM_LOCK, lock);
    }

    int FeeTechServo::genFeedback(const u8 ID) {
        int n_len = genRead(ID, SMBL_NOW_POSITION_L, this->memory_[ID], sizeof(this->memory_[ID]));
        if (n_len != sizeof(this->memory_[ID])) {
            setError(1);
            return -1;
        }
        setError(0);
        return n_len;
    }

    void FeeTechServo::genFeedbackPosition(const int ID, int &Position) {
        Position = SMBL2Host(this->memory_[ID][SMBL_NOW_POSITION_L - SMBL_NOW_POSITION_L],
                             this->memory_[ID][SMBL_NOW_POSITION_H - SMBL_NOW_POSITION_L]);

        if (!getError() && (Position & (1 << 15))) {
            Position = -(Position & ~(1 << 15));
        }
    }

    void FeeTechServo::genFeedbackSpeed(const int ID, int &Speed) {
        Speed = SMBL2Host(this->memory_[ID][SMBL_NOW_SPEED_L - SMBL_NOW_POSITION_L],
                          this->memory_[ID][SMBL_NOW_SPEED_H - SMBL_NOW_POSITION_L]);

        if (!getError() && (Speed & (1 << 15))) {
            Speed = -(Speed & ~(1 << 15));
        }
    }

    void FeeTechServo::genFeedbackLoad(const int ID, int &Load) {
        Load = SMBL2Host(this->memory_[ID][SMBL_NOW_LOAD_L - SMBL_NOW_POSITION_L],
                         this->memory_[ID][SMBL_NOW_LOAD_H - SMBL_NOW_POSITION_L]);

        if (!getError() && (Load & (1 << 10))) {
            Load = -(Load & ~(1 << 10));
        }
    }

    void FeeTechServo::genFeedbackVoltage(const int ID, int &Voltage) {
        Voltage = this->memory_[ID][SMBL_NOW_VOLTAGE - SMBL_NOW_POSITION_L];
    }

    void FeeTechServo::genFeedbackTemperature(const int ID, int &Temperature) {
        Temperature = this->memory_[ID][SMBL_NOW_TEMPERATURE - SMBL_NOW_POSITION_L];
    }

    void FeeTechServo::genFeedbackMoveStatus(const int ID, int &Move) {
        Move = this->memory_[ID][SMBL_MOVE_STATUS - SMBL_NOW_POSITION_L];
    }

    void FeeTechServo::genFeedbackCurrent(const int ID, int &Current) {
        Current = SMBL2Host(this->memory_[ID][SMBL_NOW_CURRENT_L - SMBL_NOW_POSITION_L],
                            this->memory_[ID][SMBL_NOW_CURRENT_H - SMBL_NOW_POSITION_L]);

        if (!getError() && (Current & (1 << 15))) {
            Current = -(Current & ~(1 << 15));
        }
    }

    int FeeTechServo::syncFeedback(const u8 ID[], const u8 IDN) {
        int n_len = syncRead(ID, IDN, SMBL_NOW_POSITION_L, (u8 *) this->memory_, sizeof(this->memory_[0]));
        if (n_len != sizeof(this->memory_[0])) {
            setError(1);
            return -1;
        }
        setError(0);
        return n_len;
    }

    void FeeTechServo::syncFeedbackPosition(const u8 ID[], const u8 IDN, int Position[]) {
        for (u8 i = 0; i < IDN; i++) {
            Position[i] = SMBL2Host(this->memory_[ID[i]][SMBL_NOW_POSITION_L - SMBL_NOW_POSITION_L],
                                    this->memory_[ID[i]][SMBL_NOW_POSITION_H - SMBL_NOW_POSITION_L]);

            if (!getError() && (Position[i] & (1 << 15))) {
                Position[i] = -(Position[i] & ~(1 << 15));
            }
        }
    }

    void FeeTechServo::syncFeedbackSpeed(const u8 ID[], const u8 IDN, int Speed[]) {
        for (u8 i = 0; i < IDN; i++) {
            Speed[i] = SMBL2Host(this->memory_[ID[i]][SMBL_NOW_SPEED_L - SMBL_NOW_POSITION_L],
                                 this->memory_[ID[i]][SMBL_NOW_SPEED_H - SMBL_NOW_POSITION_L]);

            if (!getError() && (Speed[i] & (1 << 15))) {
                Speed[i] = -(Speed[i] & ~(1 << 15));
            }
        }
    }

    void FeeTechServo::syncFeedbackLoad(const u8 ID[], const u8 IDN, int Load[]) {
        for (u8 i = 0; i < IDN; i++) {
            Load[i] = SMBL2Host(this->memory_[ID[i]][SMBL_NOW_LOAD_L - SMBL_NOW_POSITION_L],
                                this->memory_[ID[i]][SMBL_NOW_LOAD_H - SMBL_NOW_POSITION_L]);

            if (!getError() && (Load[i] & (1 << 10))) {
                Load[i] = -(Load[i] & ~(1 << 10));
            }
        }
    }

    void FeeTechServo::syncFeedbackVoltage(const u8 ID[], const u8 IDN, int Voltage[]) {
        for (u8 i = 0; i < IDN; i++) {
            Voltage[i] = this->memory_[ID[i]][SMBL_NOW_VOLTAGE - SMBL_NOW_POSITION_L];
        }
    }

    void FeeTechServo::syncFeedbackTemperature(const u8 ID[], const u8 IDN, int Temper[]) {
        for (u8 i = 0; i < IDN; i++) {
            Temper[i] = this->memory_[ID[i]][SMBL_NOW_TEMPERATURE - SMBL_NOW_POSITION_L];
        }
    }

    void FeeTechServo::syncFeedbackMoveStatus(const u8 ID[], const u8 IDN, int Move[]) {
        for (u8 i = 0; i < IDN; i++) {
            Move[i] = this->memory_[ID[i]][SMBL_MOVE_STATUS - SMBL_NOW_POSITION_L];
        }
    }

    void FeeTechServo::syncFeedbackCurrent(const u8 ID[], const u8 IDN, int Current[]) {
        for (u8 i = 0; i < IDN; i++) {
            Current[i] = SMBL2Host(this->memory_[ID[i]][SMBL_NOW_CURRENT_L - SMBL_NOW_POSITION_L],
                                   this->memory_[ID[i]][SMBL_NOW_CURRENT_H - SMBL_NOW_POSITION_L]);

            if (!getError() && (Current[i] & (1 << 15))) {
                Current[i] = -(Current[i] & ~(1 << 15));
            }
        }
    }
}
