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
#include <iostream>
#include <unistd.h>

snake::FeeTechServo servo0;
snake::FeeTechServo servo1;

int main(int argc, char **argv) {
    std::string port = "/dev/ttyUSB0";
    snake::u32 rate = 1000000;
    snake::u32 timeout = 1000;

    servo0.setSerial(port, rate, timeout);
    servo1 = servo0;
    servo1.openSerial();

    for (snake::u8 id = 0; id < 254; id++) {
        if (servo1.ping(id) != -1) {
            std::cout << "ID:" << int(id) << std::endl;
        }
    }

    if (true) {
        int Pos[4];
        int Speed[4];
        int Load[4];
        int Voltage[4];
        int Temper[4];
        int Move[4];
        int Current[4];

        double t = 0;
        snake::u8 IDN = 4;
        snake::u8 ID[4] = {29, 30, 31, 32};
        snake::s16 pos1[4];
        snake::s16 pos2[4];
        snake::u16 v[4] = {0, 0, 0, 0};
        snake::u8 acc[4] = {0, 0, 0, 0};

        while (false) {
            for (snake::u8 i = 0; i < 4; i++) {
                pos2[i] = pos1[i];
            }
            for (snake::u8 i = 0; i < 4; i++) {
                pos1[i] = snake::s16(0.5 * 2047 * sin(360 / 360.0 * 3.1415926 * t) + 0.5 + 2047);
            }
            t += 0.05;
            for (snake::u8 i = 0; i < 4; i++) {
                v[i] = snake::u16(abs(pos1[i] - pos2[i]) / 0.05);
            }

            servo1.syncWritePosExe(ID, IDN, pos1, v, acc);

            if (servo1.syncFeedback(ID, IDN) != -1) {
                servo1.syncFeedbackPosition(ID, IDN, Pos);
                servo1.syncFeedbackSpeed(ID, IDN, Speed);
                servo1.syncFeedbackLoad(ID, IDN, Load);
                servo1.syncFeedbackVoltage(ID, IDN, Voltage);
                servo1.syncFeedbackTemperature(ID, IDN, Temper);
                servo1.syncFeedbackMoveStatus(ID, IDN, Move);
                servo1.syncFeedbackCurrent(ID, IDN, Current);
                std::cout << "pos = " << Pos[0] << " ";
                std::cout << "speed_ = " << Speed[1] << " ";
                std::cout << "Load = " << Load[1] << " ";
                std::cout << "Voltage = " << Voltage[2] << " ";
                std::cout << "Temper = " << Temper[2] << " ";
                std::cout << "Move = " << Move[3] << " ";
                std::cout << "Current = " << Current[3] << std::endl;
            } else {
                std::cout << "read err =" << std::endl;
                sleep(2);

            }
            usleep(50 * 1000);
        }
    } else {
        int Pos;
        int Speed;
        int Load;
        int Voltage;
        int Temper;
        int Move;
        int Current;
        double t = 0;

        snake::u8 ID = 32;
        snake::s16 p1 = 0;
        snake::s16 p2 = 0;
        snake::u16 v = 0;

        while (true) {
            p2 = p1;
            p1 = snake::s16(0.5 * 2047 * sin(360 / 360.0 * 3.1415926 * t) + 0.5 + 2047);
            t += 0.05;
            v = snake::u16(abs(p1 - p2) / 0.05);
            servo1.genWritePosExe(ID, p1, 0, 0);
            std::cout << "pos2 =" << p2 << " ";
            std::cout << "pos1 =" << p1 << " ";
            std::cout << "speed =" << v << " ";

            if (servo1.genFeedback(ID) != -1) {
                servo1.genFeedbackPosition(ID, Pos);
                servo1.genFeedbackSpeed(ID, Speed);
                servo1.genFeedbackLoad(ID, Load);
                servo1.genFeedbackVoltage(ID, Voltage);
                servo1.genFeedbackTemperature(ID, Temper);
                servo1.genFeedbackMoveStatus(ID, Move);
                servo1.genFeedbackCurrent(ID, Current);
                std::cout << "pos = " << Pos << " ";
                std::cout << "speed_ = " << Speed << " ";
                std::cout << "Load = " << Load << " ";
                std::cout << "Voltage = " << Voltage << " ";
                std::cout << "Temper = " << Temper << " ";
                std::cout << "Move = " << Move << " ";
                std::cout << "Current = " << Current << std::endl;
            } else {
                std::cout << "read err =" << std::endl;
                sleep(2);
            }
            usleep(50 * 1000);
        }
    }

    servo1.closeSerial();
    return 0;
}