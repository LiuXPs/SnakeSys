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

#ifndef SNAKE_FEETECH_DATA_H
#define SNAKE_FEETECH_DATA_H

#include <iostream>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>

namespace snake {
//--------SM85BL Data Type--------
    typedef int8_t s8;
    typedef int16_t s16;
    typedef int32_t s32;

    typedef uint8_t u8;
    typedef uint16_t u16;
    typedef uint32_t u32;

//--------SM85BL Command--------
#define FUN_PING 0x01
#define FUN_RESET 0x06
#define FUN_READ 0x02
#define FUN_WRITE 0x03
#define FUN_ASYNC_WRITE 0x04
#define FUN_ASYNC_ACTION 0x05
#define FUN_SYNC_READ 0x82
#define FUN_SYNC_WRITE 0x83

//--------SM85BL Baud Rate--------
#define SMBL_1M 0
#define SMBL_0_5M 1
#define SMBL_250K 2
#define SMBL_128K 3
#define SMBL_115200 4
#define SMBL_76800 5
#define SMBL_57600 6
#define SMBL_38400 7

//--------SM85BL EEPROM(Read only)--------
#define SMBL_FIRMWARE_MAJOR_VERSION 0
#define SMBL_FIRMWARE_MINOR_VERSION 1
#define SMBL_SERVO_MAJOR_VERSION 3
#define SMBL_SERVO_MINOR_VERSION 4

//--------SM85BL EEPROM(Read/Write)--------
#define SMBL_ID 5
#define SMBL_BAUD_RATE 6
#define SMBL_RETURN_DELAY_TIME 7
#define SMBL_RESPONSE_STATUS_LEVEL 8
#define SMBL_ANGLE_LIMIT_MIN_L 9
#define SMBL_ANGLE_LIMIT_MIN_H 10
#define SMBL_ANGLE_LIMIT_MAX_L 11
#define SMBL_ANGLE_LIMIT_MAX_H 12
#define SMBL_TEMPERATURE_LIMIT_MAX 13
#define SMBL_INPUT_VOLTAGE_MAX 14
#define SMBL_INPUT_VOLTAGE_MIN 15
#define SMBL_TORQUE_MAX_L 16
#define SMBL_TORQUE_MAX_H 17
#define SMBL_UNLOADING_CONDITIONS 19
#define SMBL_LED_ALARM_CONDITIONS 20
#define SMBL_P_FACTOR 21
#define SMBL_D_FACTOR 22
#define SMBL_I_FACTOR 23
#define SMBL_STARTING_FORCE_MIN_L 24
#define SMBL_STARTING_FORCE_MIN_H 25
#define SMBL_CLOCKWISE_DEAD_AREA 26
#define SMBL_COUNTERCLOCKWISE_DEAD_AREA 27
#define SMBL_PROTECTION_CURRENT_L 28
#define SMBL_PROTECTION_CURRENT_H 29
#define SMBL_ANGULAR_RESOLUTION 30
#define SMBL_POSITION_CORRECTION_L 31
#define SMBL_POSITION_CORRECTION_H 32
#define SMBL_OPERATION_MODE 33
#define SMBL_PROTECTION_TORQUE 34
#define SMBL_PROTECTION_TIME 35
#define SMBL_OVERLOAD_TORQUE 36

//--------SM85BL RAM(Read/Write)--------
#define SMBL_TORQUE_ENABLE 40
#define SMBL_GOAL_ACCELERATION 41
#define SMBL_GOAL_POSITION_L 42
#define SMBL_GOAL_POSITION_H 43
#define SMBL_GOAL_TIME_L 44
#define SMBL_GOAL_TIME_H 45
#define SMBL_GOAL_SPEED_L 46
#define SMBL_GOAL_SPEED_H 47
#define SMBL_TORQUE_LIMIT_L 48
#define SMBL_TORQUE_LIMIT_H 49
#define SMBL_EEPROM_LOCK 55

//--------SM85BL RAM(Read only)--------
#define SMBL_NOW_POSITION_L 56
#define SMBL_NOW_POSITION_H 57
#define SMBL_NOW_SPEED_L 58
#define SMBL_NOW_SPEED_H 59
#define SMBL_NOW_LOAD_L 60
#define SMBL_NOW_LOAD_H 61
#define SMBL_NOW_VOLTAGE 62
#define SMBL_NOW_TEMPERATURE 63
#define SMBL_MOVE_STATUS 66
#define SMBL_NOW_CURRENT_L 69
#define SMBL_NOW_CURRENT_H 70
}

#endif // SNAKE_FEETECH_DATA_H
