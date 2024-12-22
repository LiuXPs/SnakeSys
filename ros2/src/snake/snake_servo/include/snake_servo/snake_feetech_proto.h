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

#ifndef SNAKE_FEETECH_PROTO_H
#define SNAKE_FEETECH_PROTO_H

#include "snake_servo/snake_feetech_data.h"

namespace snake {
    /*!
     * ----------------Error----------------
     * BIT       Name                               Detail
     * BIT7      0
     * BIT6      0
     * BIT5      Over load                          The output torque is less than the load and is set to 1
     * BIT4      0
     * BIT3      Over current                       The current exceeds the specified range and is set to 1
     * BIT2      over heated                        The temperature exceeds the specified range and is set to 1
     * BIT1      Angle error                        Angle sensor error and is set to 1
     * BIT0      Over voltage and under voltage     The voltage exceeds the specified range and is set to 1
     * ----------------Error----------------
     */

    class FeeTechProto : public serial::Serial {
    public:
        FeeTechProto();

        explicit FeeTechProto(u8 End);

        FeeTechProto(u8 End, u8 Level);

        ~FeeTechProto() override;

        FeeTechProto &operator=(const FeeTechProto &feetech_proto);

    private:
        /*!
         * Processor size side architecture.
         * 1: Large end structure.
         * 0: Small end structure.
         * Magnetic code series servo1 is low byte in front and high byte in back.
         */
        u8 end_;

        /*!
         * Servo return level.
         * 0: No reply packet is returned for other commands except read command and ping command.
         * 1: Return reply packet for all instructions.
         */
        u8 level_;

        /*!
         * Servo state.
         */
        u8 error_;
    public:
        /*!
         * Set serial parameters.
         * @param port
         * @param rate
         * @param timeout
         */
        void setSerial(const std::basic_string<char> &port, u32 rate, u32 timeout);

        /*!
         * Open serial port.
         */
        void openSerial();

        /*!
         * Close serial port.
         */
        void closeSerial();

    public:
        /*!
         * Set End value.
         * @param End
         */
        void setEnd(u8 End);

        /*!
         * Set level_ value.
         * @param Level
         */
        void setLevel(u8 Level);

        /*!
         * Set Error value.
         * @param Error
         */
        void setError(u8 Error);

        /*!
         * Get End value.
         * @return End value.
         */
        [[nodiscard]] u8
        getEnd() const;

        /*!
         * Get level_ value.
         * @return level_ value.
         */
        [[nodiscard]] u8
        getLevel() const;

        /*!
         * Get Error value.
         * @return Error value.
         */
        [[nodiscard]] u8
        getError() const;

        /*!
         * One 16 bits data split into two 8 bits data.
         * @param DataL Low address data.
         * @param DataH High address data.
         * @param Data One 16 bits data.
         */
        void Host2SMBL(u8 *DataL, u8 *DataH, u16 Data);

        /*!
         * Two 8 bits data are combined into one 16 bits data.
         * @param DataL Low address data.
         * @param DataH High address data.
         * @return One 16 bits data.
         */
        u16 SMBL2Host(u8 DataL, u8 DataH);

        /*!
         * Write buffer.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @param nData Write data.
         * @param nLen Data length.
         * @param Fun Servo command.
         */
        void writeBuf(u8 ID, u8 MemAddr, const u8 *nData, u8 nLen, u8 Fun);

        /*!
         * Return reply.
         * @param ID Servo ID.
         * @return Data length.
         * Timeout return -1.
         */
        int ack(u8 ID);

    public:
        /*!
         * Ping command.
         * @param ID Servo ID.
         * The ping command uses the broadcast ID (0xFE), and the servo1 also returns the response information.
         * @return Servo state.
         * Timeout return -1.
         */
        int ping(u8 ID = 0xFE);

        /*!
         * Reset command.
         * @param ID Servo ID.
         * @return Servo state.
         * Timeout return -1.
         */
        int reset(u8 ID);

        /*!
         * General read command.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @param nData Read data.
         * @param nLen Data length.
         * @return Data length.
         * Timeout return -1.
         */
        int genRead(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);

        /*!
         * General write command.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @param nData Write data.
         * @param nLen Data length.
         * @return Data length.
         * Timeout return -1.
         */
        int genWrite(u8 ID, u8 MemAddr, const u8 *nData, u8 nLen);

        /*!
         * Asynchronous write command.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @param nData Write data.
         * @param nLen Data length.
         * @return Data length.
         * Timeout return -1.
         */
        int asyncWrite(u8 ID, u8 MemAddr, const u8 *nData, u8 nLen);

        /*!
         * Asynchronous write action command.
         * @param ID Servo ID.
         * When sending the action command to multiple servo1, the broadcast ID (0xFE) is used.
         * Therefore, no data frame will be returned when sending this command.
         * @return Data length.
         * Timeout return -1.
         */
        int asyncWriteAction(u8 ID = 0xFE);

        /*!
         * Synchronous read command.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param MemAddr First address of data in memory table.
         * @param nData Read data.
         * @param nLen Data length.
         */
        int syncRead(const u8 ID[], u8 IDN, u8 MemAddr, u8 *nData, u8 nLen);

        /*!
         * Synchronous write command.
         * @param ID Servo ID.
         * @param IDN Servo number.
         * @param MemAddr First address of data in memory table.
         * @param nData Write data.
         * @param nLen Data length.
         */
        int syncWrite(const u8 ID[], u8 IDN, u8 MemAddr, const u8 *nData, u8 nLen);

        /*!
         * Write one byte.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @param bData One byte data.
         * @return Data length.
         * Timeout return -1.
         */
        int writeByte(u8 ID, u8 MemAddr, u8 bData);

        /*!
         * Write two bytes.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @param wData Two bytes data.
         * @return Data length.
         * Timeout return -1.
         */
        int writeWord(u8 ID, u8 MemAddr, u16 wData);

        /*!
         * Read one byte.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @return Byte data.
         * Timeout return -1.
         */
        u8 readByte(u8 ID, u8 MemAddr);

        /*!
         * Read two bytes.
         * @param ID Servo ID.
         * @param MemAddr First address of data in memory table.
         * @return Word data.
         * Timeout return -1.
         */
        u16 readWord(u8 ID, u8 MemAddr);
    };
}

#endif // SNAKE_FEETECH_PROTO_H
