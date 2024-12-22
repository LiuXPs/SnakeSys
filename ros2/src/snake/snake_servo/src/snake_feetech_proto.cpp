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

#include "snake_servo/snake_feetech_proto.h"

namespace snake {
    FeeTechProto::FeeTechProto() {
        this->end_ = 0;
        this->level_ = 1;
        this->error_ = 0;
    }

    FeeTechProto::FeeTechProto(const u8 End) {
        this->end_ = End;
        this->level_ = 1;
        this->error_ = 0;
    }

    FeeTechProto::FeeTechProto(const u8 End, const u8 Level) {
        this->end_ = End;
        this->level_ = Level;
        this->error_ = 0;
    }

    FeeTechProto::~FeeTechProto() {
        if (isOpen()) {
            close();
        }
    }

    FeeTechProto &FeeTechProto::operator=(const FeeTechProto &feetech_proto) {
        if (this == &feetech_proto) {
            return *this;
        }

        this->setEnd(feetech_proto.getEnd());
        this->setLevel(feetech_proto.getLevel());
        this->setError(feetech_proto.getError());
        this->setSerial(feetech_proto.getPort(),
                        feetech_proto.getBaudrate(),
                        feetech_proto.getTimeout().read_timeout_constant);
        return *this;
    }

    void FeeTechProto::setSerial(const std::basic_string<char> &port, u32 rate, u32 timeout) {
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
        setPort(port);
        setBaudrate(rate);
        setTimeout(to);
    }

    void FeeTechProto::openSerial() {
        try {
            open();
        }
        catch (serial::IOException &e) {
            std::cout << "Unable to open serial port ..." << std::endl;
        }

        if (isOpen()) {
            std::cout << "Serial Port initialized ..." << std::endl;
        }
    }

    void FeeTechProto::closeSerial() {
        if (isOpen()) {
            close();
            std::cout << "Serial Port closed ..." << std::endl;
        }
    }

    void FeeTechProto::setEnd(const u8 End) {
        this->end_ = End;
    }

    void FeeTechProto::setLevel(const u8 Level) {
        this->level_ = Level;
    }

    void FeeTechProto::setError(const u8 Error) {
        this->error_ = Error;
    }

    u8 FeeTechProto::getEnd() const {
        return this->end_;
    }

    u8 FeeTechProto::getLevel() const {
        return this->level_;
    }

    u8 FeeTechProto::getError() const {
        return this->error_;
    }

    void FeeTechProto::Host2SMBL(u8 *const DataL, u8 *const DataH, const u16 Data) {
        if (getEnd()) {
            *DataL = (Data >> 8);
            *DataH = (Data & 0xff);
        } else {
            *DataH = (Data >> 8);
            *DataL = (Data & 0xff);
        }
    }

    u16 FeeTechProto::SMBL2Host(const u8 DataL, const u8 DataH) {
        u16 w_data;
        if (getEnd()) {
            w_data = DataL;
            w_data <<= 8;
            w_data |= DataH;
        } else {
            w_data = DataH;
            w_data <<= 8;
            w_data |= DataL;
        }
        return w_data;
    }

    void FeeTechProto::writeBuf(const u8 ID, const u8 MemAddr, const u8 *const nData, const u8 nLen, const u8 Fun) {
        size_t msg_len = 2;
        u8 b_buf[6];
        u8 check_sum = 0;
        b_buf[0] = 0xff;
        b_buf[1] = 0xff;
        b_buf[2] = ID;
        b_buf[4] = Fun;

        if (nData) {
            msg_len += nLen + 1;
            b_buf[3] = msg_len;
            b_buf[5] = MemAddr;

            write(b_buf, 6);
            write(nData, nLen);

            check_sum = ID + msg_len + Fun + MemAddr;
            for (u8 i = 0; i < nLen; i++) {
                check_sum += nData[i];
            }
            check_sum = ~check_sum;
            write(&check_sum, 1);
        } else {
            b_buf[3] = msg_len;
            write(b_buf, 5);

            check_sum = ID + msg_len + Fun;
            check_sum = ~check_sum;
            write(&check_sum, 1);
        }
    }

    int FeeTechProto::ack(const u8 ID) {
        setError(0);
        if (ID != 0xfe && getLevel()) {
            u8 b_buf[6];
            u8 cal_sum = 0;
            size_t cal_size = read(b_buf, 6);
            if (cal_size != 6) {
                return -1;
            }
            if (b_buf[0] != 0xff || b_buf[1] != 0xff) {
                return -1;
            }
            if (b_buf[2] != ID) {
                return -1;
            }
            if (b_buf[3] != 2) {
                return -1;
            }
            for (size_t i = 2; i < (cal_size - 1); i++) {
                cal_sum += b_buf[i];
            }
            cal_sum = ~cal_sum;
            if (cal_sum != b_buf[cal_size - 1]) {
                return -1;
            }
            setError(b_buf[4]);
        }
        return 1;
    }

    int FeeTechProto::ping(const u8 ID) {
        flushInput();
        writeBuf(ID, 0, nullptr, 0, FUN_PING);
        setError(0);

        u8 b_buf[6];
        u8 cal_sum = 0;
        flushOutput();
        size_t cal_size = read(b_buf, 6);
        if (cal_size != 6) {
            return -1;
        }
        if (b_buf[0] != 0xff || b_buf[1] != 0xff) {
            return -1;
        }
        if (b_buf[2] != ID) {
            return -1;
        }
        if (b_buf[3] != 2) {
            return -1;
        }
        for (size_t i = 2; i < (cal_size - 1); i++) {
            cal_sum += b_buf[i];
        }
        cal_sum = ~cal_sum;
        if (cal_sum != b_buf[cal_size - 1]) {
            return -1;
        }
        setError(b_buf[4]);
        return b_buf[4];
    }

    int FeeTechProto::reset(const u8 ID) {
        flushInput();
        writeBuf(ID, 0, nullptr, 0, FUN_RESET);
        setError(0);

        u8 b_buf[6];
        u8 cal_sum = 0;
        flushOutput();
        size_t cal_size = read(b_buf, 6);
        if (cal_size != 6) {
            return -1;
        }
        if (b_buf[0] != 0xff || b_buf[1] != 0xff) {
            return -1;
        }
        if (b_buf[3] != 2) {
            return -1;
        }
        for (size_t i = 2; i < (cal_size - 1); i++) {
            cal_sum += b_buf[i];
        }
        cal_sum = ~cal_sum;
        if (cal_sum != b_buf[cal_size - 1]) {
            return -1;
        }
        setError(b_buf[4]);
        return b_buf[4];
    }

    int FeeTechProto::genRead(const u8 ID, const u8 MemAddr, u8 *const nData, const u8 nLen) {
        flushInput();
        writeBuf(ID, MemAddr, &nLen, 1, FUN_READ);

        u8 b_buf[255];
        u8 cal_sum = 0;
        flushOutput();
        size_t cal_size = read(b_buf, nLen + 6);
        if (cal_size != static_cast<size_t>(nLen + 6)) {
            return -1;
        }
        if (b_buf[0] != 0xff || b_buf[1] != 0xff) {
            return -1;
        }
        if (b_buf[2] != ID) {
            return -1;
        }
        for (size_t i = 2; i < (cal_size - 1); i++) {
            cal_sum += b_buf[i];
        }
        cal_sum = ~cal_sum;
        if (cal_sum != b_buf[cal_size - 1]) {
            return -1;
        }
        memcpy(nData, b_buf + 5, nLen);
        setError(b_buf[4]);
        return nLen;
    }

    int FeeTechProto::genWrite(const u8 ID, const u8 MemAddr, const u8 *const nData, const u8 nLen) {
        flushInput();
        writeBuf(ID, MemAddr, nData, nLen, FUN_WRITE);
        flushOutput();
        return ack(ID);
    }

    int FeeTechProto::asyncWrite(const u8 ID, const u8 MemAddr, const u8 *const nData, const u8 nLen) {
        flushInput();
        writeBuf(ID, MemAddr, nData, nLen, FUN_ASYNC_WRITE);
        flushOutput();
        return ack(ID);
    }

    int FeeTechProto::asyncWriteAction(const u8 ID) {
        flushInput();
        writeBuf(ID, 0, nullptr, 0, FUN_ASYNC_ACTION);
        flushOutput();
        return ack(ID);
    }

    int FeeTechProto::syncRead(const u8 ID[], const u8 IDN, const u8 MemAddr, u8 *const nData, const u8 nLen) {
        flushInput();
        u8 msg_len = IDN + 4;
        u8 check_sum = 0;
        u8 b_buf[7];
        b_buf[0] = 0xff;
        b_buf[1] = 0xff;
        b_buf[2] = 0xfe;
        b_buf[3] = msg_len;
        b_buf[4] = FUN_SYNC_READ;
        b_buf[5] = MemAddr;
        b_buf[6] = nLen;
        write(b_buf, 7);

        check_sum = 0xfe + msg_len + FUN_SYNC_READ + MemAddr + nLen;
        for (u8 i = 0; i < IDN; i++) {
            write(ID + i, 1);
            check_sum += ID[i];
        }
        check_sum = ~check_sum;
        write(&check_sum, 1);

        flushOutput();
        for (u8 i = 0; i < IDN; i++) {
            u8 b_buf[255];
            u8 cal_sum = 0;
            size_t cal_size = read(b_buf, nLen + 6);
            if (cal_size != static_cast<size_t>(nLen + 6)) {
                return -1;
            }
            if (b_buf[0] != 0xff || b_buf[1] != 0xff) {
                return -1;
            }
            if (b_buf[2] != ID[i]) {
                return -1;
            }
            for (size_t j = 2; j < (cal_size - 1); j++) {
                cal_sum += b_buf[j];
            }
            cal_sum = ~cal_sum;
            if (cal_sum != b_buf[cal_size - 1]) {
                return -1;
            }
            memcpy(nData + ID[i] * nLen, b_buf + 5, nLen);
            setError(b_buf[4]);
        }
        return nLen;
    }

    int FeeTechProto::syncWrite(const u8 ID[], const u8 IDN, const u8 MemAddr, const u8 *const nData, const u8 nLen) {
        flushInput();
        if (((nLen + 1) * IDN + 4) > 0xff) {
            std::cout << "ProtocolError: Length cannot be greater than 0xff" << std::endl;
            throw (abort);
        }
        u8 msg_len = ((nLen + 1) * IDN + 4);
        u8 check_sum = 0;
        u8 b_buf[7];
        b_buf[0] = 0xff;
        b_buf[1] = 0xff;
        b_buf[2] = 0xfe;
        b_buf[3] = msg_len;
        b_buf[4] = FUN_SYNC_WRITE;
        b_buf[5] = MemAddr;
        b_buf[6] = nLen;
        write(b_buf, 7);

        check_sum = 0xfe + msg_len + FUN_SYNC_WRITE + MemAddr + nLen;
        for (u8 i = 0; i < IDN; i++) {
            write(ID + i, 1);
            write(nData + i * nLen, nLen);
            check_sum += ID[i];
            for (u8 j = 0; j < nLen; j++) {
                check_sum += nData[i * nLen + j];
            }
        }
        check_sum = ~check_sum;
        write(&check_sum, 1);
        flushOutput();
        return nLen;
    }

    int FeeTechProto::writeByte(const u8 ID, const u8 MemAddr, const u8 bData) {
        return genWrite(ID, MemAddr, &bData, 1);
    }

    int FeeTechProto::writeWord(const u8 ID, const u8 MemAddr, const u16 wData) {
        u8 b_buf[2];
        Host2SMBL(b_buf + 0, b_buf + 1, wData);
        return genWrite(ID, MemAddr, b_buf, 2);
    }

    u8 FeeTechProto::readByte(const u8 ID, const u8 MemAddr) {
        u8 b_data;
        size_t cal_size = genRead(ID, MemAddr, &b_data, 1);
        if (cal_size != 1) {
            return -1;
        } else {
            return b_data;
        }
    }

    u16 FeeTechProto::readWord(const u8 ID, const u8 MemAddr) {
        u8 n_data[2];
        u16 w_data;
        size_t cal_size = genRead(ID, MemAddr, n_data, 2);
        if (cal_size != 2) {
            return -1;
        } else {
            w_data = SMBL2Host(n_data[0], n_data[1]);
            return w_data;
        }
    }
}
