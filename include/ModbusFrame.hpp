/*
 * modbus.h
 *
 *  Created on: May 21, 2025
 *      Author: kdluzynski
 */

#ifndef INC_MODBUS_HPP_
#define INC_MODBUS_HPP_

#include <span>
#include <inttypes.h>
#include <cstring>
#include <ranges>
#include <cassert>
#include "ModbusFrameView.hpp"

namespace eModbus {


}
namespace eModbus {
    class Frame : public FrameView {
        class Exception : std::exception {
        };

    private:
        std::array<uint8_t, 300> _internalDataBuffer = {0};
        std::span<uint8_t> _dataBuffer() override {
            return std::span<uint8_t> (_internalDataBuffer);
        }

        std::span<const uint8_t> _dataBuffer() const override {
            return std::span<const uint8_t> (_internalDataBuffer);
        }

    public:

        Frame &setRawRtuData(std::span<uint8_t> RTU_Data, bool is_request, bool copy = true) {
            //			if(copy){
            isRequest(is_request);
            size_t copy_count = std::min(RTU_Data.size(), rtuBuffer().size());
            std::memcpy(rtuBuffer().data(), RTU_Data.data(), copy_count);
            MBAPLength(RTULengthWithoutCRC());
            //			}else{
            //				//TODO set flag that we are using external buffer for rtu buffer
            //				// set rtu buffer span
            //				_rtuDataBuffer = RTU_Data;
            //			}
            return *this;
        }

        Frame &setRawTcpData(std::span<const uint8_t> TCP_Data, bool is_request, bool copy = true) {
            //			if(copy){
            isRequest(is_request);
            size_t copy_count = std::min(TCP_Data.size(), _dataBuffer().size());
            std::memcpy(_dataBuffer().data(), TCP_Data.data(), copy_count);
            //			}else{
            //				//TODO set flag that we are using external buffer
            //				externalDataBuffer = true;
            //				_dataBuffer() = TCP_Data;
            //			}
            return *this;
        }


        static Frame fromRawTcpData(std::span<const uint8_t> TCP_Data, bool isRequest, bool copy = true) {
            Frame result;
            result.setRawTcpData(TCP_Data, isRequest, copy);
            return result;
        }

        static Frame fromRawRtuData(std::span<uint8_t> RTU_Data, bool isRequest, uint16_t transaction_ID = 0,
                                          bool copy = true) {
            Frame result;
            result.setRawRtuData(RTU_Data, isRequest, copy);
            return result;
        }

        Frame &clear() {
            std::memset(_dataBuffer().data(), 0, _dataBuffer().size());
            _dataBuffer() = std::span<uint8_t>(_internalDataBuffer);
            _isRequest = false;
            return *this;
        }


    };

    static std::string to_string(const Frame::ValidationStatus status) {
        switch (status) {
            case Frame::ValidationStatus::OK:return "OK";
            case Frame::ValidationStatus::InvalidCRC:return "Invalid CRC";
            case Frame::ValidationStatus::InvalidFunctionCode:return "Invalid Function Code";
            case Frame::ValidationStatus::ProtocolIdentifier:return "Protocol Identifier";
            case Frame::ValidationStatus::MBAPHeaderLengthInvalid:return "MBAP Header Length Invalid";
            default: return "Unknown";
        }

    }
}
#endif /* INC_MODBUS_HPP_ */
