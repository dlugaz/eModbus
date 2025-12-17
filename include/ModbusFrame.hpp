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
    static constexpr size_t MAX_MODBUS_FRAME_SIZE = 300;
    private:
        std::array<uint8_t, MAX_MODBUS_FRAME_SIZE> _internalBuffer = {0};
        std::span<uint8_t> _dataBuffer() override {
            return std::span<uint8_t> (_internalBuffer);
        }

        std::span<const uint8_t> _dataBuffer() const override {
            return std::span<const uint8_t> (_internalBuffer);
        }

    public:
        explicit Frame(bool isRequest) : FrameView(Frame::_dataBuffer(),isRequest,true) {
        };
        explicit Frame(FrameView& view)
            : FrameView(Frame::_dataBuffer(), view.isRequest(), true)
        {
            if (view.isTCPFrame()) {
                this->setRawTcpData(view.buffer(),view.isRequest());
            }else {
                this->setRawRtuData(view.rtuBuffer(),view.isRequest());
            }
            const std::span<const uint8_t> source_buffer = view.buffer();
            const size_t copy_count = std::min(source_buffer.size(), _internalBuffer.size());
            std::memcpy(_internalBuffer.data(), source_buffer.data(), copy_count);
        }

        Frame &setRawRtuData(std::span<uint8_t> RTU_Data, bool is_request) {
            isRequest(is_request);
            size_t copy_count = std::min(RTU_Data.size(), rtuBuffer().size());
            std::memcpy(rtuBuffer().data(), RTU_Data.data(), copy_count);
            MBAPLength(RTULengthWithoutCRC());
            return *this;
        }

        Frame &setRawTcpData(std::span<const uint8_t> TCP_Data, bool is_request) {
            isRequest(is_request);
            size_t copy_count = std::min(TCP_Data.size(), _dataBuffer().size());
            std::memcpy(_dataBuffer().data(), TCP_Data.data(), copy_count);
            return *this;
        }


        static Frame fromRawTcpData(std::span<const uint8_t> TCP_Data, bool isRequest) {
            Frame result(isRequest);
            result.setRawTcpData(TCP_Data, isRequest);
            return result;
        }

        static Frame fromRawRtuData(std::span<uint8_t> RTU_Data, bool isRequest) {
            Frame result(isRequest);
            result.setRawRtuData(RTU_Data, isRequest);
            return result;
        }
        static Frame build(bool isRequest, uint8_t slave_ID, FunctionCode function_code, uint16_t start_address,
                                 uint16_t register_count, std::span<uint16_t> registers_values = {},
                                 uint16_t transaction_ID = 0) {
            Frame frame(isRequest);
            frame.rebuild(isRequest, slave_ID, function_code, start_address, register_count, registers_values,
                          transaction_ID);
            return frame;
        }
        static Frame buildExceptionResponse(uint8_t slaveID, FunctionCode function_code, ExceptionCode exception_code,
                                                  uint16_t transaction_ID = 0) {
            Frame frame(false);
            frame.rebuildExceptionResponse(slaveID, function_code, exception_code, transaction_ID);
            return frame;
        }

    };

}
#endif /* INC_MODBUS_HPP_ */
