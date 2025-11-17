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

namespace eModbus {
    inline char nibbleToHexChar(uint8_t nibble) {
        if (nibble < 10) {
            return '0' + nibble;
        } else {
            return 'A' + (nibble - 10);
        }
    }

    inline std::string toString(std::span<uint8_t> _dataBuffer) {
        std::string result;
        result.reserve(3 * _dataBuffer.size());
        for (uint8_t byte: _dataBuffer) {
            result += nibbleToHexChar(byte >> 4);
            result += nibbleToHexChar(byte & 0x0F);
            result += ' ';
        }
        return result;
    }

}
namespace eModbus {
    class Frame {
        class Exception : std::exception {
        };

    private:
        std::array<uint8_t, 300> _internalDataBuffer = {0};

        bool _isRequest = false;

        // Non-const getter for mutable access
        auto _dataBuffer() {
            return std::span<uint8_t> (_internalDataBuffer);
        }

        // Const getter for read-only access
        auto _dataBuffer() const {
            return std::span<const uint8_t> (_internalDataBuffer);
        }
        enum FRAME_POS {
            TRANSACTION_ID = 0,
            PROTOCOL_ID = 2,
            LENGTH = 4,
            UNIT_ID = 6,
            FUNCTION_CODE = 7,
            DATA = 8,
            EXCEPTION_CODE = DATA,
            START_ADDRESS = DATA,
            BYTE_COUNT = DATA,
            REGISTER_DATA = BYTE_COUNT + 1,
            REGISTER_COUNT = START_ADDRESS + 2,
            BYTE_COUNT_MULTIPLE_REGISTERS = REGISTER_COUNT + 2,
            REGISTER_DATA_WRITE_SINGLE = START_ADDRESS + 2,
            REGISTER_DATA_WRITE_MULTIPLE = BYTE_COUNT_MULTIPLE_REGISTERS + 1,
        };

        static uint16_t betole(const uint8_t *bigendiandata) {
            return static_cast<uint16_t>(bigendiandata[0] << 8) | (bigendiandata[1] & 0xFF);
        }

        static void letobe(const uint16_t littleendiandata, uint8_t *data) {
            data[0] = static_cast<uint8_t>((littleendiandata >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>((littleendiandata) & 0xFF);
        }

        uint16_t RTULengthWithoutCRC() const {
            int result = calculateRTULength() - CRC_SIZE;
            return (result >= 0) ? static_cast<uint16_t>(result) : 0;
        }

    public:
        enum FunctionCode {
            ReadCoils = 0x01,
            ReadDiscreteInputs = 0x02,
            ReadHoldingRegisters = 0x03,
            ReadInputRegisters = 0x04,
            WriteSingleCoil = 0x05,
            WriteSingleRegister = 0x06,
            WriteMultipleCoils = 0x0F,
            WriteMultipleRegisters = 0x10,
            Diagnostics = 0x08,
            MaskWriteRegister = 0x16,
            ReadWriteMultipleRegisters = 0x17,
            ReadDeviceIdentification = 0x0E,
            Invalid = 0
        };


        static uint16_t calculateModbusCRC(const std::span<const uint8_t> data) {
            static constexpr uint16_t table[256] = {
                0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
                0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
                0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
                0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
                0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
                0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
                0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
                0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
                0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
                0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
                0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
                0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
                0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
                0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
                0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
                0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
                0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
                0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
                0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
                0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
                0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
                0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
                0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
                0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
                0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
                0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
                0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
                0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
                0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
                0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
            };

            uint8_t xor_ = 0;
            uint16_t crc = 0xFFFF;
            for (uint8_t byte: data) {
                xor_ = byte ^ crc;
                crc >>= 8;
                crc ^= table[xor_];
            }
            return crc;
        }

        uint16_t calculateModbusCRC() const {
            return calculateModbusCRC(_dataBuffer().subspan(RTU_HEADER_START_POSITION, RTULengthWithoutCRC()));
        }

        void appendCRC() {
            crc(calculateModbusCRC());
        }

        uint16_t crcPosition() const {
            return RTU_HEADER_START_POSITION + RTULengthWithoutCRC();
        }

        uint16_t crc() const {
            const uint16_t crcPos = crcPosition();
            uint16_t crcVal = _dataBuffer()[crcPos] | (_dataBuffer()[crcPos + 1] << 8);
            //			return betole(&_dataBuffer()[crcPos]);
            return crcVal;
        }

        void crc(uint16_t value) {
            const uint16_t crcPos = crcPosition();
            _dataBuffer()[crcPos] = value & 0xFF;
            _dataBuffer()[crcPos + 1] = (value >> 8) & 0xFF;
            //			letobe(value,&_dataBuffer()[crcPos]);
        }


        std::span<uint8_t> buffer()  {
            return _dataBuffer();
        }

        void buffer(std::span<uint8_t> newBuffer) {
            _dataBuffer() = newBuffer;
        }

        std::span<uint8_t> rtuBuffer()  {
            return _dataBuffer().subspan(RTU_HEADER_START_POSITION);
        }

        // explicit ModbusFrame () = default;
        // explicit ModbusFrame(std::span<uint8_t> data_buffer)
        // :_dataBuffer()(data_buffer){
        // }
        // explicit ModbusFrame()
        //     {
        // }


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

        bool isRequest() const {
            return _isRequest;
        }

        Frame &isRequest(bool is_request) {
            _isRequest = is_request;
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


        uint16_t transactionID() const {
            return betole(&_dataBuffer()[TRANSACTION_ID]);
        }

        Frame &transactionID(uint16_t value) {
            letobe(value, &_dataBuffer()[TRANSACTION_ID]);
            return *this;
        }

        uint16_t protocolID() const {
            return betole(&_dataBuffer()[PROTOCOL_ID]);
        }

        Frame &protocolID(uint16_t value) {
            letobe(value, &_dataBuffer()[PROTOCOL_ID]);
            return *this;
        }

        uint16_t MBAPLength() const {
            return betole(&_dataBuffer()[LENGTH]);
        }

        Frame &MBAPLength(uint16_t value) {
            letobe(value, &_dataBuffer()[LENGTH]);
            return *this;
        }

        uint16_t RTULength() const {
            return MBAPLength() + CRC_SIZE;
        }

        uint16_t pduLength() const {
            uint16_t len = MBAPLength();
            if (len == 0)
                len = RTULengthWithoutCRC();
            return len - UNIT_ID_SIZE;
        }

        Frame &slaveID(uint8_t value) {
            _dataBuffer()[UNIT_ID] = value;
            return *this;
        }

        uint8_t slaveID() const {
            return _dataBuffer()[UNIT_ID];
        }

        FunctionCode functionCode() const {
            return static_cast<FunctionCode>(_dataBuffer()[FUNCTION_CODE] & 0x7F);
        }

        Frame &functionCode(FunctionCode value) {
            _dataBuffer()[FUNCTION_CODE] = static_cast<uint8_t>(value);
            return *this;
        }

        bool hasStartAddress() const {
            if (isException())
                return false;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return _isRequest;
                case WriteSingleCoil:
                case WriteSingleRegister:
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return true;
                default:
                    return false;
            }
        }

        uint16_t startAddress() const {
            if (!hasStartAddress())
                return 0;
            return betole(&_dataBuffer()[START_ADDRESS]);
        }

        Frame &startAddress(uint16_t value) {
            if (hasStartAddress())
                letobe(value, &_dataBuffer()[START_ADDRESS]);
            return *this;
        }

        uint16_t byteCount() const {
            if (isException())
                return 0;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return _isRequest ? 0 : _dataBuffer()[BYTE_COUNT];
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return _isRequest ? _dataBuffer()[BYTE_COUNT_MULTIPLE_REGISTERS] : 0;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return 2;
                default:
                    return 0;
            }
        }

        Frame &byteCount(uint8_t value) {
            if (!isException()) {
                switch (functionCode()) {
                    case ReadCoils:
                    case ReadDiscreteInputs:
                    case ReadHoldingRegisters:
                    case ReadInputRegisters:
                        if (!_isRequest)
                            _dataBuffer()[BYTE_COUNT] = value;
                        break;
                    case WriteMultipleCoils:
                    case WriteMultipleRegisters:
                        if (_isRequest)
                            _dataBuffer()[BYTE_COUNT_MULTIPLE_REGISTERS] = value;
                        break;
                    default:
                    case WriteSingleCoil:
                    case WriteSingleRegister:
                        break;
                }
            }
            return *this;
        }

        uint16_t registerCount() const {
            if (isException())
                return 0;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                    return _isRequest ? betole(&_dataBuffer()[REGISTER_COUNT]) : (byteCount() * 8);
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return _isRequest ? betole(&_dataBuffer()[REGISTER_COUNT]) : (byteCount() / 2);
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return 1;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return betole(&_dataBuffer()[REGISTER_COUNT]);
                default:
                    return 0;
            }
        }

        Frame &registerCount(uint16_t value) {
            if (!isException()) {
                switch (functionCode()) {
                    case ReadCoils:
                    case ReadDiscreteInputs:
                    case ReadHoldingRegisters:
                    case ReadInputRegisters:
                        if (_isRequest)
                            letobe(value, &_dataBuffer()[REGISTER_COUNT]);
                        break;
                    case WriteMultipleCoils:
                    case WriteMultipleRegisters:
                        letobe(value, &_dataBuffer()[REGISTER_COUNT]);
                        break;
                    case WriteSingleCoil:
                    case WriteSingleRegister:
                    default:
                        break;
                }
            }
            return *this;
        }

        enum ExceptionCode {
            IllegalFunction = 0x01,
            IllegalDataAddress = 0x02,
            IllegalDataValue = 0x03,
            SlaveDeviceFailure = 0x04,
            Acknowledge = 0x05,
            SlaveDeviceBusy = 0x06,
            NegativeAcknowledge = 0x07,
            MemoryParityError = 0x08,
        };

        bool isException() const {
            return (_dataBuffer()[FUNCTION_CODE] & 0x80) != 0;
        }

        Frame &isException(bool setFlag) {
            if (setFlag) {
                _isRequest = false;
                _dataBuffer()[FUNCTION_CODE] |= 0x80;
            } else
                _dataBuffer()[FUNCTION_CODE] &= ~0x80;

            return *this;
        }

        ExceptionCode exceptionCode() const {
            return static_cast<ExceptionCode>(isException() ? _dataBuffer()[EXCEPTION_CODE] : 0);
        }

        Frame &exceptionCode(ExceptionCode exception_code) {
            _dataBuffer()[EXCEPTION_CODE] = exception_code;
            return *this;
        }

        enum class ValidationStatus {
            OK,
            ProtocolIdentifier,
            MBAPHeaderLengthInvalid,
            InvalidCRC,
            TransactionID,
            InvalidFunctionCode,
            Unknown,
        };

        ValidationStatus validateTCP() const {
            if (protocolID() != 0)
                return ValidationStatus::ProtocolIdentifier;

            if (MBAPLength() == 0)
                return ValidationStatus::MBAPHeaderLengthInvalid;

            return validateCommon();
        }

        ValidationStatus validateCommon() const {
            uint8_t function_code = static_cast<uint8_t>(functionCode());
            if (function_code == 0)
                return ValidationStatus::InvalidFunctionCode;
            return ValidationStatus::OK;
        }

        ValidationStatus validateRTU() const {
            ValidationStatus commonValidation = validateCommon();
            if (commonValidation != ValidationStatus::OK)
                return commonValidation;
            if (crc() != calculateModbusCRC()) {
                return ValidationStatus::InvalidCRC;
            }

            return ValidationStatus::OK;
        }

        Frame &clear() {
            std::memset(_dataBuffer().data(), 0, _dataBuffer().size());
            _dataBuffer() = std::span<uint8_t>(_internalDataBuffer);
            _isRequest = false;
            return *this;
        }

        std::span<const uint8_t> rtuFrame() {
            uint16_t rtuLength = calculateRTULength();
            appendCRC();
            return _dataBuffer().subspan(RTU_HEADER_START_POSITION, rtuLength);
        }

        int tcpFrameSize() const {
            return MBAP_HEADER_SIZE + pduLength();
        }

        std::span<const uint8_t> tcpFrame() {
            // if (MBAPLength() == 0) {
            MBAPLength(RTULengthWithoutCRC());
            // }
            return _dataBuffer().subspan(0, tcpFrameSize());
        }

        std::span<uint8_t> registersData()  {
            if (!hasRegistersValues())
                return {};
            uint16_t data_pos = 0;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    data_pos = FRAME_POS::REGISTER_DATA;
                    break;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    data_pos = FRAME_POS::REGISTER_DATA_WRITE_SINGLE;
                    break;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    data_pos = FRAME_POS::REGISTER_DATA_WRITE_MULTIPLE;
                    break;
                default:
                    break;
            }
            return _dataBuffer().subspan(data_pos, byteCount());
        }

        static uint16_t swap_bytes(uint16_t val) {
            return (val << 8) | (val >> 8);
        }

        bool hasRegistersValues() const {
            if (isException())
                return false;
            switch (functionCode()) {
                default:
                    return false;
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return !_isRequest;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return true;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return _isRequest;
            }
        }

        std::vector<uint16_t> registersValues()  {
            const std::span<uint8_t> byte_span = registersData(); // Assuming this is valid for the call's duration
            std::vector<uint16_t> result;
            if (functionCode() == ReadCoils || functionCode() == ReadDiscreteInputs) {
                result.reserve(byte_span.size() * 8); // Reserve for all bits
                for (unsigned int i = 0; i < byte_span.size() * 8; ++i) {
                    // Corrected iota range
                    size_t bit_index = i % 8;
                    size_t byte_index = i / 8;
                    bool bit_value = (byte_span[byte_index] >> bit_index) & 0x1;
                    result.push_back(static_cast<uint16_t>(bit_value ? 0xFF00 : 0));
                }
            } else {
                result.reserve(byte_span.size() / 2);
                for (unsigned int i = 0; i < byte_span.size() / 2; ++i) {
                    size_t idx1 = i * 2;
                    size_t idx2 = i * 2 + 1;

                    uint16_t val = static_cast<uint16_t>(byte_span[idx1]) |
                                   (static_cast<uint16_t>(byte_span[idx2]) << 8); // Assuming little-endian in byte_span

                    result.push_back(swap_bytes(val));
                }
            }
            return result;
        }

        Frame &registersValues(std::span<uint16_t> values) {
            if (hasRegistersValues()) {
                std::span<uint8_t> registers_data = registersData();
                for (uint16_t i = 0; i < registers_data.size(); i += 2) {
                    size_t pos_values = i / 2;
                    if (pos_values >= values.size())
                        break;
                    registers_data[i] = values[pos_values] >> 8;
                    registers_data[i + 1] = values[pos_values] & 0xFF;
                }
            }
            return *this;
        }

        static void tests() {
            std::vector<uint8_t> testData = {0x04, 0x01, 0x00, 0x0a, 0x00, 0x0d, 0xdd, 0x98};
            Frame frame = Frame::fromRawRtuData(testData, true);
            assert(frame.RTULength() == 8);
            assert(frame.slaveID()==0x04);
            assert(frame.functionCode() == FunctionCode::ReadCoils);
            assert(frame.startAddress() == 10);
            assert(frame.registerCount() == 13);
            assert(frame.validateRTU() == ValidationStatus::OK);

            testData = {0x04, 0x01, 0x02, 0x0a, 0x11, 0xb3, 0x50};
            frame = Frame::fromRawRtuData(testData, false);
            assert(frame.RTULength() == 7);
            assert(frame.slaveID()==0x04);
            assert(frame.functionCode() == FunctionCode::ReadCoils);
            assert(frame.byteCount() == 2);
            assert(frame.registersData()[0] == 0x0a);
            assert(frame.registersData()[1] == 0x11);
            assert(frame.validateRTU() == ValidationStatus::OK);

            testData = {0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xcb};
            frame = Frame::fromRawRtuData(testData, true);
            assert(frame.RTULength() == 8);
            assert(frame.slaveID()==0x01);
            assert(frame.functionCode() == FunctionCode::ReadInputRegisters);
            assert(frame.startAddress() == 0);
            assert(frame.registerCount() == 2);
            assert(frame.byteCount() == 0);
            assert(frame.validateRTU() == ValidationStatus::OK);

            testData = {0x01, 0x04, 0x04, 0x00, 0x06, 0x00, 0x05, 0xdb, 0x86};
            frame = Frame::fromRawRtuData(testData, false);
            assert(frame.RTULength() == 9);
            assert(frame.slaveID()==0x01);
            assert(frame.functionCode() == FunctionCode::ReadInputRegisters);
            assert(frame.byteCount() == 4);
            assert(frame.registersData()[0] == 0x00);
            assert(frame.registersData()[1] == 0x06);
            assert(frame.registersData()[2] == 0x00);
            assert(frame.registersData()[3] == 0x05);
            assert(frame.validateRTU() == ValidationStatus::OK);

            testData = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
            frame = Frame::fromRawRtuData(testData, true);
            assert(frame.RTULength() == 8);
            assert(frame.slaveID()==0x01);
            assert(frame.functionCode() == FunctionCode::ReadHoldingRegisters);
            assert(frame.startAddress() == 0);
            assert(frame.registerCount() == 2);
            assert(frame.byteCount() == 0);
            assert(frame.validateRTU() == ValidationStatus::OK);

            testData = {0x01, 0x03, 0x04, 0x00, 0x06, 0x00, 0x05, 0xda, 0x31};
            frame = Frame::fromRawRtuData(testData, false);
            assert(frame.RTULength() == 9);
            assert(frame.slaveID()==0x01);
            assert(frame.functionCode() == FunctionCode::ReadHoldingRegisters);
            assert(frame.byteCount() == 4);
            assert(frame.registersData()[0] == 0x00);
            assert(frame.registersData()[1] == 0x06);
            assert(frame.registersData()[2] == 0x00);
            assert(frame.registersData()[3] == 0x05);
            assert(frame.validateRTU() == ValidationStatus::OK);
        }

        uint16_t calculateRTULength() const {
            return calculateRTULength(isException(),_isRequest,functionCode(),byteCount());
        }
        uint16_t calculateRTULength(bool isException, bool isRequest, FunctionCode functionCode, uint16_t byteCount) const {
            if (isException) {
                return RTU_HEADER_SIZE + EXCEPTION_CODE_SIZE + CRC_SIZE;
            }
            switch (functionCode) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    if (_isRequest)
                        return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + REGISTER_COUNT_SIZE + CRC_SIZE;
                    else
                        return RTU_HEADER_SIZE + BYTE_COUNT_SIZE + byteCount + CRC_SIZE;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + WRITE_DATA_SIZE + CRC_SIZE;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    if (_isRequest)
                        return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + REGISTER_COUNT_SIZE + BYTE_COUNT_SIZE + byteCount
                               + CRC_SIZE;
                    else
                        return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + REGISTER_COUNT_SIZE + CRC_SIZE;
                default:
                    return 0;
            };
        }

        uint16_t calculateExpectedResponseRTULength() const {
            if (!_isRequest)
                return RTULength();
            return calculateRTULength(false,true,functionCode(),registerCount()*2);
        }
        int calculateResponseTransmissionTimeMs(const int bitsPerSecond){
            // constexpr int SLAVE_ID_LEN = 1;
            // constexpr int FUNCTION_LEN = 1;
            // constexpr int BYTE_COUNT_LEN = 1;
            // int DATA_LEN = request.registerCount();
            // constexpr int CRC_LEN = 2;
            // int length = SLAVE_ID_LEN + FUNCTION_LEN + BYTE_COUNT_LEN + DATA_LEN + CRC_LEN ;
            int length = calculateExpectedResponseRTULength();
            return calculateTransmissionTimeMs(length,bitsPerSecond);
        }
        int calculateTransmissionTimeMs(const size_t length, const int bitsPerSecond) {
            constexpr int BITS_PER_BYTE = 10;
            constexpr int INCREASE_PRECISION = 10;
            const int result = ((BITS_PER_BYTE * 1000 * static_cast<ssize_t>(length) * INCREASE_PRECISION / bitsPerSecond) +
                                5) / INCREASE_PRECISION;

            // return (result < 3)?3:result;
            return (result < 0)?0:result;
        }
        int calculateTransmissionTimeMs(const int bitsPerSecond) {
            return calculateTransmissionTimeMs(calculateRTULength(),bitsPerSecond);
        }
        //TODO assign registersValues split into request and response

        static Frame build(bool is_request, uint8_t slave_ID, FunctionCode function_code, uint16_t start_address,
                                 uint16_t register_count, std::span<uint16_t> registers_values = {},
                                 uint16_t transaction_ID = 0) {
            Frame frame;
            frame.rebuild(is_request, slave_ID, function_code, start_address, register_count, registers_values,
                          transaction_ID);
            return frame;
        }

        Frame &rebuild(bool is_request, uint8_t slave_ID, FunctionCode function_code, uint16_t start_address,
                             uint16_t register_count, std::span<uint16_t> registers_values = {},
                             uint16_t transaction_ID = 0) {
            isRequest(is_request);
            transactionID(transaction_ID);
            slaveID(slave_ID);
            functionCode(function_code);

            startAddress(start_address);
            registerCount(register_count);
            byteCount(register_count * 2); //merge registerCount and bytecount?
            registersValues(registers_values);

            MBAPLength(RTULengthWithoutCRC());
            appendCRC();
            return *this;
        }

        static Frame buildExceptionResponse(uint8_t slaveID, FunctionCode function_code, ExceptionCode exception_code,
                                                  uint16_t transaction_ID = 0) {
            Frame frame;
            frame.rebuildExceptionResponse(slaveID, function_code, exception_code, transaction_ID);
            return frame;
        }

        Frame &rebuildExceptionResponse(uint8_t slave_ID, FunctionCode function_code, ExceptionCode exception_code,
                                              uint16_t transaction_ID = 0) {
            transactionID(transaction_ID);
            slaveID(slave_ID);
            functionCode(function_code);
            isException(true);
            exceptionCode(exception_code);

            MBAPLength(RTULengthWithoutCRC());
            appendCRC();
            return *this;
        }

        std::string toString()  {
            return eModbus::toString(_dataBuffer());
        }

        static constexpr uint8_t MBAP_HEADER_SIZE = 7;
        static constexpr uint8_t RTU_HEADER_START_POSITION = MBAP_HEADER_SIZE - 1;
        static constexpr uint8_t TRANSACTION_ID_SIZE = 2;
        static constexpr uint8_t PROTOCOL_ID_SIZE = 2;
        static constexpr uint8_t TCP_LENGTH_SIZE = 2;
        static constexpr uint8_t UNIT_ID_SIZE = 1;
        static constexpr uint8_t RTU_HEADER_SIZE = 2;
        static constexpr uint8_t BYTE_COUNT_SIZE = 1;
        static constexpr uint8_t STARTING_ADDRESS_SIZE = 2;
        static constexpr uint8_t REGISTER_COUNT_SIZE = 2;
        static constexpr uint8_t WRITE_DATA_SIZE = 2;
        static constexpr uint8_t CRC_SIZE = 2;
        static constexpr uint8_t EXCEPTION_CODE_SIZE = 1;

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
