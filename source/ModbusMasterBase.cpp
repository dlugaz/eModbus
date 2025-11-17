//
// Created by kdluzynski on 12.08.2025.
//
#include "ModbusMasterBase.hpp"

#include <map>
#include <chrono>

#include "ModbusRegisterBuffer.hpp"
using namespace std::chrono_literals;
eModbus::MasterBase::MasterBase(IStreamDevice &serial_device):_streamDevice(serial_device) {
}


eModbus::MasterBase eModbus::MasterBase::TCP(IStreamDevice &serial_device) {
    eModbus::MasterBase result(serial_device);
    result.isTCP = true;
    return result;
}

eModbus::MasterBase eModbus::MasterBase::RTU(IStreamDevice &serial_device) {
    eModbus::MasterBase result(serial_device);
    result.isTCP = false;
    return result;
}

std::vector<uint16_t> eModbus::MasterBase::read(const uint8_t slave_ID, const RegisterType register_type,
    const uint16_t start_address, const uint8_t quantity) {
    eModbus::Frame frame = eModbus::Frame::build(
        true,
        slave_ID,
        getFunctionCode(true,register_type),
        start_address,
        quantity);
    // ModbusFrame receiveFrame{};
    sendReceiveFrame(frame,frame);
    if (frame.isException())
        throw ModbusException(frame.exceptionCode());
    return frame.registersValues();
}

void eModbus::MasterBase::read(const uint8_t slave_ID, const eModbus::RegisterBufferView &outBuffer) {
    std::ranges::copy(read(
                            slave_ID,
                            outBuffer.registerType(),
                            outBuffer.startAddress(),
                            outBuffer.buffer().size()),
                    outBuffer.buffer().begin());
}


void eModbus::MasterBase::write(uint8_t slave_ID, RegisterType register_type, uint16_t start_address,
    std::span<uint16_t> values) {
    eModbus::Frame frame = eModbus::Frame::build(
        true,
        slave_ID,
        getFunctionCode(false,register_type),
        start_address,
        values.size(),values);
    // eModbus::Frame receiveFrame{};
    sendReceiveFrame(frame,frame);
    if (frame.isException())
        throw ModbusException(frame.exceptionCode());
}

void eModbus::MasterBase::sendFrame(eModbus::Frame &send_frame, const uint16_t timeout_ms) const {
    const SerialError err = _streamDevice.write(
        isTCP ? send_frame.tcpFrame() : send_frame.rtuFrame(), timeout_ms);
    if (err != SerialError::SUCCESS)
        throw StreamDeviceFailure(err);
}

void eModbus::MasterBase::receiveFrame(eModbus::Frame &receive_frame,const uint16_t timeout_ms) const {
    receive_frame.isRequest(false);
    const SerialError err = _streamDevice.read(isTCP ? receive_frame.buffer() : receive_frame.rtuBuffer(),
                                         timeout_ms);
    if (err != SerialError::SUCCESS) {
        throw StreamDeviceFailure(err);
    }

}

void eModbus::MasterBase::sendReceiveFrame(eModbus::Frame &send_frame, eModbus::Frame &receive_frame) {

    uint16_t slave_ID = send_frame.slaveID();
    uint32_t baud = 0;

    if (!devicesBaudratesMap.contains(slave_ID)) {
        baud = detectBaud(slave_ID, baudrates);
        if (baud == 0)
            throw StreamDeviceFailure(SerialError::TIMEOUT);;
    } else {
        baud = devicesBaudratesMap[slave_ID];
    }
    _streamDevice.baudrate(baud);
    sendFrame(send_frame, send_frame.calculateTransmissionTimeMs(baud) * 2);
    receiveFrame(receive_frame, getResponseTimeout(send_frame, devicesBaudratesMap[slave_ID]));

    eModbus::Frame::ValidationStatus validation = receive_frame.validateRTU();
    if (validation != eModbus::Frame::ValidationStatus::OK)
        throw InvalidFrame(validation);
}

uint32_t eModbus::MasterBase::getResponseTimeout(eModbus::Frame send_frame, const unsigned long baud) const {
    return send_frame.calculateResponseTransmissionTimeMs(baud) + deviceResponseTime_ms;
}

uint32_t eModbus::MasterBase::detectBaud(const uint8_t slave_ID, std::span<const uint32_t> baudrates) {

    eModbus::Frame send_frame = eModbus::Frame::build(true, slave_ID, eModbus::Frame::FunctionCode::ReadInputRegisters, 0, 1);
    eModbus::Frame receive_frame;
    uint32_t working_baud = 0;
    uint32_t originalBaud = _streamDevice.baudrate();
    if (originalBaud != IStreamDevice::InvalidBaudrate) {
        for (const auto baud : baudrates) {
            _streamDevice.baudrate(baud); // Set the new baud rate for testing

            SerialError err = _streamDevice.write(send_frame.rtuFrame(), send_frame.calculateTransmissionTimeMs(baud)*2);
            if (err != SerialError::SUCCESS) {
                break;
            }

            err = _streamDevice.read(receive_frame.rtuBuffer(), getResponseTimeout(send_frame, baud));
            if (err == SerialError::TIMEOUT) {
                continue;
            }
            if (err != SerialError::SUCCESS) {
                break;
            }

            if (receive_frame.validateRTU() == eModbus::Frame::ValidationStatus::OK) {
                // Success! We found the working baud rate.
                working_baud = baud; // Return the working baud and leave it set.
                break;
            }
        }
        _streamDevice.baudrate(originalBaud);
    }else {
        originalBaud = 9600;
        if (_streamDevice.write(send_frame.rtuFrame(), send_frame.calculateTransmissionTimeMs(originalBaud)*2) != SerialError::SUCCESS) {
            return IStreamDevice::InvalidBaudrate;
        }

        if (_streamDevice.read(receive_frame.rtuBuffer(), getResponseTimeout(send_frame, originalBaud)) != SerialError::SUCCESS) {
            return IStreamDevice::InvalidBaudrate;
        }

        if (receive_frame.validateRTU() == eModbus::Frame::ValidationStatus::OK) {
            working_baud = baudrates.empty() ? 1 : baudrates[0];
        }
    }
    if (working_baud) {
        devicesBaudratesMap.emplace(slave_ID,working_baud);
    }else {
        devicesBaudratesMap.erase(slave_ID);
    }

    return working_baud;
}


std::map<uint8_t, uint32_t> eModbus::MasterBase::scanForDevices(std::span<uint32_t> baudrates, uint16_t timeoutMs) {
    constexpr int MODBUS_MIN_ADDRESS = 1;
    constexpr int MODBUS_MAX_ADDRESS = 247;

    for (int slave_id = MODBUS_MIN_ADDRESS; slave_id <= MODBUS_MAX_ADDRESS; ++slave_id) {
        int baud = detectBaud(slave_id, baudrates);
        if (baud != 0) {
            devicesBaudratesMap[slave_id] = baud;
        }
    }

    return devicesBaudratesMap;
}

eModbus::Frame::FunctionCode eModbus::MasterBase::getFunctionCode(bool isRead, RegisterType register_type) {
    switch(register_type){
        case RegisterType::Coil:
            return isRead?
                       eModbus::Frame::FunctionCode::ReadCoils
                       :eModbus::Frame::FunctionCode::WriteMultipleCoils;
        case RegisterType::DiscreteInput:
            return isRead
                       ?eModbus::Frame::FunctionCode::ReadDiscreteInputs
                       :throw std::invalid_argument("Unable to write to Discrete Inputs");
        case RegisterType::AnalogInput:
            return isRead?
                       eModbus::Frame::FunctionCode::ReadInputRegisters
                       :throw std::invalid_argument("Unable to write to Input Registers");;
        case RegisterType::Holding:
            return isRead?
                       eModbus::Frame::FunctionCode::ReadHoldingRegisters
                       :eModbus::Frame::FunctionCode::WriteMultipleRegisters;
        default:
            throw std::invalid_argument("Unknown Register Type");
            //				return ModbusFrame::FunctionCode::Invalid;
    }

}
