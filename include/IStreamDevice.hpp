/*
 * ISerialDevice.hpp
 *
 *  Created on: Jul 16, 2025
 *      Author: kdluzynski
 */

#ifndef INC_ISTREAMDEVICE_HPP_
#define INC_ISTREAMDEVICE_HPP_
#include <inttypes.h>
#include <span>
#include <functional>
#include <string>
#include <string_view>
/**
 * @brief Defines common error codes for serial operations.
 */
enum class SerialError : uint8_t {
    SUCCESS = 0,
    TIMEOUT,
    READY_TIMEOUT,
	INTERNAL_ERROR,
    BUSY,
    BUFFER_TOO_SMALL,
    INVALID_ARGUMENT,
    // Add more specific errors as needed
    UNKNOWN_ERROR
};

inline std::string to_string(SerialError error) {
    switch (error) {
        case SerialError::SUCCESS: return "SUCCESS";
        case SerialError::TIMEOUT: return "TIMEOUT";
        case SerialError::READY_TIMEOUT: return "READY_TIMEOUT";
        case SerialError::INTERNAL_ERROR: return "INTERNAL_ERROR";
        case SerialError::BUSY: return "BUSY";
        case SerialError::BUFFER_TOO_SMALL: return "BUFFER_TOO_SMALL";
        case SerialError::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        default:
        case SerialError::UNKNOWN_ERROR: return "UNKNOWN_ERROR";
    }
}


class IStreamDevice {
protected:
    std::function<void(std::span<uint8_t> received_data)> rxCompleteCallback;
    std::function<void()> txCompleteCallback;

public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~IStreamDevice() = default;

    /**
     * @brief Reads data from the serial device into a buffer.
     * This function should block until 'length' bytes are read or a timeout occurs.
     *
     * @param buffer Pointer to the buffer to store the read data.
     * @param length The maximum number of bytes to read.
     * @param bytes_read_out Pointer to a variable to store the actual number of bytes read.
     * @param timeout_ms The maximum time to wait for data in milliseconds.
     * @return SerialError indicating success or the type of error.
     */
    virtual SerialError read(std::span<uint8_t> buffer, uint32_t timeout_ms, size_t* bytes_read_out = nullptr) = 0;

    /**
     * @brief Writes data from a buffer to the serial device.
     * This function should block until 'length' bytes are written or a timeout occurs.
     *
     * @param buffer Pointer to the data to write.
     * @param length The number of bytes to write.
     * @param bytes_written_out Pointer to a variable to store the actual number of bytes written.
     * @param timeout_ms The maximum time to wait for write completion in milliseconds.
     * @return SerialError indicating success or the type of error.
     */
    virtual SerialError write(std::span<const uint8_t> buffer, uint32_t timeout_ms, size_t* bytes_written_out = nullptr) = 0;
    static constexpr uint32_t InvalidBaudrate = 0;
    virtual void baudrate(uint32_t baudrate) {
    }

    virtual uint32_t baudrate() const {
        return InvalidBaudrate;
    }
    /**
     * @brief Flushes any pending outgoing data from the serial buffer.
     * @return SerialError indicating success or the type of error.
     */
    virtual SerialError flush() = 0;
    // Example member functions to be called from the HAL callbacks
    virtual void setOnTxCompleteCallback(std::function<void()> callback){
    	txCompleteCallback = std::move(callback);
    }

    virtual void setOnRxCompleteCallback(std::function<void(std::span<uint8_t> received_data)> callback){
    	rxCompleteCallback = std::move(callback);
    }
private:
    virtual void onTxComplete() = 0;

    virtual void onRxComplete(uint16_t size) = 0;
};


#endif /* INC_ISTREAMDEVICE_HPP_ */
