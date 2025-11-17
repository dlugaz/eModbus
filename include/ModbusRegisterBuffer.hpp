//
// Created by kdluzynski on 07.10.2025.
//

#ifndef MODBUSREGISTERBUFFER_HPP
#define MODBUSREGISTERBUFFER_HPP
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

#include "ModbusTag.hpp"
#include "ModbusUtils.hpp"

namespace eModbus {
    class RegisterBufferView {
    public:
        using RegistersValueType = uint16_t;
        constexpr explicit RegisterBufferView(const uint16_t& startAddress,RegisterType registerType,const std::span<RegistersValueType> container):
        startAddress_ {startAddress},registerType_{registerType}, buffer_{container}{}

        template<typename T, eModbus::ByteOrder Order = eModbus::ByteOrder::MSB>
        constexpr void put(uint16_t modbus_address, const T& value) const{
            // The conversion function must be constexpr
            convertToRegisters<T, Order>(
                get_buffer_for_address(modbus_address),
                value
            );
        }
        template<typename T, eModbus::ByteOrder Order = eModbus::ByteOrder::MSB>
        constexpr void put(const Tag& tag, const T& value) const{
            // The conversion function must be constexpr
            convertToRegisters<T, Order>(
                get_buffer_for_address(tag.register_number),
                value
            );
        }

        template<typename T, eModbus::ByteOrder Order = eModbus::ByteOrder::MSB>
        constexpr T get(uint16_t modbus_address) const {
            // The conversion function must be constexpr
            return convertFromRegisters<T, Order>(
                get_buffer_for_address(modbus_address)
            );
        }

        template<typename T, eModbus::ByteOrder Order = eModbus::ByteOrder::MSB>
        constexpr T get(const Tag& tag) const {
            // The conversion function must be constexpr
            return convertFromRegisters<T, Order>(
                get_buffer_for_address(tag.register_number)
            );
        }

        template<typename T>
        constexpr void get_into(uint16_t modbus_address, T &destination) const
        {
            destination = get<T>(modbus_address);
        }

        constexpr uint16_t startAddress() const {
            return startAddress_;
        }
        constexpr std::span<RegistersValueType> buffer() const {
            return buffer_;
        }
        constexpr RegisterType registerType() const {
            return registerType_;
        }
        constexpr std::span<RegistersValueType> get_buffer_for_address(uint16_t modbus_address) const {
            const uint16_t offset = calculate_offset(modbus_address);
            return buffer_.subspan(offset);
        }
    private:
        const uint16_t startAddress_ = 0;
        const std::span<RegistersValueType> buffer_;
        const RegisterType registerType_;


        constexpr uint16_t calculate_offset(uint16_t modbus_address) const {
            if (modbus_address < startAddress_) {
                throw std::out_of_range("Modbus address is below buffer start address.");
            }
            uint16_t offset = modbus_address - startAddress_;


            if (offset > buffer_.size() || offset > MAX_MODBUS_REGISTERS) {
                throw std::out_of_range("Modbus address exceeds buffer size");
            }
            return offset;
        }
    };

    class RegisterBuffer {
    public:
        using RegistersValueType = uint16_t;

        // Constructor now properly sizes the vector before creating a view
        RegisterBuffer(uint16_t startAddress, RegisterType registerType, uint16_t numRegisters)
            : startAddress_{startAddress},
                registerType_(registerType),
              registersValue_(numRegisters) // 1. The vector is created and sized.
        {
            // Now registersValue_ is valid and will not reallocate unexpectedly.
        }

        // Provide a method to get a non-owning view
        RegisterBufferView view() {
            return RegisterBufferView(startAddress_,registerType_, registersValue_);
        }

        // For convenience, you can forward the most common methods.
        // This still avoids code duplication by calling the view's implementation.
        template<typename T, eModbus::ByteOrder Order = eModbus::ByteOrder::MSB>
        constexpr void put(uint16_t modbus_address, const T& value) {
            view().put<T, Order>(modbus_address, value);
        }

        template<typename T, eModbus::ByteOrder Order = eModbus::ByteOrder::MSB>
        constexpr T get(uint16_t modbus_address){
            return view().get<T, Order>(modbus_address);
        }

        uint16_t startAddress_;
        RegisterType registerType_;
        std::vector<RegistersValueType> registersValue_; // 2. The data is owned by this class.
    };
}

#endif //MODBUSREGISTERBUFFER_HPP