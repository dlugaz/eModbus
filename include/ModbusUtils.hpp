#pragma once
#include <cstdint>
#include <cmath>
#include <span>      // For std::span (C++20)
#include <bit>       // For std::bit_cast (C++20)
#include <cstring>   // For std::memcpy (pre-C++20 fallback)
#include <stdexcept> // For exceptions like std::out_of_range
#include <algorithm> // For std::fill

namespace eModbus {
    constexpr uint16_t MAX_MODBUS_REGISTERS = 125;
    enum class RegisterType{
        Coil,
        DiscreteInput,
        AnalogInput,
        Holding,
    };
    // --- Configuration and Utilities ---

    // Define ByteOrder to differentiate between MSB and LSB extraction/insertion
    enum class ByteOrder {
        LSB,
        MSB // Default order is often MSB-first in protocol logic
    };


    // Safe Bit Casting: Uses std::bit_cast (C++20) or std::memcpy fallback (pre-C++20)
#if __cplusplus < 202002L
    template<typename Target, typename Source>
    Target bit_cast_fallback(const Source& source) noexcept {
        static_assert(sizeof(Target) == sizeof(Source));
        Target target;
        std::memcpy(&target, &source, sizeof(Target));
        return target;
    }
#define MODBUS_BIT_CAST bit_cast_fallback
#else
#define MODBUS_BIT_CAST std::bit_cast
#endif

    // Utilities to access single bytes of a 16-bit register
    constexpr uint8_t getU8MSB(const uint16_t registerValue) { return static_cast<uint8_t>(registerValue >> 8); }
    constexpr uint8_t getU8LSB(const uint16_t registerValue) { return static_cast<uint8_t>(registerValue & 0xFF); }

    constexpr void setU8LSB(uint16_t& currentRegisterValue, const uint16_t u8Value) {
        currentRegisterValue = static_cast<uint16_t>((currentRegisterValue & ~(0xFF00)) | u8Value);
    }
    constexpr void setU8MSB(uint16_t& currentRegisterValue, const uint16_t u8Value) {
        currentRegisterValue = static_cast<uint16_t>((currentRegisterValue & ~(0x00FF)) | u8Value<<8);
    }

    // Utility to combine two registers into a 32-bit integer (Standard Modbus Order: MSW, LSW).
    constexpr uint32_t registersToU32(const std::span<const uint16_t> registers) {
        if (registers.size() < 2) throw std::out_of_range("Span too small for 32-bit read (need 2 registers).");
        // [0] is MSW, [1] is LSW
        return (static_cast<uint32_t>(registers[0]) << 16) | registers[1];
    }

    // Utility to split a 32-bit integer into two registers (Standard Modbus Order: MSW, LSW).
    constexpr void u32ToRegisters(uint32_t source, std::span<uint16_t> registers) {
        if (registers.size() < 2) throw std::out_of_range("Span too small for 32-bit write (need 2 registers).");
        // [0] is MSW, [1] is LSW
        registers[0] = static_cast<uint16_t>(source >> 16);     // MSW
        registers[1] = static_cast<uint16_t>(source & 0xFFFF);  // LSW
    }
    template<size_t N>
    constexpr std::array<uint8_t, N> registersToBytes(const std::span<const uint16_t> registers) {
        std::array<uint8_t, N> result = {}; // Initialize to zero
        size_t byte_index = 0;
        for (size_t i = 0; i < N/2; ++i) {
            const uint16_t reg = registers[i];
            result[byte_index++] = getU8MSB(reg);
            result[byte_index++] = getU8LSB(reg);
        }
        return result;
    }

    // ------------------------------------------------------------------------
    // PRIMARY TEMPLATES (The Public Interface)
    // ------------------------------------------------------------------------

    // Read: Returns value by copy. Uses ByteOrder::MSB as default for non-8-bit types.
    template<typename T, ByteOrder Order = ByteOrder::MSB>
    constexpr T convertFromRegisters(const std::span<const uint16_t> registers) {
        return static_cast<T>(registers[0]);
    }

    // Write: Returns void (performs side effect). Uses ByteOrder::MSB as default.
    template<typename T, ByteOrder Order = ByteOrder::MSB>
    constexpr void convertToRegisters(std::span<uint16_t> registers, const T& source) {
        registers[0] = static_cast<uint16_t>(source);
    }


    // ------------------------------------------------------------------------
    // ** Specialized `convertFromModbusRegisters` (Read - Return by Value) **
    // ------------------------------------------------------------------------

    // Type: float (2 registers)
    template<>
    constexpr float convertFromRegisters<float>(const std::span<const uint16_t> registers) {
        if (registers.size() < 4 ) throw std::out_of_range("Registers too small");
        uint32_t combined_u32 = registersToU32(registers);
        return MODBUS_BIT_CAST<float>(combined_u32);
    }

    // Type: uint32_t (2 registers)
    template<>
    constexpr uint32_t convertFromRegisters<uint32_t>(const std::span<const uint16_t> registers) {
        if (registers.size() < 4 ) throw std::out_of_range("Registers too small");
        return registersToU32(registers);
    }

    // Type: uint16_t (1 register)
    template<>
    constexpr uint16_t convertFromRegisters<uint16_t>(const std::span<const uint16_t> registers) {
        if (registers.size() < 1 ) throw std::out_of_range("Registers too small");
        return registers[0];
    }

    // Type: uint8_t (MSB) (1 register, specific byte order)
    template<>
    constexpr uint8_t convertFromRegisters<uint8_t, ByteOrder::MSB>(
        const std::span<const uint16_t> registers) {
        if (registers.size() < 1 ) throw std::out_of_range("Registers too small");
        return getU8MSB(registers[0]);
    }

    // Type: uint8_t (LSB) (1 register, specific byte order)
    template<>
    constexpr uint8_t convertFromRegisters<uint8_t, ByteOrder::LSB>(
        const std::span<const uint16_t> registers) {
        if (registers.size() < 1 ) throw std::out_of_range("Registers too small");
        return getU8LSB(registers[0]);
    }

    // Type: std::string (N registers)
    template<>
    constexpr std::string convertFromRegisters<std::string>(const std::span<const uint16_t> registers) {
        std::string result;
        result.reserve(registers.size() * 2);

        for (uint16_t reg : registers) {
            char msb = static_cast<char>(getU8MSB(reg));
            char lsb = static_cast<char>(getU8LSB(reg));

            // Stop on null terminator, otherwise append.
            if (msb == 0) break;
            result += msb;

            if (lsb == 0) break;
            result += lsb;
        }
        return result;
    }
    template<>
    constexpr std::vector<uint8_t> convertFromRegisters<std::vector<uint8_t>>(const std::span<const uint16_t> registers) {
        std::vector<uint8_t> result;
        result.reserve(registers.size() * 2);

        for (uint16_t reg : registers) {
            char msb = static_cast<char>(getU8MSB(reg));
            char lsb = static_cast<char>(getU8LSB(reg));

            result.push_back(msb);
            result.push_back(lsb);
        }
        return result;
    }



    // ------------------------------------------------------------------------
    // ** Specialized `convertToModbusRegisters` (Write - Void) **
    // ------------------------------------------------------------------------

    // Type: uint16_t (1 register)
    template<>
    constexpr void convertToRegisters<uint16_t>(std::span<uint16_t> registers, const uint16_t& source) {
        if (registers.size() < 1 ) throw std::out_of_range("Registers too small");
        registers[0] = source;
    }


    // Type: float (2 registers)
    template<>
    constexpr void convertToRegisters<float>(std::span<uint16_t> registers, const float& source) {
        if (registers.size() < 4 ) throw std::out_of_range("Registers too small");
        uint32_t combined = MODBUS_BIT_CAST<uint32_t>(source);
        u32ToRegisters(combined, registers);
    }

    // Type: uint32_t (2 registers)
    template<>
    constexpr void convertToRegisters<uint32_t>(std::span<uint16_t> registers, const uint32_t& source) {
        if (registers.size() < 4 ) throw std::out_of_range("Registers too small");
        u32ToRegisters(source, registers);
    }

    // Type: uint8_t (MSB) (1 register, preserves LSB)
    template<>
    constexpr void convertToRegisters<uint8_t, ByteOrder::MSB>(std::span<uint16_t> registers, const uint8_t& source) {
        if (registers.size() < 1 ) throw std::out_of_range("Registers too small");
        // Keep LSB (0x00FF), overwrite MSB
        registers[0] = (registers[0] & 0x00FF) | (static_cast<uint16_t>(source) << 8);
    }

    // Type: uint8_t (LSB) (1 register, preserves MSB)
    template<>
    constexpr void convertToRegisters<uint8_t, ByteOrder::LSB>(std::span<uint16_t> registers, const uint8_t& source) {
        if (registers.size() < 1 ) throw std::out_of_range("Registers too small");
        // Keep MSB (0xFF00), overwrite LSB
        registers[0] = (registers[0] & 0xFF00) | source;
    }

    // Type: std::string (N registers)
    template<>
    constexpr void convertToRegisters<std::string>(std::span<uint16_t> registers, const std::string& source) {
        size_t char_index = 0;
        size_t max_chars = registers.size() * 2;
        if (max_chars < source.length() ) throw std::out_of_range("Registers too small");
        // Fill the destination with zero to ensure null termination/padding
        std::fill(registers.begin(), registers.end(), 0x0000);

        for (uint16_t& reg : registers) {
            // Write MSB
            if (char_index < source.length() && char_index < max_chars) {
                reg |= (static_cast<uint16_t>(source[char_index]) << 8);
                char_index++;
            }
            // Write LSB
            if (char_index < source.length() && char_index < max_chars) {
                reg |= static_cast<uint16_t>(source[char_index]);
                char_index++;
            }

            // If the string is fully written, break (remaining registers are already 0x0000)
            if (char_index >= source.length()) {
                break;
            }
        }
    }

    template<>
    inline void convertToRegisters<std::vector<uint8_t>>(std::span<uint16_t> registers, const std::vector<uint8_t>& source) {

        const size_t max_regs = registers.size();
        const size_t max_bytes_to_write = max_regs * 2;
        const size_t source_byte_count = source.size();
        const size_t bytes_to_process = std::min(source_byte_count, max_bytes_to_write);
        if (max_bytes_to_write < source.size() ) throw std::out_of_range("Registers too small");
        size_t byte_idx = 0;
        for (size_t reg_idx = 0; byte_idx < bytes_to_process; reg_idx++) {
            setU8MSB(registers[reg_idx],source[byte_idx]);
            byte_idx++;
            if (byte_idx < bytes_to_process) {
                setU8LSB(registers[reg_idx],source[byte_idx]);
                byte_idx++;
            }
        }
    }

    // uint16_t reg_value = static_cast<uint16_t>(source[byte_idx++]) << 8;
    // if (byte_idx < bytes_to_process) {
    //     reg_value |= source[byte_idx++];
    // } else {
    // }
    // // 3. Write the resulting 16-bit register to the destination
    // registers[reg_idx++] = reg_value;


}