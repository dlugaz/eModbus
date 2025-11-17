//
// Created by kdluzynski on 07.10.2025.
//

#ifndef MODBUSTAGVARIANT_HPP
#define MODBUSTAGVARIANT_HPP
#include <cstdint>
#include <vector>

#include "ModbusRegisterBuffer.hpp"
#include "ModbusUtils.hpp"

namespace eModbus {
    class TagVariant {
        std::vector<uint16_t> data; //store variant in modbus registers
        // eModbus::RegisterBuffer data2;
    public:
        TagVariant() = default;
        TagVariant(const TagVariant&) = default;
        TagVariant& operator=(const TagVariant&) = default;
        TagVariant(TagVariant&&) = default;
        TagVariant& operator=(TagVariant&&) = default;
        ~TagVariant() = default;



    };
}
#endif //MODBUSTAGVARIANT_HPP