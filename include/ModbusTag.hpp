//
// Created by kdluzynski on 07.10.2025.
//

#ifndef MODBUSTAG_H
#define MODBUSTAG_H
#include <string_view>

#include "ModbusMasterBase.hpp"
namespace eModbus {
    enum class RegisterType;

    struct Tag {
        enum class modbus_parameter_type:char {
            U8 = 0x00,
            U16 = 0x01,
            U32 = 0x02,
            FLOAT = 0x03,
            ASCII = 0x04,
            U8_LSB = 0x07,
            U8_MSB = 0x08,
            BOOL = 0x09,
            BYTE_ARRAY = 0x0A,
        };

        enum class parameter_representation: char {
            NUMERICAL,
            BOOLEAN,
            BITSET,
            STRING,
            STRING_PASSWORD,
            TIME,
            DROPDOWN,
            SLIDER,
            LINK,
            NUMERICAL_HEX,
        };

        enum class user_level :char {
            NOONE = 0,
            DEFAULT = 0,
            OPERATOR = 1,
            OPERATOR1 = 1,
            OPERATOR2 = 2,
            OPERATOR3 = 3,
            OPERATOR4 = 4,
            OPERATOR5 = 5,
            OPERATOR6 = 6,
            OPERATOR7 = 7,
            OPERATOR8 = 8,
            OPERATOR9 = 9,
            SETUP = 10,
            ADMIN = 11,
            RESERVED = 12,
            SERVICE = 13,
            MSERVICE = 14,
            GOD = 15,
        };

        std::string name;
        std::string info;
        std::string unit;
        eModbus::RegisterType register_type;
        uint16_t register_number;
        uint16_t register_length;
        Tag::modbus_parameter_type register_value_type;
        Tag::parameter_representation representation_type;
        user_level access_level;
        float max_value;
        float min_value;
        std::string options;
        uint8_t precision;
        bool is_editable;
        std::string default_value;
        std::string category;
        std::string key;

    };
}
#endif //MODBUSTAG_H