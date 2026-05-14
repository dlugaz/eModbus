#pragma once

#include "config.hpp"
#include <string>
#include <ModbusUtils.hpp>

#include "ModbusTagValue.hpp"

namespace eModbus {
	struct TagCore {
		RegisterType register_type;
		uint16_t register_number;
		uint16_t register_length;
		TagID key;
	};
	using StringType = std::string;
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

		StringType name;
		StringType info;
		StringType unit;
		RegisterType register_type;
		uint16_t register_number;
		uint16_t register_length;
		modbus_parameter_type register_value_type;
		parameter_representation representation_type;
		user_level access_level;
		float max_value;
		float min_value;
		StringType options;
		uint8_t precision;
		bool is_editable;
		TagID parent;
		TagID key;
	};
}
