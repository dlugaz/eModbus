#pragma once
#include <vector>

#include "ModbusUtils.hpp"


namespace eModbus {
	class TagValue {
		// Holds only the registers for THIS specific tag[cite: 1]
		std::vector<uint16_t> registers;

	public:
		explicit TagValue(std::span<const uint16_t> data)
			: registers(data.begin(), data.end()) {}

		TagValue() = default;
		TagValue(const TagValue&) = default;
		TagValue(TagValue&&) = default;
		TagValue& operator=(const TagValue& v) = default;
		TagValue& operator=(TagValue&&) = default;

		template<typename T>
		TagValue(const T& value) {
			set(value); // Reuse your existing set logic
		}
		// Automatic/Explicit conversion using your template system
		template<typename T, ByteOrder Order = ByteOrder::MSB>
		T as() const {
			if (registers.empty()) {
				throw std::runtime_error("TagValue contains no data");
			}
			// Uses the existing specialization mechanism[cite: 2, 3]
			return convertFromRegisters<T, Order>(registers);
		}
		template<typename T, ByteOrder Order = ByteOrder::MSB>
		void set(const T& value) {
			size_t numRegs = requiredRegisters<T>();

			if (registers.size() < numRegs) {
				registers.resize(numRegs);
			}

			convertToRegisters<T, Order>(registers, value);
		}

		template<typename T>
		TagValue &operator=(const T &value) {
			set(value);
			return *this;
		}
		template<typename T>
		operator T() const {
			return as<T>();
		}

		std::vector<uint16_t> data() const { return registers; }
		size_t size() const { return registers.size(); }

		auto operator<=>(const TagValue & tag_value) const = default;
	};
	template<>
	  constexpr TagValue convertFromRegisters<TagValue>(const std::span<const uint16_t> registers) {
		return TagValue(registers);
	}
}
