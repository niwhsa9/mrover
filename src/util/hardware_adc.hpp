#pragma once

#include "main.h"
#include "messaging.hpp"

#include <bit>
#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

namespace mrover {


    class ADCSensor {
	public:
		ADCSensor() = default;

		ADCSensor(ADC_HandleTypeDef *hadc, uint8_t channels)
			: m_hadc(hadc), m_channels(channels) {
			m_values.resize(channels);
		}

		uint32_t get_raw_channel_value(uint8_t channel) {
			return m_values.at(channel);
		}

		void update() {

			HAL_ADC_Start_DMA(m_hadc, static_cast<uint32_t*>(static_cast<void*>(m_values.data())), m_channels * (sizeof(uint32_t) / sizeof(uint16_t)));
		}

	private:
		ADC_HandleTypeDef* m_hadc;
		uint8_t m_channels;
		std::vector<uint16_t> m_values;
	};

} // namespace mrover
