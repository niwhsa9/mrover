// AS7262 Spectral sensor
// Datasheet: https://www.mouser.com/datasheet/2/588/AS7262_DS000486_2-00-1082195.pdf

#pragma once
//#include "FreeRTOS.h"
#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "hardware.hpp"
#include "i2c_mux.hpp"
#include <memory>

//#include "cmsis_os2.h"

// How do I get this file to know about these?
extern osSemaphoreId_t spectral_read_status;
extern osSemaphoreId_t spectral_write_status;

namespace mrover {

    class Spectral {
    public:
    	Spectral() = default;

    	Spectral(std::shared_ptr<SMBus> i2c_bus, std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel);

        void poll_status_reg();

    	void update_channel_data(); // updates all of the channels

        uint16_t get_channel_data(uint8_t channel);

        void reboot();

        bool is_error();

        void init();

        constexpr static std::uint16_t SPECTRAL_7b_ADDRESS = 0x49;
        constexpr static std::uint8_t I2C_AS72XX_SLAVE_STATUS_REG = 0x00;
        constexpr static std::uint8_t I2C_AS72XX_SLAVE_TX_VALID = 0x02;
        constexpr static std::uint8_t I2C_AS72XX_SLAVE_RX_VALID = 0x01;
    private:
        bool m_error{};
        bool m_initialized{};

        std::shared_ptr<SMBus> m_i2c_bus;
        std::shared_ptr<I2CMux> m_i2c_mux;
        uint8_t m_i2c_mux_channel;
        constexpr static std::uint8_t CHANNEL_DATA_LENGTH = 6;
        std::array<uint16_t, CHANNEL_DATA_LENGTH> channel_data {};
        // Sensor Raw Data Registers Start, 6 channels, 2 bytes each.
        // See pg. 22 of datasheet for more info.
        constexpr static std::uint8_t CHANNEL_V_HIGH = 0x08;
        constexpr static std::uint8_t CONTROL_SETUP_REG = 0x04;
        constexpr static std::uint8_t INT_TIME_REG = 0x05;
    };

} // namespace mrover

