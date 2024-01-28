// PCA9546A I2C Multiplexer/Switch
// Datasheet: https://www.ti.com/lit/ds/symlink/pca9546a.pdf

#pragma once

#include "stm32g4xx_hal.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware_i2c.hpp"

#include <numbers>
#include "hardware.hpp"
#include "units/units.hpp"
#include <memory>

extern osSemaphoreId_t spectral_read_status;
extern osSemaphoreId_t spectral_write_status;

namespace mrover {

    class I2CMux {
    public:
    	I2CMux() = default;

    	I2CMux(std::shared_ptr<SMBus> i2c_bus);

        void set_channel(uint8_t channel);

    private:
        std::shared_ptr<SMBus> m_i2c_bus;
        uint8_t current_channel = 0;
        constexpr static std::uint16_t MUX_7b_ADDRESS = 0x70;
    };

} // namespace mrover

