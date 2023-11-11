#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "heater.hpp"

namespace mrover {

    constexpr static float MAX_HEATER_TEMP = 65.0f;

    Heater::Heater(DiagTempSensor const& diag_temp_sensor, Pin const& heater_pin)
    	: m_diag_temp_sensor(std::move(diag_temp_sensor)),
		  m_heater_pin(std::move(heater_pin)),
		  m_state(false),
		  m_auto_shutoff_enabled(true),
		  m_last_time_received_message(0)
	   {}

    float Heater::get_temp() {
    	return diag_temp_sensor.get_temp();
    }

    bool Heater::get_state() {
    	return m_state;
    }

    void Heater::enable_if_possible(bool enable) {
    	if (enable && m_auto_shutoff_enabled && get_temp() >= MAX_HEATER_TEMP) {
    		// The only time you don't service the request is if auto_shutoff is enabled
    		// and they want to turn it on when past the heater temp
			m_state = false;
    	}
    	else {
    		m_state = enable;
    	}

    	m_heater_pin.write(m_state);

    	// TODO - need to update m_last_time_received_message and feed watchdog
    }

    void Heater::update_temp_and_auto_shutoff_if_applicable() {
    	m_diag_temp_sensor.update_temp();
    	if (m_state && m_auto_shutoff_enabled && get_temp() >= MAX_HEATER_TEMP) {
			m_state = false;
			m_heater_pin.write(false);
		}
    }

    void Heater::turn_off_if_watchdog_not_fed() {
    	if (m_state) {
			bool watchdog_is_fed_recently = true; // TOOD - actually need to implement logic
			if (!watchdog_is_fed_recently) {
				m_state = false;
				m_heater_pin.write(false);
			}
    	}
    }

    void Heater::feed_watchdog() {
    	// TODO - implement the code
    }

} // namespace mrover

