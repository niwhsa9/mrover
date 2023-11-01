////////////////////////////////
//      TMCS1108A4BQDR Current Sensor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////

#include "diag_curr_sensor.h"


// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created current sensor object
DiagCurrentSensor* new_diag_current_sensor(ADCSensor* adc_sensor, uint8_t channel) {
    DiagCurrentSensor* current_sensor = (DiagCurrentSensor*) malloc(sizeof(DiagCurrentSensor));
    current_sensor->adc_sensor = adc_sensor;
    current_sensor->channel = channel;
    current_sensor->amps = 0;

    return current_sensor;
}

// REQUIRES: valid current sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_diag_current_sensor_val(DiagCurrentSensor* sensor) {
    // sensor returns volts (I think) so get to millivolts and solve the proportion for amps then add the offset. (vcc/2)
	uint16_t adc_val = get_adc_sensor_value(sensor->adc_sensor, sensor->channel);
	float measured_volts = (adc_val * 3.3f) / 4096.0f;

	// V_out = I_in * S + V_s / 2
	// I_in = (V_s / 2 - V_out) / -S
	sensor->amps = (DIAG_CURR_V_s / 2.0f - measured_volts) / -DIAG_CURR_S;
}

// REQUIRES: valid current sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_diag_current_sensor_val(DiagCurrentSensor* sensor) {
    return sensor->amps;
}
