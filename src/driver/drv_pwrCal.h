#pragma once

// For backwards compatibility
typedef enum {
    PWR_CAL_MULTIPLY,
    PWR_CAL_DIVIDE
} pwr_cal_type_t;

void PwrCal_Init(pwr_cal_type_t type, float default_voltage_cal,
                 float default_current_cal, float default_power_cal);
void PwrCal_Scale(int raw_voltage, float raw_current, int raw_power,
                  float *real_voltage, float *real_current, float *real_power);
float PwrCal_ScalePowerOnly(int raw_power);
// Periodic retained re-publish of the 4 calibration constants, so they
// reach MQTT/HA even if the boot-time publish happened before the MQTT
// connection was up (it is silently dropped in that case - no queueing).
// Call this once per second; it throttles itself internally.
void PwrCal_PublishPeriodic(void);
