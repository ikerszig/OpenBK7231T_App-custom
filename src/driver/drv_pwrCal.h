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
// Same as PwrCal_Scale, but picks the low-load Current correction curve
// for the given device index (0/1) - twin-mode drivers (two physical
// sensor chips, e.g. BL0942 CH1+CH2) use this instead, since the two
// chips can have different low-load nonlinearity. Single-device drivers
// keep calling plain PwrCal_Scale (always device 0).
void PwrCal_ScaleForDevice(int device, int raw_voltage, float raw_current, int raw_power,
                  float *real_voltage, float *real_current, float *real_power);
float PwrCal_ScalePowerOnly(int raw_power);
// Periodic retained re-publish of the 4 calibration constants, so they
// reach MQTT/HA even if the boot-time publish happened before the MQTT
// connection was up (it is silently dropped in that case - no queueing).
// Call this once per second; it throttles itself internally.
void PwrCal_PublishPeriodic(void);
