#include "drv_pwrCal.h"

#include <math.h>
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../mqtt/new_mqtt.h"

static pwr_cal_type_t cal_type;

static float voltage_cal = 1;
static float current_cal = 1;
static float power_cal = 1;
// Additive raw-domain offset for current, to correct the non-zero
// noise-floor/leakage reading seen at low loads (the BL0942 Calibrate()
// mechanism is multiplicative-only and cannot fix this). Reuses the
// unused CFG_OBK_RES_KIA slot (HLW8112-specific, not used by this
// BL0942-only device) for persistence - our board never runs the
// HLW8112 driver so this slot is safely free to repurpose.
#define CFG_OBK_CURRENT_OFFSET CFG_OBK_RES_KIA
static float current_offset_raw = 0;

static int latest_raw_voltage;
static float latest_raw_current;
static int latest_raw_power;

// Piecewise-linear kis-terheleses Current korrekcios gorbe (2026-10-21).
// A BL0942 kulon RMS-arammero utja kis (de nem nulla) terhelesnel torzit,
// mig a Power-ut pontosnak tunik a teljes tartomanyban (l.
// CB2S_OpenBeken_projekt_allapot.md 3. fejezet) - ezert ez a korrekcio
// CSAK a kijelzett Current-et (es a belole szamolt latszolagos/meddo
// teljesitmenyt) befolyasolja, a Power-t es az Energy Total-t (kWh,
// szamlazas) NEM. A gorbe pontjait az aram_nemlinearitas_elemez.py
// statisztikaja alapjan allitjuk - tisztan RAM-beli allapot, nincs ra
// kulon flash-slot, a persistalast a /startup_command mezobe irt
// CurrentCurvePoint hivas-sorozat adja (ugyanaz a minta, mint a BL0942opts
// inditasi parancs). Alapertelmezetten ures (nincs korrekcio, szorzo=1).
#define MAX_CURVE_POINTS 12
static float curve_current[MAX_CURVE_POINTS];
static float curve_mult[MAX_CURVE_POINTS];
static int curve_n = 0;

static float InterpolateCurrentMultiplier(float real_current) {
    if (curve_n == 0) return 1.0f;
    if (real_current <= curve_current[0]) return curve_mult[0];
    if (real_current >= curve_current[curve_n - 1]) return curve_mult[curve_n - 1];
    for (int i = 0; i < curve_n - 1; i++) {
        if (real_current >= curve_current[i] && real_current <= curve_current[i + 1]) {
            float span = curve_current[i + 1] - curve_current[i];
            if (span <= 0.0f) return curve_mult[i];
            float frac = (real_current - curve_current[i]) / span;
            return curve_mult[i] + (curve_mult[i + 1] - curve_mult[i]) * frac;
        }
    }
    return 1.0f;
}

//#define PWRCAL_DEBUG

// 2026-10-20: publish the 4 calibration constants to MQTT (retained) so
// HA keeps a history of WHEN they change - without this, a future
// recalibration is only remembered if a human manually logs it
// (kalibracio_rogzit.py), which is exactly the kind of thing that gets
// forgotten. Retained so a fresh HA subscription also sees the current
// value immediately, not just future changes.
static void PublishCalTopic(int cfg_index, float value) {
    const char *topic;
    if (cfg_index == CFG_OBK_VOLTAGE) topic = "calibration_voltage";
    else if (cfg_index == CFG_OBK_CURRENT) topic = "calibration_current";
    else if (cfg_index == CFG_OBK_POWER) topic = "calibration_power";
    else if (cfg_index == CFG_OBK_CURRENT_OFFSET) topic = "calibration_current_offset";
    else return;
    MQTT_PublishMain_StringFloat(topic, value, 6,
        OBK_PUBLISH_FLAG_RETAIN | OBK_PUBLISH_FLAG_QOS_ZERO);
}

static commandResult_t Calibrate(const char *cmd, const char *args, float raw,
                                 float *cal, int cfg_index) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    float real = Tokenizer_GetArgFloat(0);
	if (real == 0.0f) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "%s",
                     CMD_GetResultString(CMD_RES_BAD_ARGUMENT));
        return CMD_RES_BAD_ARGUMENT;
    }
#define VERY_SMALL_VAL 0.001f
	if (raw > -VERY_SMALL_VAL && raw < VERY_SMALL_VAL) {
		ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "Calibration incorrect - connect load first.");
		return CMD_RES_ERROR;
	}


    *cal = (cal_type == PWR_CAL_MULTIPLY ? real / raw : raw / real);
    if (isnan(*cal) || ((*cal) > -VERY_SMALL_VAL && (*cal) < VERY_SMALL_VAL))
    {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "Calibration incorrect - value is zero");
        return CMD_RES_ERROR;
    }
    CFG_SetPowerMeasurementCalibrationFloat(cfg_index, *cal);
    PublishCalTopic(cfg_index, *cal);

#ifdef PWRCAL_DEBUG
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "%s: you gave %f, set ref to %f",
                cmd, real, *cal);
#endif
    return CMD_RES_OK;
}

static commandResult_t CalibrateVoltage(const void *context, const char *cmd,
                                        const char *args, int cmdFlags) {
    return Calibrate(cmd, args, latest_raw_voltage, &voltage_cal,
                     CFG_OBK_VOLTAGE);
}

static commandResult_t CalibrateCurrent(const void *context, const char *cmd,
                                        const char *args, int cmdFlags) {
    return Calibrate(cmd, args, latest_raw_current, &current_cal,
                     CFG_OBK_CURRENT);
}

static commandResult_t CalibrateCurrentZero(const void *context, const char *cmd,
                                        const char *args, int cmdFlags) {
    current_offset_raw = latest_raw_current;
    CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT_OFFSET, current_offset_raw);
    PublishCalTopic(CFG_OBK_CURRENT_OFFSET, current_offset_raw);
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "CurrentZero: offset set to %f (raw)", current_offset_raw);
    return CMD_RES_OK;
}

static commandResult_t CalibratePower(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    return Calibrate(cmd, args, latest_raw_power, &power_cal, CFG_OBK_POWER);
}

// PowerSet/CurrentSet/VoltageSet mind az aktuális nyers ertekhez (egy
// PILLANATNYI, akar masodperceken belul ingadozo terheleshez) viszonyitanak -
// ez a lakas-osszfogyasztasnal (CH2) megbizhatatlan pivot egy kis (<1%)
// finomitashoz, mert (1) a terheles sajat maga is percek/masodpercek alatt
// jelentosen valtozhat, es (2) a "latest_raw_power" megosztott valtozo,
// amit a ket csatorna VALTVA ir felul - igy sosem biztos, melyik csatorna
// nyers erteket kapnank el. Ez a parancs ehelyett a MAR BEALLITOTT
// power_cal konstanst szorozza egy pontosan megadott arannyal - fuggetlen
// az aktualis terhelestol es attol, melyik csatorna irta utoljara a
// latest_raw_power-t.
static commandResult_t CalibratePowerScale(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    float factor = Tokenizer_GetArgFloat(0);
    if (factor <= 0.0f || isnan(factor)) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "%s",
                     CMD_GetResultString(CMD_RES_BAD_ARGUMENT));
        return CMD_RES_BAD_ARGUMENT;
    }
    power_cal *= factor;
    CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_POWER, power_cal);
    PublishCalTopic(CFG_OBK_POWER, power_cal);
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "PowerCalScale: factor %f, new power_cal = %f",
                factor, power_cal);
    return CMD_RES_OK;
}

// CurrentCurvePoint <index> <aram> <szorzo> - beallitja/felulirja a
// gorbe "index"-edik tortespontjat. A pontokat NOVEKVO aram-sorrendben
// kell beallitani (0, 1, 2, ...) - az interpolacio ezt feltetelezi,
// nem rendezi ujra. index 0..MAX_CURVE_POINTS-1.
static commandResult_t CurrentCurvePointCmd(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    int index = Tokenizer_GetArgInteger(0);
    float current = Tokenizer_GetArgFloat(1);
    float mult = Tokenizer_GetArgFloat(2);
    if (index < 0 || index >= MAX_CURVE_POINTS || mult <= 0.0f || isnan(mult)) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "%s",
                     CMD_GetResultString(CMD_RES_BAD_ARGUMENT));
        return CMD_RES_BAD_ARGUMENT;
    }
    curve_current[index] = current;
    curve_mult[index] = mult;
    if (index + 1 > curve_n) curve_n = index + 1;
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                "CurrentCurvePoint: [%d] = (%f A, x%f), curve_n=%d",
                index, current, mult, curve_n);
    return CMD_RES_OK;
}

static commandResult_t CurrentCurveClearCmd(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    curve_n = 0;
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "%s", "CurrentCurveClear: korrekcios gorbe torolve");
    return CMD_RES_OK;
}

static float Scale(float raw, float cal) {
    return (cal_type == PWR_CAL_MULTIPLY ? raw * cal : raw / cal);
}

void PwrCal_Init(pwr_cal_type_t type, float default_voltage_cal,
                 float default_current_cal, float default_power_cal) {
    cal_type = type;

    voltage_cal = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_VOLTAGE,
                                                          default_voltage_cal);
    current_cal = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT,
                                                          default_current_cal);
    power_cal = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_POWER,
                                                        default_power_cal);
    current_offset_raw = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT_OFFSET, 0);

    // Publish the boot-time baseline too, not just future changes - a
    // fresh HA install, or one that missed the retained message, still
    // needs to see what's currently configured.
    PublishCalTopic(CFG_OBK_VOLTAGE, voltage_cal);
    PublishCalTopic(CFG_OBK_CURRENT, current_cal);
    PublishCalTopic(CFG_OBK_POWER, power_cal);
    PublishCalTopic(CFG_OBK_CURRENT_OFFSET, current_offset_raw);

	//cmddetail:{"name":"VoltageSet","args":"Voltage",
	//cmddetail:"descr":"Measure the real voltage with an external, reliable power meter and enter this voltage via this command to calibrate. The calibration is automatically saved in the flash memory.",
	//cmddetail:"fn":"CalibrateVoltage","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("VoltageSet", CalibrateVoltage, NULL);
	//cmddetail:{"name":"CurrentSet","args":"Current",
	//cmddetail:"descr":"Measure the real Current with an external, reliable power meter and enter this Current via this command to calibrate. The calibration is automatically saved in the flash memory.",
	//cmddetail:"fn":"CalibrateCurrent","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentSet", CalibrateCurrent, NULL);
	//cmddetail:{"name":"CurrentZero","args":"",
	//cmddetail:"descr":"Call this with NO load connected (or only negligible load) to capture the current noise-floor/offset and subtract it from all future readings. Run this BEFORE CurrentSet, or re-run CurrentSet afterwards if gain drifts.",
	//cmddetail:"fn":"CalibrateCurrentZero","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentZero", CalibrateCurrentZero, NULL);
	//cmddetail:{"name":"PowerSet","args":"Power",
	//cmddetail:"descr":"Measure the real Power with an external, reliable power meter and enter this Power via this command to calibrate. The calibration is automatically saved in the flash memory.",
	//cmddetail:"fn":"CalibratePower","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("PowerSet", CalibratePower, NULL);
	//cmddetail:{"name":"PowerCalScale","args":"Factor",
	//cmddetail:"descr":"Multiply the EXISTING power calibration constant by the given exact factor, independent of the current live reading. Use this for a small, known correction (e.g. derived from a long-period official-meter comparison) where PowerSet's live-load pivot would be unreliable (load not stable, or shared twin-channel raw value).",
	//cmddetail:"fn":"CalibratePowerScale","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("PowerCalScale", CalibratePowerScale, NULL);
	//cmddetail:{"name":"CurrentCurvePoint","args":"Index Current Multiplier",
	//cmddetail:"descr":"Set one point (index, current in A, multiplier) of the piecewise-linear low-load Current correction curve. Points must be set in increasing-current order (index 0,1,2,...). Only affects the displayed Current (and apparent/reactive power derived from it) - Power and Energy Total are untouched.",
	//cmddetail:"fn":"CurrentCurvePointCmd","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentCurvePoint", CurrentCurvePointCmd, NULL);
	//cmddetail:{"name":"CurrentCurveClear","args":"",
	//cmddetail:"descr":"Clear the low-load Current correction curve (back to no correction, multiplier 1.0 everywhere).",
	//cmddetail:"fn":"CurrentCurveClearCmd","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentCurveClear", CurrentCurveClearCmd, NULL);
}

void PwrCal_PublishPeriodic(void) {
    // Every ~60s - cheap (4 small retained publishes), and guarantees
    // delivery regardless of exactly when MQTT connected after boot.
    static int ticks = 0;
    ticks++;
    if (ticks < 60) return;
    ticks = 0;
    PublishCalTopic(CFG_OBK_VOLTAGE, voltage_cal);
    PublishCalTopic(CFG_OBK_CURRENT, current_cal);
    PublishCalTopic(CFG_OBK_POWER, power_cal);
    PublishCalTopic(CFG_OBK_CURRENT_OFFSET, current_offset_raw);
}

void PwrCal_Scale(int raw_voltage, float raw_current, int raw_power,
                  float *real_voltage, float *real_current, float *real_power) {
    latest_raw_voltage = raw_voltage;
    latest_raw_current = raw_current;
    latest_raw_power = raw_power;

    *real_voltage = Scale(raw_voltage, voltage_cal);
    *real_current = Scale(raw_current - current_offset_raw, current_cal);
    *real_current *= InterpolateCurrentMultiplier(*real_current);
    *real_power = Scale(raw_power, power_cal);
}

float PwrCal_ScalePowerOnly(int raw_power) {
    return Scale(raw_power, power_cal);
}
