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

// Piecewise-linear kis-terheleses Current korrekcios gorbe (2026-10-21,
// csatorankentire bovitve 2026-10-21 kesobb ugyanaznap).
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
//
// A ket fizikai BL0942 chip (device 0 = "Nappali"/klima, device 1 =
// "Lakas teljes") kulon-kulon torzit (megerositve: egy kozos gorbe a
// lakas 0.75-1.00A savjaban tartosan ~60%-os maradek hibat hagyott, mig
// minden mas sav ~10% alatt volt) - ezert ket KULON gorbe-tomb van,
// device-index szerint valasztva.
#define MAX_CURVE_POINTS 20
#define CURVE_DEVICE_COUNT 2
static float curve_current[CURVE_DEVICE_COUNT][MAX_CURVE_POINTS];
static float curve_mult[CURVE_DEVICE_COUNT][MAX_CURVE_POINTS];
static int curve_n[CURVE_DEVICE_COUNT] = {0, 0};

static int ClampDevice(int device) {
    if (device < 0 || device >= CURVE_DEVICE_COUNT) return 0;
    return device;
}

// Ideiglenes (nem perzisztalt, alapertelmezetten KI) kapcsolo: ha BE van
// kapcsolva, minden VALODI meresnel (nem csak valtozaskor - a sima
// Voltage/Current/Power MQTT-publikalas a changeSendThreshold + ~60
// tick-es heartbeat miatt erosen szurt) kipublikalja a Voltage/Current/
// Power nyers (gain+offset kalibralt - Current eseten meg a kis-
// terheleses gorbe ELOTTI) erteket is, csatorankent kulon topicra.
// 2026-10-22: kiterjesztve Voltage-ra es Power-ra is (eredetileg csak
// Current volt) - a user eszrevette, hogy a sima (szurt) Current-
// mintaszam majdnem egyenlo a ket csatornan (mert a heartbeat
// elfedi a kulonbseget), es csak a gate nelkuli current_raw mutatta
// meg a valodi teljesitmeny-kulonbseget (sikeres meresek aranya) a ket
// csatorna kozott - ugyanez
// a torzitas Voltage/Power eseten is fennallhat, szeretnenk azt is latni.
// Apparent/Reactive/PF nem kell kulon, mert mind V/I/P-bol szamolhato.
// Ezzel a korrekcios gorbe elesben maradhat (jo erteket mutat a
// felhasznalonak), mialatt PARHUZAMOSAN tiszta, szures-mentes
// referencia-adat gyulik a HA-ban - nem kell visszafejteni semmit a mar
// alkalmazott szuresen/korrekcion at.
// Szandekosan NEM a /startup_command-ba valo, ideiglenes hangolasi
// eszkoz ("amig beall a rendszer") - manualisan kapcsolando ki, ha mar
// nem kell.
static int publish_raw_current = 0;

static commandResult_t CurrentRawPublishCmd(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    publish_raw_current = (Tokenizer_GetArgInteger(0) != 0);
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "CurrentRawPublish: %s",
                publish_raw_current ? "ON" : "OFF");
    return CMD_RES_OK;
}

static float InterpolateCurrentMultiplier(int device, float real_current) {
    device = ClampDevice(device);
    int n = curve_n[device];
    if (n == 0) return 1.0f;
    if (real_current <= curve_current[device][0]) return curve_mult[device][0];
    if (real_current >= curve_current[device][n - 1]) return curve_mult[device][n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (real_current >= curve_current[device][i] && real_current <= curve_current[device][i + 1]) {
            float span = curve_current[device][i + 1] - curve_current[device][i];
            if (span <= 0.0f) return curve_mult[device][i];
            float frac = (real_current - curve_current[device][i]) / span;
            return curve_mult[device][i] + (curve_mult[device][i + 1] - curve_mult[device][i]) * frac;
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

// CurrentCurvePoint <device> <index> <aram> <szorzo> - beallitja/felulirja
// a "device" (0=Nappali/klima, 1=Lakas teljes) csatorna gorbejenek
// "index"-edik tortespontjat. A pontokat NOVEKVO aram-sorrendben kell
// beallitani (index 0, 1, 2, ...) csatorankent - az interpolacio ezt
// feltetelezi, nem rendezi ujra. index 0..MAX_CURVE_POINTS-1.
static commandResult_t CurrentCurvePointCmd(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 4)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    int device = Tokenizer_GetArgInteger(0);
    int index = Tokenizer_GetArgInteger(1);
    float current = Tokenizer_GetArgFloat(2);
    float mult = Tokenizer_GetArgFloat(3);
    if (device < 0 || device >= CURVE_DEVICE_COUNT || index < 0 ||
        index >= MAX_CURVE_POINTS || mult <= 0.0f || isnan(mult)) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "%s",
                     CMD_GetResultString(CMD_RES_BAD_ARGUMENT));
        return CMD_RES_BAD_ARGUMENT;
    }
    curve_current[device][index] = current;
    curve_mult[device][index] = mult;
    if (index + 1 > curve_n[device]) curve_n[device] = index + 1;
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER,
                "CurrentCurvePoint: dev %d [%d] = (%f A, x%f), curve_n=%d",
                device, index, current, mult, curve_n[device]);
    return CMD_RES_OK;
}

// CurrentCurveClear <device> - torli a megadott csatorna gorbejet (vissza
// "nincs korrekcio"-ra). A masik csatorna gorbeje erintetlen marad.
static commandResult_t CurrentCurveClearCmd(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    int device = Tokenizer_GetArgInteger(0);
    if (device < 0 || device >= CURVE_DEVICE_COUNT) {
        ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "%s",
                     CMD_GetResultString(CMD_RES_BAD_ARGUMENT));
        return CMD_RES_BAD_ARGUMENT;
    }
    curve_n[device] = 0;
    ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "CurrentCurveClear: dev %d gorbeje torolve", device);
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
	//cmddetail:{"name":"CurrentCurvePoint","args":"Device Index Current Multiplier",
	//cmddetail:"descr":"Set one point (index, current in A, multiplier) of the piecewise-linear low-load Current correction curve FOR ONE DEVICE (0=primary/Nappali, 1=twin-mode secondary/Lakas - the two physical BL0942 chips can distort differently). Points must be set in increasing-current order per device (index 0,1,2,...). Only affects the displayed Current (and apparent/reactive power derived from it) - Power and Energy Total are untouched.",
	//cmddetail:"fn":"CurrentCurvePointCmd","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentCurvePoint", CurrentCurvePointCmd, NULL);
	//cmddetail:{"name":"CurrentCurveClear","args":"Device",
	//cmddetail:"descr":"Clear the low-load Current correction curve for the given device (0 or 1) - back to no correction, multiplier 1.0 everywhere for that device only.",
	//cmddetail:"fn":"CurrentCurveClearCmd","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentCurveClear", CurrentCurveClearCmd, NULL);
	//cmddetail:{"name":"CurrentRawPublish","args":"0Or1",
	//cmddetail:"descr":"Toggle publishing RAW Voltage/Current/Power (gain+offset calibrated; Current also BEFORE the low-load curve) to MQTT (voltage_raw/current_raw/power_raw, +_b for device 1), on EVERY successful measurement - not gated by changeSendThreshold/heartbeat like the normal Voltage/Current/Power topics, and not tied to webpage renders like current_calc_pv. Temporary tuning/debug aid: lets the curve stay active (good displayed values) while clean, filter-free reference data accumulates in parallel. Not persisted across reboot - turn off manually when done.",
	//cmddetail:"fn":"CurrentRawPublishCmd","file":"driver/drv_pwrCal.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("CurrentRawPublish", CurrentRawPublishCmd, NULL);
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

// Beltart/single-device driverek (bl0937, cse7766, rn8209) ezt hivjak,
// device-index nelkul - ezek sosem twin-mode-ban futnak, a device 0
// gorbejet hasznaljak (nincs masik csatorna, aminek kulon gorbe kellene).
void PwrCal_Scale(int raw_voltage, float raw_current, int raw_power,
                  float *real_voltage, float *real_current, float *real_power) {
    PwrCal_ScaleForDevice(0, raw_voltage, raw_current, raw_power,
                          real_voltage, real_current, real_power);
}

// A ket fizikai BL0942 chip (vagy barmely jovobeli twin-mode driver)
// KULON Current-korrekcios gorbet hasznal - l. a fajl elejen levo
// megjegyzest arrol, miert (a ket chip kulon-kulon torzit). A gain/offset
// kalibracio (voltage_cal/current_cal/power_cal/current_offset_raw)
// TOVABBRA IS kozos/globalis - csak a kis-terheleses korrekcios gorbe
// csatorankenti.
void PwrCal_ScaleForDevice(int device, int raw_voltage, float raw_current, int raw_power,
                  float *real_voltage, float *real_current, float *real_power) {
    latest_raw_voltage = raw_voltage;
    latest_raw_current = raw_current;
    latest_raw_power = raw_power;

    *real_voltage = Scale(raw_voltage, voltage_cal);
    *real_current = Scale(raw_current - current_offset_raw, current_cal);
    *real_power = Scale(raw_power, power_cal);
    if (publish_raw_current) {
        MQTT_PublishMain_StringFloat(
            device == 0 ? "voltage_raw" : "voltage_raw_b",
            *real_voltage, 1, OBK_PUBLISH_FLAG_QOS_ZERO);
        MQTT_PublishMain_StringFloat(
            device == 0 ? "current_raw" : "current_raw_b",
            *real_current, 3, OBK_PUBLISH_FLAG_QOS_ZERO);
        MQTT_PublishMain_StringFloat(
            device == 0 ? "power_raw" : "power_raw_b",
            *real_power, 2, OBK_PUBLISH_FLAG_QOS_ZERO);
    }
    *real_current *= InterpolateCurrentMultiplier(device, *real_current);
}

float PwrCal_ScalePowerOnly(int raw_power) {
    return Scale(raw_power, power_cal);
}
