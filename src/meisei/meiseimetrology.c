
#include "lpclib.h"
#include "meisei.h"
#include "meiseiprivate.h"



/* Process a frame with meteorological measurements. */
static LPCLIB_Result _MEISEI_processMetrology_IMS100 (
        MEISEI_InstanceData *instance)
{
    float f;
    float temperature = NAN;
    float humidity = NAN;
    float rh_temperature = instance->metro.rh_temperature;

    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    uint8_t fragment = instance->frameCounter % 64;

    /* Reference frequency value */
    if ((fragment / 2) % 2 == 0) {
        instance->refFreq = instance->configPacketEven.w[1];
    }

    /* Main temperature and humidity calculations are based in part on description found in
     * the GRUAN document "Technical characteristics and GRUAN data processing for the Meisei
     * RS-11G and iMS-100 radiosondes", Rev 1.0 (2018-02-21):
     * https://www.gruan.org/gruan/editor/documents/gruan/GRUAN-TD-5_MeiseiRadiosondes_v1_20180221.pdf
     */

    /* Main temperature sensor */
    if (_MEISEI_checkValidCalibration(instance, CALIB_IMS100_MAIN_TEMPERATURE)) {
        if (!isnan(instance->refFreq)) {
            /* See GRUAN document, part B */
            f = instance->configPacketEven.w[5];
            f = f / instance->refFreq * 4.0f;
            if (f > 1.0f) {     /* Sanity check */
                f = 1.0f / (f - 1.0f);
                /* Calculate sensor resistance (kOhms).
                 * GRUAN TD-5 uses four coefficients A[3:0] for this. However, only a reduced version
                 * with A[2:0] produces satisfying results.
                 * TODO: What is the orphaned coefficient used for?
                 */
                f = instance->config[53] - instance->config[56]
                  + instance->config[54] * f
                  + instance->config[55] * f*f
                  ;

                /* Detect a broken sensor. Set an arbitrary limit of 2.5 MOhm.
                 * On a typical sonde this would correspond to a temperature far below -100°C.
                 */
                instance->metro.temperatureSensorBroken = (f >= 2500.0f);

                if (!instance->metro.temperatureSensorBroken) {
                    /* Get temperature from resistance.
                     * Sonde sends sampling points for cubic splines, but here we just use linear
                     * interpolation with very small error.
                     */
                    if (f <= instance->config[33]) {
                        temperature = instance->config[17];
                    } else if (f >= instance->config[44]) {
                        temperature = instance->config[28];
                    } else {
                        /* Table has the resistance values. For linear interpolation we take the logarithm. */
                        for (int i = 0; i < 11; i++) {
                            if (f < instance->config[34 + i]) {
                                f = (logf(f) - logf(instance->config[33 + i]))
                                  / (logf(instance->config[34 + i]) - logf(instance->config[33 + i]));
                                temperature = instance->config[17 + i]
                                    - f * (instance->config[17 + i] - instance->config[18 + i]);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    /* Humidity sensor temperature */
    if (_MEISEI_checkValidCalibration(instance, CALIB_IMS100_RH_TEMPERATURE)) {
        if (fragment % 4 == 3) {
            if (!isnan(instance->refFreq)) {
                /* See GRUAN TD-5, p.129 G.11 */
                f = instance->configPacketOdd.w[1];
                f = f / instance->refFreq * 4.0f;
                if (f > 1.0f) {     /* Sanity check */
                    f = 1.0f / (f - 1.0f);
                    /* Calculate sensor resistance (kOhms).
                     * GRUAN TD-5 (see main temperature sensor above)
                     */
                    f = instance->config[53] - instance->config[56]
                      + instance->config[54] * f
                      + instance->config[55] * f*f
                      ;

                    /* GRUAN TD-5 p.129 G.13 */
                    f = logf(f);
                    f = instance->config[57]*f*f*f + instance->config[58]*f + instance->config[59];
                    rh_temperature = 1.0f / f - 273.15f;
                }
            }
        }
    }

    //TODO effective humidity sensor temperature (requires pressure)

    /* Relative humidity */
    if (_MEISEI_checkValidCalibration(instance, CALIB_HUMIDITY)) {
        if (!isnan(instance->refFreq)) {
            /* See GRUAN document, part D */
            f = instance->configPacketEven.w[6];
            f = f / instance->refFreq * 4.0f;
            humidity = 0.0f
                    + instance->config[49]
                    + instance->config[50] * f
                    + instance->config[51] * f*f
                    + instance->config[52] * f*f*f
                    ;
            humidity = fmaxf(humidity, 0.0f);
            humidity = fminf(humidity, 100.0f);
        }
    }

    //TODO aux temperature sensor @ humidity sensor
    //TODO temperature correction for humidity
    //TODO radiation correction for main temperature sensor? (GRUAN, part C)

    /* Radio temperature */
    if ((fragment / 2) % 2 == 0) {
        instance->metro.txTemperature = (instance->configPacketEven.w[9] % 256) * 0.5f - 64.0f;
    } else {
        instance->metro.ana8_unknown = instance->configPacketEven.w[9] % 256;
    }

    /* CPU temperature */
    //TODO Broken! While the 1.05V bias seems to be fine, the gain (-3.6mV/°C) is definitely higher!
    if ((fragment / 2) % 2 == 0) {
        /* ADC reading scaled to 16 bits, full scale = 3.3V. */
        f = instance->configPacketOdd.w[8] / 65536.0f * 3.3f;
        /* RL78/G13 data: 25°C = 1.05V, -3.6mV/°C */
//            instance->metro.cpuTemperature = 25.0f - (f - 1.05f) / 0.0036f;
instance->metro.cpuTemperature = 25.0f - (f - 1.05f) / 0.036f;  // Arbitrary gain factor = 10
    }

    instance->metro.temperature = temperature;
    instance->metro.humidity = humidity;
    instance->metro.rh_temperature = rh_temperature;

    return result;
}



/* Process a frame with meteorological measurements. */
static LPCLIB_Result _MEISEI_processMetrology_RS11G (
        MEISEI_InstanceData *instance)
{
    float f;
    float temperature = NAN;
    float humidity = NAN;

    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    uint8_t fragment = instance->frameCounter % 64;

    /* Reference frequency value */
    if ((fragment / 2) % 2 == 0) {
        instance->refFreq = instance->configPacketEven.w[1];
    }

    /* Main temperature and humidity calculations are based in part on description found in
     * the GRUAN document "Technical characteristics and GRUAN data processing for the Meisei
     * RS-11G and iMS-100 radiosondes", Rev 1.0 (2018-02-21):
     * https://www.gruan.org/gruan/editor/documents/gruan/GRUAN-TD-5_MeiseiRadiosondes_v1_20180221.pdf
     */

    /* Main temperature sensor */
    if (_MEISEI_checkValidCalibration(instance, CALIB_RS11G_MAIN_TEMPERATURE)) {
        if (!isnan(instance->refFreq)) {
            /* See GRUAN document, part B */
            f = instance->configPacketEven.w[5];
            f = f / instance->refFreq * 4.0f;
            if (f > 1.0f) {     /* Sanity check */
                f = 1.0f / (f - 1.0f);
                /* Calculate sensor resistance (kOhms).
                 * GRUAN TD-5 uses four coefficients A[3:0] for this. However, only a reduced version
                 * with A[2:0] produces satisfying results.
                 * TODO: What is the orphaned coefficient used for?
                 */
                f = instance->config[33] - instance->config[36]
                  + instance->config[34] * f
                  + instance->config[35] * f*f
                  ;

                /* Detect a broken sensor. Set an arbitrary limit of 2.5 MOhm.
                 * On a typical sonde this would correspond to a temperature far below -100°C.
                 */
                instance->metro.temperatureSensorBroken = (f >= 2500.0f);

                if (!instance->metro.temperatureSensorBroken) {
                    /* Get temperature from resistance.
                     * Sonde sends sampling points for cubic splines, but here we just use linear
                     * interpolation with very small error.
                     */
                    if (f <= instance->config[37]) {
                        temperature = instance->config[17];
                    } else if (f >= instance->config[47]) {
                        temperature = instance->config[27];
                    } else {
                        /* Table has the resistance values. For linear interpolation we take the logarithm. */
                        for (int i = 0; i < 10; i++) {
                            if (f < instance->config[38 + i]) {
                                f = (logf(f) - logf(instance->config[37 + i]))
                                  / (logf(instance->config[38 + i]) - logf(instance->config[37 + i]));
                                temperature = instance->config[17 + i]
                                    - f * (instance->config[17 + i] - instance->config[18 + i]);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    //TODO effective humidity sensor temperature (requires pressure)

    /* Relative humidity */
    if (_MEISEI_checkValidCalibration(instance, CALIB_HUMIDITY)) {
        if (!isnan(instance->refFreq)) {
            /* See GRUAN document, part D */
            f = instance->configPacketEven.w[6];
            f = f / instance->refFreq * 4.0f;
            humidity = 0.0f
                    + instance->config[49]
                    + instance->config[50] * f
                    + instance->config[51] * f*f
                    + instance->config[52] * f*f*f
                    ;
            humidity = fmaxf(humidity, 0.0f);
            humidity = fminf(humidity, 100.0f);
        }
    }

    instance->metro.temperature = temperature;
    instance->metro.humidity = humidity;
    instance->metro.txTemperature = NAN;
    instance->metro.cpuTemperature = NAN;

    return result;
}



/* Process a frame with meteorological measurements. */
LPCLIB_Result _MEISEI_processMetrology (
        MEISEI_InstanceData *instance)
{
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (instance->model == MEISEI_MODEL_IMS100) {
        result = _MEISEI_processMetrology_IMS100(instance);
    }
    else if (instance->model == MEISEI_MODEL_RS11G) {
        result = _MEISEI_processMetrology_RS11G(instance);
    }

    return result;
}
