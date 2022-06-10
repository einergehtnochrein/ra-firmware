
#include "lpclib.h"
#include "meisei.h"
#include "meiseiprivate.h"



/* Process a frame with meteorological measurements. */
LPCLIB_Result _MEISEI_processMetrology (
        MEISEI_InstanceData *instance)
{
    float f;
    float temperature = NAN;
    float humidity = NAN;
    float cpuTemperature = NAN;

    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    if (instance->model == MEISEI_MODEL_RS11G) {
        //TODO
    }
    else if (instance->model == MEISEI_MODEL_IMS100) {
        uint8_t fragment = instance->frameCounter % 64;

        /* Reference frequency value */
        if ((fragment / 2) % 2 == 0) {
            instance->refFreq = _MEISEI_getPayloadHalfWord(instance->configPacketEven.fields, 1);
        }

        /* Relative humidity */
        if (_MEISEI_checkValidCalibration(instance, CALIB_HUMIDITY)) {
            if (!isnan(instance->refFreq)) {
                f = _MEISEI_getPayloadHalfWord(instance->configPacketEven.fields, 6);
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

        /* Radio temperature */
        if ((fragment / 2) % 2 == 0) {
            instance->metro.txTemperature =
                (_MEISEI_getPayloadHalfWord(instance->configPacketEven.fields, 9) % 256) * 0.5f - 64.0f;
        }
    }

    instance->metro.temperature = temperature;
    instance->metro.humidity = humidity;
    instance->metro.cpuTemperature = cpuTemperature;

    return result;
}


