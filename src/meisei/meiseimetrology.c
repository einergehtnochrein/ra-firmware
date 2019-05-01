
#include "lpclib.h"
#include "meisei.h"
#include "meiseiprivate.h"



/* Process a frame with meteorological measurements. */
LPCLIB_Result _MEISEI_processMetrology (
        MEISEI_InstanceData *instance)
{
    float f;

    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    uint8_t fragment = instance->frameCounter % 64;

    if (instance->model == MEISEI_MODEL_RS11G) {
        instance->metro.temperature = NAN;
    }
    else if (instance->model == MEISEI_MODEL_IMS100) {
        /* Temperature is sent in every fragment (twice per second).
        * We only send the temperature once per second! */
        //TODO Arbitrary calculation... (PTC, alpha/2*beta=213.6, beta=1.9e-5, R0=35917)
        //TODO Definitely wrong!
        f = _MEISEI_getPayloadHalfWord(instance->configPacketOdd.fields, 5) / 35917.0f - 1.0f;
        f = 45625.0f + f / 1.9e-5f;
        if (f >= 0) {
            instance->metro.temperature = -188.6f + sqrtf(f);
        }
        else {
            instance->metro.temperature = NAN;
        }
        
        /* Handle values that are only sent in certain fragments */
        if ((fragment % 4) == 1) {
            //TODO this is just the raw value
            instance->metro.cpuTemperature = _MEISEI_getPayloadHalfWord(instance->configPacketOdd.fields, 8);
        }
    }
    else {
        instance->metro.temperature = NAN;
    }

    return result;
}


