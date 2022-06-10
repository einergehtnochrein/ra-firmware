
#include "lpclib.h"
#include "meisei.h"
#include "meiseiprivate.h"



/* Process a frame with meteorological measurements. */
LPCLIB_Result _MEISEI_processMetrology (
        MEISEI_InstanceData *instance)
{
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
        //TODO
    }

    instance->metro.temperature = temperature;
    instance->metro.humidity = humidity;
    instance->metro.cpuTemperature = cpuTemperature;

    return result;
}


