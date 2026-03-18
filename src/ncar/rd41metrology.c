
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "ncar.h"
#include "ncarprivate.h"


LPCLIB_Result _RD41_processMetrologyBlock (
        const RD41_SubFrameMetrology *rawMetro,
        NCAR_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    instance->metro.pressure = rawMetro->pressure;
    instance->metro.T = rawMetro->temperature;

    return LPCLIB_SUCCESS;
}

