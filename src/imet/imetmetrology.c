
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "imet.h"
#include "imetprivate.h"




/* Read 24-bit little-endian integer from memory */
static uint32_t _IMET_read24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}



LPCLIB_Result _IMET_processPtuFrame (
        const IMET_FramePtu *rawPtu,
        IMET_CookedMetrology *cookedMetro)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    cookedMetro->pressure = _IMET_read24(rawPtu->pressure24) / 100.0f;
    cookedMetro->temperature = rawPtu->temperature / 100.0f;
    cookedMetro->humidity = rawPtu->humidity / 100.0f;
    cookedMetro->batteryVoltage = rawPtu->batteryVoltage / 10.0f;

    /* Some values are not transmitted in the short packet */
    cookedMetro->temperatureInternal = NAN;
    cookedMetro->temperaturePSensor = NAN;
    cookedMetro->temperatureUSensor = NAN;

    return result;
}



LPCLIB_Result _IMET_processPtuxFrame (
        const IMET_FramePtux *rawPtux,
        IMET_CookedMetrology *cookedMetro)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    cookedMetro->frameCounter = rawPtux->packetNumber;
    cookedMetro->pressure = _IMET_read24(rawPtux->pressure24) / 100.0f;
    cookedMetro->temperature = rawPtux->temperature / 100.0f;
    cookedMetro->humidity = rawPtux->humidity / 100.0f;
    cookedMetro->batteryVoltage = rawPtux->batteryVoltage / 10.0f;
    cookedMetro->temperatureInternal = rawPtux->internalTemperature / 100.0f;
    cookedMetro->temperaturePSensor = rawPtux->pressureSensorTemperature / 100.0f;
    cookedMetro->temperatureUSensor = rawPtux->humiditySensorTemperature / 100.0f;

    return result;
}



