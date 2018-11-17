
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "m10.h"
#include "m10private.h"


LPCLIB_Result _M10_processMetrologyBlock (
        const struct _M10_ConfigBlock *rawConfig,
        M10_CookedMetrology *cookedMetro)
{
    cookedMetro->batteryVoltage = rawConfig->vbat * 0.00668f;  /* Factor from SPDXL */

    // Temperature calculation taken from SPDXL
    ;

    return LPCLIB_SUCCESS;
}

