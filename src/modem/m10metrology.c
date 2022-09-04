
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "m10.h"
#include "m10private.h"


LPCLIB_Result _M10_processMetrologyBlock (
        const struct _M10_ConfigBlock *rawConfig,
        M10_CookedMetrology *cookedMetro)
{
    cookedMetro->batteryVoltage = rawConfig->adc_vbat * 0.00668f;  /* Factor from SPDXL */

    //TODO Temperature calculation follows solution in SPDXL. --> Must see M10 schematic to explain
    cookedMetro->temperature = NAN;
    if (rawConfig->adc_temperature_range < 3) {
        const float Rs[3] = {12.1e3f, 36.5e3f, 475e3f};
        const float Rp[3] = {1e20f, 330e3f, 3e6f};
        int32_t adcReadout = rawConfig->adc_temperature - 0xA000;
        if (adcReadout > 0) {
            float x = (4095.0f - adcReadout) / adcReadout;
            x = x - Rs[rawConfig->adc_temperature_range] / Rp[rawConfig->adc_temperature_range];
            if (x != 0) {
                x = Rs[rawConfig->adc_temperature_range] / x;
                if (x > 0) {
                    x = logf(x);

                    //TODO Polynomial coefficients for T(R) curve fit
                    const float a[4] = {
                        1.07303516e-3f,
                        2.41296733e-4f,
                        2.26744154e-6f,
                        6.52855181e-8f
                    };
                    x = a[0] + a[1]*x + a[2]*x*x + a[3]*x*x*x;
                    if (x != 0) {
                        cookedMetro->temperature = 1.0f / x - 273.16f;
                    }
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}

