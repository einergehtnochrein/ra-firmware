
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "psb3.h"
#include "psb3private.h"
#include "observer.h"
#include "rinex.h"


/* Helper to read from BCD coded fields. No check is done if the number is a valid BCD number! */
static uint32_t _PSB3_readBCD (uint8_t bcd)
{
    return 10 * (bcd / 16) + (bcd % 16);
}


LPCLIB_Result _PSB3_processPayload (
        const PSB3_Packet *payload,
        PSB3_InstanceData *instance)
{
    LLA_Coordinate lla;

    /* GPS */
    lla.lat = NAN;
    lla.lon = NAN;
    lla.alt = NAN;
    lla.climbRate = NAN;
    lla.velocity = NAN;
    lla.direction = NAN;

    float altitude = 1000.0f * _PSB3_readBCD(payload->altitude[0])
                   +   10.0f * _PSB3_readBCD(payload->altitude[1])
                   +    0.1f * _PSB3_readBCD(payload->altitude[2])
                   ;
    if (altitude > -100.0) {
        lla.alt = altitude;

        double latitude =             _PSB3_readBCD(payload->latitude[0])
                        +  1.0/60.0 * _PSB3_readBCD(payload->latitude[1])
                        + 0.01/60.0 * _PSB3_readBCD(payload->latitude[2])
                        + 1e-4/60.0 * _PSB3_readBCD(payload->latitude[3])
                        ;
        lla.lat = latitude * (M_PI / 180.0);

        double longitude =      10.0 * _PSB3_readBCD(payload->longitude[0])
                         +       1.0 * (_PSB3_readBCD(payload->longitude[1]) / 10)
                         + 10.0/60.0 * (_PSB3_readBCD(payload->longitude[1]) % 10)
                         +  0.1/60.0 * _PSB3_readBCD(payload->longitude[2])
                         + 1e-3/60.0 * _PSB3_readBCD(payload->longitude[3])
                         + 1e-4/60.0 * (_PSB3_readBCD(payload->longitude[4]) / 10)
                         ;
        lla.lon = longitude * (M_PI / 180.0f);

        GPS_applyGeoidHeightCorrection(&lla);
    }
    instance->gps.observerLLA = lla;

    /* PTU */

    /* ADC conversion results (temperature/humidity) are 16-bit results (65536 or 1.25V full-scale).
     *
     * Temperature: NTC with 56k2 series bias resistor @ 1.25V
     *              Resistance R = raw / 65536 - raw) * 56k2
     *              Temperature 1/T = 1/T25 + 1/B * ln (R/R25)
     *              B and R25 guessed from UAII2022 flights in Lindenberg
     * Humidity: HIH-4021 sensor with 5.0V supply, uncompensated RH = (Vout/5.0 - 0.16) / 0.0062
     *
     * TODO: Better guess for NTC R25/B?
     *       RH sensor temperature correction?
     */
    float R = payload->rawTemperature / (65536.0f - payload->rawTemperature) * 56.2e3f;
    const float B = 3000.0f;
    const float R25 = 2000.0f;
    instance->metro.temperature = 1.0f / (1.0f/(273.15f+25.0f) + 1/B*logf(R/R25));
    instance->metro.humidity = ((payload->rawHumidity / 65536.0f * 1.25f) / 5.0f - 0.16f) / 0.0062f;

    return LPCLIB_SUCCESS;
}

