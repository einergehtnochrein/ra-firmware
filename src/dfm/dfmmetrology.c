
#include "lpclib.h"
#include "dfm.h"
#include "dfmprivate.h"


/* Process a frame with meteorological measurements. */
LPCLIB_Result _DFM_processMetrologyBlock (
        const DFM_SubFrameConfig *rawConfig,
        DFM_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    /* Reference values follow the fast analog channels, but minimum channel number is 3. */
    uint8_t firstRefChannel = 3;
    if (instance->numAnalog > 3) {
        firstRefChannel = instance->numAnalog;
    }

    /* Determine current channel number */
    uint8_t channel = rawConfig->type;

    /* Analog or reference channel? */
    if (channel <= firstRefChannel + 1) {
        /* Convert raw value to float */
        uint32_t mantissa = 0
                | (rawConfig->h[1] << 16)
                | (rawConfig->h[2] << 12)
                | (rawConfig->h[3] <<  8)
                | (rawConfig->h[4] <<  4)
                | (rawConfig->h[5] <<  0)
                ;
        uint32_t exponent = (1u << rawConfig->h[0]);
        float f = (float)mantissa / (float)exponent;

        /* Reference value? */
        if (channel == firstRefChannel + 0) {
            instance->metro._ref3 = f;
        }
        else if (channel == firstRefChannel + 1) {
            instance->metro._ref4 = f;
        }
        else if (channel == 0) {
            /* Temperature calculation follows the reasoning given by user Zilog80 in this thread:
             * http://www.fingers-welt.de/phpBB/viewtopic.php?f=14&t=43&start=2325#p196322
             */
            if (!isnan(instance->metro._ref3) && !isnan(instance->metro._ref4)) {
                f = (f - instance->metro._ref3) / instance->metro._ref4 * 220.0f/5.0f;
                instance->metro.temperature = 1.0f / (logf(f) / 3450.0f + 1.0f / (273.15f + 25.0f)) - 273.15f;
            }
        }
    }
    else {
        /* For channels above the reference channels:
         * Split the 6 nibble payload into its three constituents.
         */
//        uint8_t subChannel = rawConfig->h[0];
        uint16_t u16value = 0
                | (rawConfig->h[1] << 12)
                | (rawConfig->h[2] <<  8)
                | (rawConfig->h[3] <<  4)
                | (rawConfig->h[4] <<  0)
                ;
//        uint8_t fragment = rawConfig->h[5];

        if (instance->platform == SONDE_DFM06) {
            instance->metro.batteryVoltage = NAN;
            instance->metro.cpuTemperature = NAN;

            switch (channel - (int)instance->maxConfigChannel) {
                case DFM06_CHANNEL_CONFIG_NAME:
                    /* Ignored once detected */
                    break;
            }
        }
        if (instance->platform == SONDE_DFM09) {
            switch (channel - (int)instance->maxConfigChannel) {
                case DFM09_CHANNEL_CONFIG_BATTERY_VOLTAGE:
                    instance->metro.batteryVoltage = u16value / 1000.0f;
                    break;
                case DFM09_CHANNEL_CONFIG_CPU_TEMPERATURE:
                    instance->metro.cpuTemperature = u16value / 100.0f - 273.16f;
                    break;
                case DFM09_CHANNEL_CONFIG_NAME:
                    /* Ignored once detected */
                    break;
            }
        }
    }

    return result;
}


